# Technical Reference: av_stack — A Modular Autonomous Driving Stack

> Japanese version: [`../docs_ja/TECHNICAL.md`](../docs_ja/TECHNICAL.md)

> **Reading level:** This document assumes familiarity with state-space control theory,
> Bayesian filtering, and convex optimisation. The goal is to give a reader the
> mathematical foundation to reproduce, extend, or audit every algorithm in the stack —
> not merely to describe what the code does.

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Vehicle Dynamics — Kinematic Bicycle Model](#2-vehicle-dynamics--kinematic-bicycle-model)
3. [Linear Algebra Backend](#3-linear-algebra-backend)
4. [Control Layer](#4-control-layer)
   - 4.1 [PID with Anti-Windup](#41-pid-with-anti-windup)
   - 4.2 [Discrete LQR via DARE](#42-discrete-lqr-via-dare)
   - 4.3 [Linear Time-Varying MPC with FISTA](#43-linear-time-varying-mpc-with-fista)
5. [Sensor Models](#5-sensor-models)
6. [Perception — Information-Form Sensor Fusion](#6-perception--information-form-sensor-fusion)
7. [World Model — IMM Kalman Tracker](#7-world-model--imm-kalman-tracker)
8. [Motion Planning](#8-motion-planning)
9. [Safety Guardrail — RSS Monitor](#9-safety-guardrail--rss-monitor)
10. [Probabilistic Occupancy Prediction](#10-probabilistic-occupancy-prediction)
11. [Verification and Numerical Results](#11-verification-and-numerical-results)
12. [References](#12-references)

---

## 1. System Architecture

The stack is organised as four concentric layers, ordered by increasing complexity and
decreasing safety-criticality:

```
Sensors (LiDAR / Radar / Camera)
       │
       ▼
┌──────────────────────────────────────────────────────────────────┐
│  Perception  ·  sensors.py → perception.py                       │
│  Information-form fusion over anisotropic per-sensor covariances │
└─────────────────────────────┬────────────────────────────────────┘
                              │ FusedDetection (z, R, class, v_prior)
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  World Model  ·  tracking.hpp (C++, pybind11)                    │
│  IMM Kalman filter per track; greedy Mahalanobis association      │
│  Constant-velocity prediction → predicted trajectories            │
└─────────────────────────────┬────────────────────────────────────┘
                              │ confirmed tracks + predictions
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  Planner  ·  planning.py  +  occupancy.py                        │
│  IDM longitudinal · smoothstep lateral · multi-lane cost search  │
│  Probabilistic occupancy grid for soft path cost                 │
└─────────────────────────────┬────────────────────────────────────┘
                              │ Trajectory (x,y,v,κ)
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  Safety Guardrail  ·  guardrail.py  (independent monitor)        │
│  RSS longitudinal · TTC · RSS lateral                            │
└─────────────────────────────┬────────────────────────────────────┘
                              │ (a, δ) or (−b_emergency, δ_hold)
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│  Controller  ·  mpc.hpp / lqr.hpp (C++, pybind11)               │
│  LTV-MPC or Lateral LQR + longitudinal PID                       │
└─────────────────────────────┬────────────────────────────────────┘
                              │ actuator commands → bicycle plant
                              ▼
                         Vehicle (bicycle.hpp, RK4)
```

**Separation of concerns.** The safety guardrail is deliberately placed *after* the
planner and *before* the controller. It reads directly from the world model — not from
the planner — so a planning failure cannot compromise it. This mirrors the
doer–checker pattern mandated in IEC 61508 / ISO 26262 for SIL 2+ monitors.

---

## 2. Vehicle Dynamics — Kinematic Bicycle Model

### 2.1 Continuous-time model

The kinematic bicycle collapses the four-wheeled vehicle to a single rear axle and
one steering angle at the front axle. State $s = [x, y, \psi, v]^\top \in \mathbb{R}^4$, input
$u = [a, \delta]^\top \in \mathbb{R}^2$:

$$\begin{aligned}
\dot{x} &= v \cos \psi \\
\dot{y} &= v \sin \psi \\
\dot{\psi} &= (v / L) \tan \delta \\
\dot{v} &= a
\end{aligned}$$

where $L = 2.7$ m is the wheelbase. The model is valid at low-to-moderate slip
angles; for highway speeds below tyre-limit conditions the kinematic assumption
holds to within a few percent of a dynamic (force-based) model.

### 2.2 Numerical integration — Runge–Kutta 4

The discrete step $s_{k+1} = \Phi(s_k, u_k, dt)$ is computed with the classical
fourth-order Runge–Kutta method:

$$\begin{aligned}
k_1 &= f(s_k, u_k) \\
k_2 &= f(s_k + (dt/2) k_1,\ u_k) \\
k_3 &= f(s_k + (dt/2) k_2,\ u_k) \\
k_4 &= f(s_k + dt\, k_3,\ u_k) \\
s_{k+1} &= s_k + \tfrac{dt}{6}(k_1 + 2k_2 + 2k_3 + k_4)
\end{aligned}$$

**Why RK4 and not Euler?** The local truncation error of Euler is $O(dt^2)$; for
$dt = 0.1$ s and $v = 13$ m/s the yaw-rate term $v/L \cdot \tan \delta$ accumulates roughly
0.5 m/s yaw-rate error per second. RK4 is $O(dt^5)$ locally and reduces this to the
sub-millimetre range, which matters because the MPC prediction horizon spans 15
steps (1.5 s) and an inaccurate prediction model systematically biases the optimal
input.

### 2.3 Discrete-time Jacobians by central finite differences

For the MPC and LQR linearisation, we need the discrete-time Jacobians:

$$\begin{aligned}
A_k &= \partial \Phi / \partial s \,\big|_{(s_k, u_k)} \in \mathbb{R}^{4\times 4} \\
B_k &= \partial \Phi / \partial u \,\big|_{(s_k, u_k)} \in \mathbb{R}^{4\times 2}
\end{aligned}$$

These are computed numerically by central finite differences:

$$\begin{aligned}
A_k(i,j) &= \big[ \Phi(s_k + \varepsilon e_j, u_k) - \Phi(s_k - \varepsilon e_j, u_k) \big]_i / (2\varepsilon) \\
B_k(i,j) &= \big[ \Phi(s_k, u_k + \varepsilon e_j) - \Phi(s_k, u_k - \varepsilon e_j) \big]_i / (2\varepsilon)
\end{aligned}$$

with $\varepsilon = 10^{-6}$. The key advantage over analytical Jacobians is **consistency**: the
finite-difference Jacobian is guaranteed to be consistent with the same RK4 step
that the plant uses, so the LTV error model $e_{k+1} \approx A_k e_k + B_k \Delta u_k + d_k$ has
no modelling mismatch. Analytical Jacobians require deriving the chain rule through
the RK4 stages, which introduces sign errors and is invalidated whenever the
integration method changes.

**Verification.** For a typical operating point $(s_0, u_0)$ the linearisation
residual satisfies:

$$\lVert \Phi(s_0 + \delta s, u_0) - (\Phi(s_0, u_0) + A_k \delta s) \rVert / \lVert \delta s \rVert \approx 2.3 \times 10^{-6} \quad \text{(second-order)}$$

confirming that the Jacobians are correct to the expected $O(\varepsilon^2)$ accuracy of central
differences.

---

## 3. Linear Algebra Backend

All numerical work uses a minimal header-only dense matrix library (`linalg.hpp`)
with no external dependencies (no Eigen, BLAS, or LAPACK). The design trade-off:
portability and zero setup cost in exchange for $O(n^3)$ dense algorithms, acceptable
because all matrices are at most ~$`30 \times 30`$ ($N \cdot m = 30$ for the condensed MPC QP with
$N = 15$, $m = 2$).

**`solve(A, b)` — Gauss–Jordan with partial pivoting.** For each column $c$:

1. Find the pivot row $p = \operatorname*{argmax}_{i \ge c} |A(i,c)|$.
2. Swap rows $c$ and $p$ in $A$ and $b$.
3. Divide row $c$ by the diagonal entry.
4. Eliminate column $c$ from all other rows.

A small regularisation $A(c,c) \mathrel{+}= 10^{-9}$ is added if the pivot is below $10^{-12}$
to handle near-singular cases gracefully (this occurs for the DARE at very early
iterations when $P$ is still close to $Q$).

**`inv(A)` = `solve(A, I)`.** Used only for tracker innovation covariance inversion
($2\times2$ matrices), so the cost is negligible.

---

## 4. Control Layer

### 4.1 PID with Anti-Windup

Standard three-term controller with integral clamping:

$$u(t) = K_p\, e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d\, \dot{e}(t)$$

Discretised with Euler integration for the integral and a backward difference for
the derivative. Two saturation levels are applied independently:

- **Integral clamp** `[i_min, i_max]`: limits the accumulated integral before it
  feeds into the sum, preventing windup when the actuator saturates.
- **Output clamp** `[out_min, out_max]`: limits the final command.

This is the classical *conditional anti-windup* scheme (Åström & Wittenmark §3.5).
The integral is clamped before summing, not after, so the windup is prevented
rather than corrected.

### 4.2 Discrete LQR via DARE

**Problem.** Given the discrete time-invariant linear system $e_{k+1} = A e_k + B u_k$,
find the state-feedback gain $K$ that minimises the infinite-horizon quadratic cost:

$$J = \sum_{k=0}^{\infty} (e_k^\top Q e_k + u_k^\top R u_k), \quad u_k = -K e_k$$

**DARE solution.** The optimal gain satisfies:

$$\begin{aligned}
P &= Q + A^\top P A - A^\top P B (R + B^\top P B)^{-1} B^\top P A \quad \text{(Discrete Algebraic Riccati Equation)} \\
K &= (R + B^\top P B)^{-1} B^\top P A
\end{aligned}$$

The implementation solves the DARE by **value iteration** (backward recursion in a
finite-horizon approximation that converges to the infinite-horizon solution):

$$\begin{aligned}
P_0 &= Q \\
P_{n+1} &= Q + A^\top P_n A - A^\top P_n B (R + B^\top P_n B)^{-1} B^\top P_n A
\end{aligned}$$

This converges to the stabilising solution $P^*$ quadratically near the fixed point.
Convergence criterion: $\lVert P_{n+1} - P_n \rVert_1 < 10^{-10}$ (typically in < 30 iterations for
the 2-state model). `iters = 1000` is a conservative upper bound.

**Lateral path-tracking model.** The LQR is applied to the 2-state lateral error
model $e = [e_y, e_\psi]^\top$:

$$\begin{aligned}
e_y' &= v \cdot e_\psi \\
e_\psi' &= (v/L)\, \delta - v \kappa_{ref}
\end{aligned}$$

Discretised with Euler over $dt$:

$$A = \begin{bmatrix} 1 & v \cdot dt \\ 0 & 1 \end{bmatrix}, \quad
B = \begin{bmatrix} 0 \\ (v/L) \cdot dt \end{bmatrix}, \quad
Q = \operatorname{diag}(q_{e_y}, q_{e_\psi}), \quad
R = [r_\delta]$$

The feedforward $\delta_{ff} = \arctan(L \kappa_{ref})$ cancels the curvature term, so the LQR
only needs to regulate the residual errors. The gain $K$ is recomputed at each step
with the current speed $v$ (clamped to 0.5 m/s to avoid singularity at standstill),
making this an **online gain-scheduled LQR** rather than a fixed gain.

**Steady-state performance.** For the double-lane-change trajectory at 12 m/s, the
lateral LQR achieves a steady-state cross-track error of **$8.6 \times 10^{-14}$ m** (machine
precision), confirming that the curvature feedforward and the LQR feedback together
track the reference exactly for this constant-speed trajectory.

### 4.3 Linear Time-Varying MPC with FISTA

#### 4.3.1 Problem formulation

Track a reference trajectory $\{(s_{ref,k}, u_{ref,k})\}_{k=0}^{N}$ over a receding
horizon $N = 15$ steps ($dt = 0.1$ s, 1.5 s look-ahead) subject to box constraints:

$$\begin{aligned}
\min \quad & \sum_{k=1}^{N} e_k^\top \bar{Q} e_k + \sum_{k=0}^{N-1} \Delta u_k^\top \bar{R} \Delta u_k \\
\text{s.t.} \quad & e_{k+1} = A_k e_k + B_k \Delta u_k + d_k, \quad e_k = s_k - s_{ref,k} \\
& a_{min} \le u_{ref,k}(0) + \Delta u_k(0) \le a_{max} \\
& \delta_{min} \le u_{ref,k}(1) + \Delta u_k(1) \le \delta_{max}
\end{aligned}$$

where $\bar{Q} = \operatorname{diag}(q_x, q_y, q_\psi, q_v)$ and $\bar{R} = \operatorname{diag}(r_a, r_\delta)$ are repeated
block-diagonal along the horizon, and $d_k = \Phi(s_{ref,k}, u_{ref,k}) - s_{ref,k+1}$ is
the affine defect from the nonlinear reference trajectory.

#### 4.3.2 Per-stage linearisation

At stage $k$, linearise the RK4 step around $(s_{ref,k}, u_{ref,k})$ via central
finite differences (Section 2.3) to obtain $(A_k, B_k)$. This gives the **linear
time-varying (LTV)** error dynamics:

$$e_{k+1} = A_k e_k + B_k \Delta u_k + d_k$$

The defect $d_k$ absorbs the Taylor remainder and ensures that $e_k = 0$ is
consistent with $s_k = s_{ref,k}$ for all $k$ — a necessary property for the
constraint to be correct.

#### 4.3.3 Condensing

Define the stacked state vector $E = [e_1^\top, \dots, e_N^\top]^\top \in \mathbb{R}^{Nn}$ and input
perturbation $\Delta U = [\Delta u_0^\top, \dots, \Delta u_{N-1}^\top]^\top \in \mathbb{R}^{Nm}$. By forward recursion:

$$E = S_x e_0 + S_u \Delta U + \text{offset}$$

The **condensed sensitivity matrix** $S_u \in \mathbb{R}^{Nn \times Nm}$ has block-lower-triangular
structure:

```math
S_u[(k)n+i,\, (j)m+c] = \big[ A_{k-1} \cdots A_{j+1} B_j \big](i,c) \quad \text{for } j \le k-1
```

and the offset $\text{offset}[k \cdot n : (k+1) \cdot n]$ propagates both $e_0$ and all defects
$d_0, \dots, d_{k-1}$ through the product of $A$ matrices.

**Condensed QP** (eliminating E):

$$\begin{aligned}
& \min_{\Delta U} \ \tfrac{1}{2} \Delta U^\top H \Delta U + g^\top \Delta U \\
& \text{s.t.} \quad lo \le \Delta U \le hi \\[4pt]
& H = 2(S_u^\top \bar{Q}_{blk} S_u + \bar{R}_{blk}) \quad \text{(positive definite)} \\
& g = 2 S_u^\top \bar{Q}_{blk}\, \text{offset} \\
& lo[k \cdot m + j] = u_{min}[j] - u_{ref,k}[j] \\
& hi[k \cdot m + j] = u_{max}[j] - u_{ref,k}[j]
\end{aligned}$$

$H$ is always positive definite because $\bar{R}_{blk}$ has strictly positive diagonal.

#### 4.3.4 Solver: FISTA with box projection

**Why FISTA?** The condensed QP has a box constraint set, for which the projection
$\Pi_{[lo,hi]}$ is the componentwise clamp — a closed-form $O(Nm)$ operation. For
strongly convex objectives with closed-form projection, the Fast Iterative
Shrinkage-Thresholding Algorithm (Beck & Teboulle, 2009) achieves $O(1/k^2)$ convergence
in the objective gap, compared to $O(1/k)$ for vanilla projected gradient. Interior-point
methods would achieve faster asymptotic convergence but require an LP/QP solver
dependency and are overkill for the problem sizes here ($Nm = 30$).

**Algorithm** (FISTA with momentum restart not implemented — plain FISTA suffices):

$$\begin{aligned}
& \text{Initialise:} \ x_0 = y_0 = 0 \in \mathbb{R}^{Nm}, \ t_0 = 1 \\
& \text{For } i = 0, 1, \dots, I-1: \\
& \quad grad = 2 H y_i + g \quad \text{(gradient at } y_i\text{)} \\
& \quad x_{i+1} = \Pi_{[lo,hi]}( y_i - \alpha \cdot grad ) \quad \text{(proximal step + projection)} \\
& \quad t_{i+1} = (1 + \sqrt{1 + 4 t_i^2}) / 2 \\
& \quad y_{i+1} = x_{i+1} + \frac{t_i - 1}{t_{i+1}}(x_{i+1} - x_i) \quad \text{(Nesterov momentum)}
\end{aligned}$$

**Step size.** The step size $\alpha = 1/L_f$ where $L_f = \lambda_{\max}(2H)$ is the Lipschitz
constant of the gradient $2Hy + g$. The largest eigenvalue is estimated by 60
iterations of the **power method**:

$$\begin{aligned}
v_0 &= (1/\sqrt{Nm})\, \mathbf{1}_{Nm} \\
w_{k+1} &= H v_k / \lVert H v_k \rVert, \quad \lambda \approx \lVert H v_k \rVert
\end{aligned}$$

Power iteration converges at rate $(\lambda_1/\lambda_2)^k$; for well-conditioned $H$ (which is
typical because $\bar{R}$ regularises) 60 iterations is conservative. Adding $10^{-9}$ to
the power estimate guarantees a finite step size.

**Convergence.** For $I = 200$ iterations the optimality gap satisfies:

$$f(x_i) - f(x^*) \le L_f \lVert x_0 - x^* \rVert^2 / (2i^2)$$

(Beck & Teboulle Theorem 4.4). In practice, 200 iterations is far more than needed:
for the horizon and weights used here the iterate changes by less than $10^{-8}$ after
~40 iterations.

---

## 5. Sensor Models

All three sensors are modelled at the **object-detection level** (not raw signal level).
Each sensor checks visibility (range $r \le r_{max}$, bearing $|\beta| \le \theta_{fov}/2$), applies a
miss probability $p_{miss}$, and returns a detection with measurement $z \in \mathbb{R}^2$ (world
frame position) and noise covariance $R \in \mathbb{R}^{2\times 2}$.

### 5.1 LiDAR

LiDAR has near-isotropic spatial resolution; the noise is modelled as isotropic:

$$\begin{aligned}
z &= [x, y]^\top + n, \quad n \sim \mathcal{N}(0, \sigma_L^2 I_2), \quad \sigma_L = 0.15 \text{ m} \\
R_L &= \sigma_L^2 I_2
\end{aligned}$$

Parameters: $r_{max} = 80$ m, $\theta_{fov} = 120°$, $p_{miss} = 0.05$.

### 5.2 Radar

Radar has high range resolution but poor cross-range (azimuth) resolution — the
classic tradeoff for frequency-modulated continuous-wave (FMCW) systems. The noise
is anisotropic along the line-of-sight (LOS):

$$\begin{aligned}
\text{Noise in LOS frame:} \quad & n_{LOS} \sim \mathcal{N}(0, \operatorname{diag}(\sigma_r^2, \sigma_{lat}^2)) \\
\text{Noise in world frame:} \quad & n = \operatorname{Rot}(\alpha)\, n_{LOS}, \quad \alpha = \psi_{ego} + \beta_{detection} \\
R_{Radar} &= \operatorname{Rot}(\alpha) \cdot \operatorname{diag}(\sigma_r^2, \sigma_{lat}^2) \cdot \operatorname{Rot}(\alpha)^\top
\end{aligned}$$

with $\sigma_r = 0.4$ m (range), $\sigma_{lat} = 1.2$ m (lateral). Additionally, radar measures
the **Doppler radial velocity** $v_r = (v_x \cos \alpha + v_y \sin \alpha) + n_v$, $n_v \sim \mathcal{N}(0, \sigma_{vr}^2)$,
$\sigma_{vr} = 0.1$ m/s. Parameters: $r_{max} = 120$ m, $\theta_{fov} = 90°$, $p_{miss} = 0.05$.

### 5.3 Camera

Camera has high angular (bearing) resolution but poor range resolution, which grows
with distance (apparent-size-based range estimation):

$$\begin{aligned}
\text{Bearing noise:} \quad & \sigma_b = 0.6° \quad \text{(}\sigma_{bearing}\text{ in radians)} \\
\text{Range relative error:} \quad & \sigma_{range} / r = 10\% \\
\text{Noise in polar frame:} \quad & (n_r, n_b) \sim \mathcal{N}(0, \operatorname{diag}((0.10 r)^2, r^2 \sigma_b^2)) \\
\text{Noise in world frame:} \quad & n = \operatorname{Rot}(\alpha) \cdot [n_r, n_b]^\top \\
R_{Camera} &= \operatorname{Rot}(\alpha) \cdot \operatorname{diag}((0.10 r)^2, (r \sigma_b)^2) \cdot \operatorname{Rot}(\alpha)^\top
\end{aligned}$$

Parameters: $r_{max} = 70$ m, $\theta_{fov} = 70°$, $p_{miss} = 0.04$. Camera also provides
the object class label $kind \in \{car, truck, \dots\}$.

---

## 6. Perception — Information-Form Sensor Fusion

### 6.1 Inter-sensor clustering

Detections from all three sensors are first clustered to associate measurements that
refer to the same physical object. Greedy nearest-neighbour with **Mahalanobis gating**:

$$\begin{aligned}
d^2(i, j) &= \Delta z_{ij}^\top (R_i + R_j)^{-1} \Delta z_{ij} < \gamma_{gate} = 9.21 \\
\Delta z_{ij} &= z_i - z_j
\end{aligned}$$

The gate $\gamma_{gate} = 9.21$ is the 99th percentile of the $\chi^2(2)$ distribution, which is
the chi-squared distribution with 2 degrees of freedom (the dimension of $z$). Using
the **sum covariance** $R_i + R_j$ rather than a fixed Euclidean distance is critical
for camera detections: a camera at 50 m has $\sigma_{range} \approx 5$ m, so naive Euclidean
gating would reject correct associations. The sum-covariance gate treats the camera's
uncertainty correctly, associating it with a LiDAR detection even when they disagree
by several metres in range.

### 6.2 Information-form fusion

Within each cluster, detections are fused by the **information-form weighted average**,
which is the maximum-likelihood estimate under the assumption of independent Gaussian
noise:

$$\begin{aligned}
R_{fused}^{-1} &= \sum_i R_i^{-1} \quad \text{(sum of Fisher informations)} \\
z_{fused} &= R_{fused} \cdot \sum_i R_i^{-1} z_i \quad \text{(information-weighted mean)}
\end{aligned}$$

This is equivalent to the Kalman update with multiple independent measurements applied
sequentially (order-invariant). Properties:
- The fused covariance is always smaller than any individual $R_i$.
- Sensors with larger uncertainty (smaller $R_i^{-1}$) contribute less to the fused mean.
- No linearisation is required because all measurements are in the same state space $\mathbb{R}^2$.

### 6.3 Velocity prior from Doppler

If the cluster contains a Radar detection, its Doppler reading `v_r` provides a rough
velocity prior for initialising a new Kalman track:

$$\begin{aligned}
\alpha_{LOS} &= \operatorname{arctan2}(z_{fused}[1], z_{fused}[0]) \quad \text{(LOS angle from origin, approximate)} \\
v_{prior} &= v_r \cdot [\cos \alpha_{LOS}, \sin \alpha_{LOS}]^\top
\end{aligned}$$

This warm-starts the Kalman filter's velocity state, reducing the cold-start velocity
error from $O(v_{true})$ to $O(\sigma_{vr} \cdot \text{angular\_error})$.

---

## 7. World Model — IMM Kalman Tracker

### 7.1 Constant-velocity Kalman filter (CV-KF)

Each track's single-model filter uses a constant-velocity prediction model:

$$\begin{aligned}
\text{State:} \quad & x = [x_p, y_p, v_x, v_y]^\top \in \mathbb{R}^4 \\
\text{Measurement:} \quad & z = [x_p, y_p]^\top \quad \text{(position only)}
\end{aligned}$$

**Process model** (exact discretisation of constant-velocity continuous-time model):

$$F = \begin{bmatrix}
1 & 0 & dt & 0 \\
0 & 1 & 0 & dt \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}$$

**Process noise** (continuous white-noise acceleration model, discretised):

$$Q(dt) = \sigma_a^2 \cdot \begin{bmatrix}
dt^4/4 & 0 & dt^3/2 & 0 \\
0 & dt^4/4 & 0 & dt^3/2 \\
dt^3/2 & 0 & dt^2 & 0 \\
0 & dt^3/2 & 0 & dt^2
\end{bmatrix}$$

This is the standard **Singer model** (Bar-Shalom et al., §6.3) where $\sigma_a$ is the
acceleration standard deviation. The power-spectral-density matrix is $q(t) = \sigma_a^2 \cdot G G^\top$,
$G = [0, 0, 1, 0;\ 0, 0, 0, 1]^\top$, integrated over $dt$.

**Measurement model**:

$$H = \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0
\end{bmatrix}$$

**Kalman update** (Joseph form for numerical stability is not used here; small
matrices make catastrophic cancellation unlikely):

$$\begin{aligned}
\nu &= z - H \hat{x} \quad \text{(innovation)} \\
S &= H \hat{P} H^\top + R \quad \text{(innovation covariance)} \\
K &= \hat{P} H^\top S^{-1} \quad \text{(Kalman gain)} \\
x &\leftarrow \hat{x} + K \nu \\
P &\leftarrow (I - K H) \hat{P}
\end{aligned}$$

The update also returns the **innovation likelihood** for IMM weighting:

$$L(z \mid \hat{x}, \hat{P}, R) = \mathcal{N}(\nu; 0, S) = \exp(-\tfrac{1}{2} \nu^\top S^{-1} \nu) / (2\pi |S|^{1/2})$$

### 7.2 Interacting Multiple Model (IMM) filter

The IMM (Blom & Bar-Shalom, 1988) maintains two CV models in parallel with different
process noises:

| Model | $\sigma_a$ | Behaviour |
|---|---|---|
| Cruise | 0.5 m/s² | Smooth highway cruising |
| Manoeuvre | 6.0 m/s² | Hard braking or swerving |

The IMM probability vector $\mu = [\mu_0, \mu_1]^\top$ (sum to 1) reflects which model is
currently active. A Markov transition matrix governs mode switching:

$$\Pi = \begin{bmatrix}
p_{stay} & 1 - p_{stay} \\
1 - p_{stay} & p_{stay}
\end{bmatrix}, \quad p_{stay} = 0.95$$

**One IMM cycle** (per timestep):

**Step 1 — Interaction (mixing).** Compute the predicted model probabilities:

$$\bar{c}_j = \sum_i \Pi_{ij} \mu_i \quad \text{(normaliser for mode } j\text{'s mixing)}$$

Compute the mixed initial condition for model $j$:

$$\begin{aligned}
\hat{x}^0_j &= \sum_i (\Pi_{ij} \mu_i / \bar{c}_j)\, x_i \\
\hat{P}^0_j &= \sum_i (\Pi_{ij} \mu_i / \bar{c}_j) \big[ P_i + (x_i - \hat{x}^0_j)(x_i - \hat{x}^0_j)^\top \big]
\end{aligned}$$

**Step 2 — Mode-conditioned prediction.** Each filter $j$ predicts from $(\hat{x}^0_j, \hat{P}^0_j)$.

**Step 3 — Mode-conditioned update.** Each filter $j$ updates with measurement $z$ and
returns likelihood $L_j = L(z \mid \text{model } j)$.

**Step 4 — Model probability update.**

$$\mu_j(\text{new}) = L_j \bar{c}_j / \sum_k L_k \bar{c}_k$$

**Step 5 — Fused estimate.**

$$\begin{aligned}
\hat{x}_{IMM} &= \sum_j \mu_j \hat{x}_j \\
P_{IMM} &= \sum_j \mu_j \big[ P_j + (\hat{x}_j - \hat{x}_{IMM})(\hat{x}_j - \hat{x}_{IMM})^\top \big]
\end{aligned}$$

The **manoeuvre probability $\mu_1$** is itself a useful output: it quantifies how
aggressively the agent is manoeuvring and can gate downstream cost functions
(e.g. the planner can penalise proximity to high-$`\mu_1`$ agents more heavily).

### 7.3 Multi-object tracking (MOT)

The `MultiObjectTracker` wraps a vector of `Track` objects, each containing one IMM
filter and lifecycle counters (`hits`, `misses`, `confirmed`).

**One tracker cycle:**

1. **Predict.** All tracks run `IMM::predict(dt)`.
2. **Associate.** Build a list of `(d², track_idx, det_idx)` pairs for all (track, detection)
   pairs within the gate $\gamma = 9.21$. Sort ascending and greedily assign in $O(n \log n)$:
   the pair with the smallest $d^2$ is matched first; both indices are then locked.
3. **Update.** Matched tracks run `IMM::update(z, R)`. Unmatched tracks increment `misses`.
4. **Spawn.** Unmatched detections spawn tentative new tracks; velocity is initialised
   from `v_prior` if available (Radar Doppler), otherwise zero.
5. **Prune.** Tracks with $misses > max\_miss = 5$ are deleted. Tracks with
   $hits \ge min\_hits = 3$ are promoted to `confirmed`.

The **Mahalanobis gate** $d^2 = \nu^\top S^{-1} \nu < \gamma$ prevents false associations beyond
the sensor noise level. $\gamma = 9.21$ corresponds to 99% containment under the
true Gaussian model; the 1% of true detections that fall outside the gate are handled
by new-track spawning.

---

## 8. Motion Planning

### 8.1 Architecture: decoupled behaviour and trajectory planning

The planner follows the classical **decoupled** structure (Paden et al., 2016):

1. **Longitudinal planning** — IDM produces a smooth speed profile.
2. **Lateral planning** — smoothstep produces a smooth lane-change profile.
3. **Multi-lane search** — both profiles are evaluated for each candidate lane;
   cost and safety rejection select the best.

This separation avoids the combinatorial explosion of joint longitudinal–lateral
optimisation while remaining expressive enough to plan both car-following and
lane changes.

### 8.2 Longitudinal: Intelligent Driver Model (IDM)

The IDM (Treiber et al., 2000) generates a smooth acceleration from the ego speed
$v$, the gap to the lead $s$, and the lead speed $v_{lead}$:

$$\begin{aligned}
s^*(v, \Delta v) &= s_0 + \max\!\big(0,\ v T + v \Delta v / (2 \sqrt{a b})\big) \\
a_{IDM} &= a \big[1 - (v/v_{des})^4 - (s^*/s)^2\big]
\end{aligned}$$

Parameters: $a = 1.5$ m/s² (max acceleration), $b = 2.0$ m/s² (comfortable deceleration),
$T = 1.5$ s (desired time headway), $s_0 = 5.0$ m (minimum jam distance),
$v_{des} = 13.0$ m/s (desired speed). The IDM produces a continuous speed profile over
the planning horizon:

$$\begin{aligned}
v[k+1] &= \max(0,\ v[k] + a_{IDM}(v[k], s[k], v_{lead}[k]) \cdot dt) \\
x[k+1] &= x[k] + v[k] \cdot dt
\end{aligned}$$

For each candidate lane, the nearest predicted in-lane lead is identified at each
step $k$; if no lead exists, the free-road model is used ($s \to \infty$).

**Why IDM over a simpler PID?** IDM intrinsically encodes the social force model: it
does not produce stop-and-go chatter at the boundary between following and free-flow,
because `s*` creates a smooth desired gap that grows with speed. A PID speed controller
with a fixed target speed cannot model this regime transition.

### 8.3 Lateral: smoothstep lane-change profile

The lateral profile interpolates smoothly between the current lane $y_0$ and the
target lane $y_{target}$:

$$\begin{aligned}
\sigma(t; a, b) &= \operatorname{clamp}((t-a)/(b-a),\ 0,\ 1) \\
y(t) &= y_0 + (y_{target} - y_0) \cdot [ 3\sigma^2 - 2\sigma^3 ] \quad \text{(smoothstep, } C^1 \text{ continuous)}
\end{aligned}$$

$t_{change} = 3.0$ s is the lane-change duration. The smoothstep has zero first derivative
at $t = a$ and $t = b$, so the lateral jerk is bounded (no impulse at the start or
end of the manoeuvre). The lateral curvature $\kappa = \ddot{y} / (1 + \dot{y}^2)^{3/2}$ implied by
the smoothstep is then used by the controller.

### 8.4 Multi-lane candidate evaluation

For each candidate lane $y_{target} \in \{0.0, \text{LANE}\}$, a trajectory $(x[k], y[k], v[k])$
is generated and scored. The cost function is:

$$\begin{aligned}
\text{cost} = {}& w_1\, \mathbb{E}[\lVert v - v_{des} \rVert^2] && \text{(speed keeping)} \\
& - w_2\, (x[M] - x[0]) && \text{(progress reward, negative = maximise)} \\
& + w_3\, \max_k(v[k]^2 |\kappa[k]|) && \text{(comfort: max lateral acceleration)} \\
& + w_4\, [\text{target} \ne \text{ego\_lane}] && \text{(lane-change cost)} \\
& + w_5\, |\text{target\_lane}| && \text{(right-lane preference, positive } y = \text{ left)} \\
& + w_6 / \max(\text{min\_clear}, 10^{-3}) && \text{(margin penalty, soft)}
\end{aligned}$$

$$w = [1.0, 0.4, 4.0, 6.0, 1.0, 30.0]$$

**Safety rejection (hard constraint).** The minimum clearance to any predicted agent
at any horizon step is computed. If $\text{min\_clear} < \text{safe\_radius} = 4.5$ m, the candidate
is set to `None` (infeasible). This is the planner's own collision avoidance layer,
independent of the guardrail.

**Emergency fallback.** If all candidates are infeasible, the planner returns a
decelerating trajectory at $1.5b$ in the current lane with label `EMERGENCY_SLOW`.
The guardrail then takes over with RSS-derived braking.

---

## 9. Safety Guardrail — RSS Monitor

The guardrail is an **independent monitor** that re-derives safety from the world
model without trusting the planner's reasoning. It operates on the proposed actuator
command $(a, \delta)$ and may veto it, substituting a fail-safe response.

### 9.1 Longitudinal RSS (Shalev-Shwartz et al., 2017)

**Worst-case distance.** Consider the ego vehicle at speed $v_{ego}$ and a lead vehicle
at speed $v_{lead}$. In the worst case:

- The lead brakes immediately at maximum deceleration $b_{lead}$.
- The ego accelerates for its reaction time $\rho$ before braking at minimum capability $b_{min}$.

The minimum safe following distance (stopping-distance argument) is:

$$d_{RSS}(v_{ego}, v_{lead}) = v_{ego}\, \rho + \tfrac{1}{2} a_{ego}\, \rho^2 + (v_{ego} + \rho a_{ego})^2 / (2 b_{min}) - v_{lead}^2 / (2 b_{lead})$$

clamped to 0. Parameters: $\rho = 0.4$ s, $a_{ego} = 1.0$ m/s², $b_{min} = 4.0$ m/s²,
$b_{lead} = 8.0$ m/s². If the actual gap $< d_{RSS}$, the guardrail triggers.

**Physical interpretation.** $v_{ego}\, \rho + \tfrac{1}{2} a_{ego}\, \rho^2$ is the distance the ego travels
during reaction time while still accelerating. $(v_{ego} + \rho a_{ego})^2 / (2 b_{min})$ is
the subsequent stopping distance. $v_{lead}^2 / (2 b_{lead})$ is the lead vehicle's
stopping distance (subtracted because the lead also stops). The formula guarantees
no collision under the stated worst-case model.

### 9.2 Time-to-collision (TTC)

$$\text{TTC} = \text{gap} / (v_{ego} - v_{lead}) \quad [\text{if } v_{ego} > v_{lead}]$$

Triggers when $\text{TTC} < 2.5$ s. TTC is a complementary check to RSS: it catches close
following at matched speeds where the RSS distance is satisfied (no closing velocity)
but the absolute gap is dangerously small.

### 9.3 Lateral RSS for cut-in detection

A cut-in is dangerous only when **both** the lateral and longitudinal safe distances
are simultaneously violated (Shalev-Shwartz et al., §4). The guardrail checks both:

**Lateral RSS distance:**

$$d_{lat}(v_{lat}) = \mu + v_{lat}\, \rho + \tfrac{1}{2} a_{lat\_max}\, \rho^2 + (v_{lat} + \rho a_{lat\_max})^2 / (2 b_{lat\_min})$$

where $\mu = 0.5$ m is a clearance buffer, $a_{lat\_max} = 0.5$ m/s², $b_{lat\_min} = 1.0$ m/s².

**Cut-in criterion** (all must hold simultaneously):

0. Lateral speed is credible: $|vy_{agent}| \le v_{y,max} = 3$ m/s. $d_{lat}$ grows with
   $vy$, so a ghost track carrying a large $vy$ estimate would inflate it until the gap
   test below passes trivially. No real vehicle merges faster than this.
1. Agent is approaching laterally: $(y_{agent} - y_{ego}) \cdot vy_{agent} < 0$.
2. Lateral gap $|y_{agent} - y_{ego}| < d_{lat}(|vy_{agent}|)$.
3. Longitudinal gap within the RSS band: $-L_{veh} \le (x_{agent} - x_{ego}) \le d_{RSS} + L_{veh}$.

Conditions 2 and 3 are the RSS pair: when both hold, the correct RSS response — there
being no safe lateral evasion — is **longitudinal braking**. Conditions 0 and 1 are
false-positive controls; they carry no safety credit and exist to protect availability.

Note that $d_{lat}$ must exceed $\text{LANE}/2$ at realistic merge speeds
($d_{lat} = 3.82$ m at $vy = 2$ m/s, against $\text{LANE}/2 = 1.75$ m). Otherwise the
check can only fire once the agent is already an in-lane lead, at which point the
longitudinal check has fired anyway and the lateral branch contributes nothing.

### 9.4 Latch mechanism

When any check triggers:

$$\begin{aligned}
& \text{latch} \leftarrow \text{hold} = 8 \quad \text{(0.8 s at } dt = 0.1 \text{ s)} \\
& \text{response: } a = -b_{emergency} = -6 \text{ m/s}^2, \ \delta = \delta_{previous} \text{ (hold steering)}
\end{aligned}$$

Each step, `latch` decrements. The guardrail overrides until $\text{latch} = 0$. This
prevents the boundary oscillation that would occur if the override deactivated as
soon as the RSS condition cleared (the ego would still be closing at the boundary).
By the time the latch expires, the ego has decelerated by $b_{emergency} \cdot 0.8 \approx 4.8$ m/s,
typically enough to restore a safe gap margin.

---

## 10. Probabilistic Occupancy Prediction

The occupancy grid provides a **continuous probabilistic map** of the road ahead,
alternative to or in conjunction with the discrete trajectory predictions used by
the planner.

### 10.1 Occupancy model

Each predicted agent trajectory $\{(x_k, y_k)\}_{k=0}^{K}$ is splatted as an
axis-aligned Gaussian representing the agent's footprint with time-growing uncertainty:

$$\begin{aligned}
\sigma(k) &= \sigma_0 + \gamma k \quad \text{(uncertainty inflation)} \\
P_i(\text{cell} \mid \text{horizon } k) &= \exp\!\big[-\tfrac{1}{2} \big( (X - x_k)^2 / (car_l + \sigma)^2 + (Y - y_k)^2 / (car_w + \sigma)^2 \big)\big]
\end{aligned}$$

with $\sigma_0 = 0.6$ m, $\gamma = 0.06$ m/step, $car_l = 2.2$ m (half-length), $car_w = 0.9$ m
(half-width). The Gaussian spread grows linearly with horizon, reflecting the
increasing uncertainty in predicted agent positions.

### 10.2 Aggregation: probabilistic OR

Multiple agents are combined by the **probabilistic OR** (union of independent events):

$$P(\text{cell occupied}) = 1 - \prod_i (1 - P_i(\text{cell}))$$

This is equivalent to treating each agent as independently occupying the cell, which
is conservative (it overestimates risk when agents are correlated). The probabilistic
OR ensures that the map saturates to 1 in regions occupied by multiple agents without
double-counting.

### 10.3 Downstream use

- **Soft cost** (planner): integrate $P(\text{occupied})$ along a candidate trajectory to
  augment the clearance term.
- **Hard veto** (guardrail): block any planned trajectory where $\max_k P(\text{cell}) > \text{threshold}$.
- **Visualisation** (`run_pipeline.py`): display the map at each timestep.

**Correctness guarantee.** The `test_occupancy` test verifies:
- Peak occupancy $> 0.8$ within 0.5 s of a confirmed agent.
- The area of cells with $P > 0.5$ grows from horizon 5 to horizon 30 (uncertainty inflation).

---

## 11. Verification and Numerical Results

All numbers quoted below are reproduced by the test suite (`pytest tests/ -v`) and
the entry-point scripts.

| Test | Metric | Result |
|---|---|---|
| Bicycle linearisation residual | $\lVert \Phi(s+\delta s) - (\Phi(s) + A \delta s) \rVert / \lVert \delta s \rVert$ | **$2.3 \times 10^{-6}$** (second-order ✓) |
| LQR steady-state cross-track | $\lvert e_y \rvert$ at steady state, double-LC | **$8.6 \times 10^{-14}$ m** |
| MPC speed tracking | $\lvert v - v_{des} \rvert$ at steady state | **$< 10^{-4}$ m/s** |
| MPC max acceleration | $\lvert a \rvert$ during speed-up | **3.00 m/s²** (limit = 3.0 ✓) |
| MPC max steering | $\lvert \delta \rvert$ during lateral recovery | **0.600 rad** (limit = 0.6 ✓) |
| IMM tracking position error | mean position RMSE after warm-up (3 s) | **< 0.6 m** |
| IMM tracking velocity error | mean velocity RMSE | **< 1.2 m/s** |
| IMM manoeuvre detection | $p_{man}$ before/after hard brake | rises significantly ✓ |
| Guardrail — nominal IDM | collision in hard-brake scenario | **None** (safe) |
| Guardrail — faulty planner, OFF | collision in hard-brake scenario | **Yes** (expected) |
| Guardrail — faulty planner, ON | collision in hard-brake scenario | **None** (caught) |
| Lateral RSS — reaction time | vs longitudinal-only | **no later** (tighter/equal) |
| Occupancy — peak | P(occupied) within 0.5 s | **> 0.8** |
| Occupancy — inflation | area(P > 0.5) at horizon 5 → 30 | **grows monotonically** |

---

## 12. References

**Vehicle dynamics**

- Rajamani, R. (2012). *Vehicle Dynamics and Control*, 2nd ed. Springer.
- Kong, J. et al. (2015). "Kinematic and dynamic vehicle models for autonomous driving control design." *IEEE IV*.

**Numerical integration**

- Press, W. H. et al. (2007). *Numerical Recipes: The Art of Scientific Computing*, 3rd ed. Cambridge.

**Control**

- Åström, K. J. & Wittenmark, B. (1997). *Computer-Controlled Systems*, 3rd ed. Prentice Hall.
- Anderson, B. D. O. & Moore, J. B. (1989). *Optimal Control: Linear Quadratic Methods*. Prentice Hall.
- Rawlings, J. B., Mayne, D. Q. & Diehl, M. (2017). *Model Predictive Control: Theory, Computation, and Design*, 2nd ed. Nob Hill Publishing.
- Beck, A. & Teboulle, M. (2009). "A fast iterative shrinkage-thresholding algorithm for linear inverse problems." *SIAM Journal on Imaging Sciences*, 2(1), 183–202.

**Estimation and tracking**

- Bar-Shalom, Y., Li, X.-R. & Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation*. Wiley.
- Blom, H. A. P. & Bar-Shalom, Y. (1988). "The interacting multiple model algorithm for systems with Markovian switching coefficients." *IEEE Transactions on Automatic Control*, 33(8), 780–783.

**Planning**

- Treiber, M., Hennecke, A. & Helbing, D. (2000). "Congested traffic states in empirical observations and microscopic simulations." *Physical Review E*, 62(2), 1805.
- Paden, B. et al. (2016). "A survey of motion planning and control techniques for self-driving urban vehicles." *IEEE Transactions on Intelligent Vehicles*, 1(1), 33–55.

**Safety**

- Shalev-Shwartz, S., Shammah, S. & Shashua, A. (2017). "On a formal model of safe and scalable self-driving cars." *arXiv:1708.06374*.
- ISO 26262:2018. *Road vehicles — Functional safety*.
- IEC 61508:2010. *Functional Safety of E/E/PE Safety-related Systems*.

**Sensor fusion**

- Thrun, S., Burgard, W. & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
- Gustafsson, F. (2010). *Statistical Sensor Fusion*. Studentlitteratur.
