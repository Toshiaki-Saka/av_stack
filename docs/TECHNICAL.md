# Technical Reference: av_stack — A Modular Autonomous Driving Stack

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
one steering angle at the front axle. State `s = [x, y, ψ, v]ᵀ ∈ ℝ⁴`, input
`u = [a, δ]ᵀ ∈ ℝ²`:

```
ẋ   = v cos ψ
ẏ   = v sin ψ
ψ̇   = (v / L) tan δ
v̇   = a
```

where `L = 2.7 m` is the wheelbase. The model is valid at low-to-moderate slip
angles; for highway speeds below tyre-limit conditions the kinematic assumption
holds to within a few percent of a dynamic (force-based) model.

### 2.2 Numerical integration — Runge–Kutta 4

The discrete step `s_{k+1} = Φ(s_k, u_k, dt)` is computed with the classical
fourth-order Runge–Kutta method:

```
k₁ = f(sₖ, uₖ)
k₂ = f(sₖ + (dt/2) k₁,  uₖ)
k₃ = f(sₖ + (dt/2) k₂,  uₖ)
k₄ = f(sₖ + dt k₃,       uₖ)

s_{k+1} = sₖ + (dt/6)(k₁ + 2k₂ + 2k₃ + k₄)
```

**Why RK4 and not Euler?** The local truncation error of Euler is O(dt²); for
`dt = 0.1 s` and `v = 13 m/s` the yaw-rate term `v/L · tan δ` accumulates roughly
0.5 m/s yaw-rate error per second. RK4 is O(dt⁵) locally and reduces this to the
sub-millimetre range, which matters because the MPC prediction horizon spans 15
steps (1.5 s) and an inaccurate prediction model systematically biases the optimal
input.

### 2.3 Discrete-time Jacobians by central finite differences

For the MPC and LQR linearisation, we need the discrete-time Jacobians:

```
Aₖ = ∂Φ/∂s |_(sₖ,uₖ)   ∈ ℝ^{4×4}
Bₖ = ∂Φ/∂u |_(sₖ,uₖ)   ∈ ℝ^{4×2}
```

These are computed numerically by central finite differences:

```
Aₖ(i,j) = [ Φ(sₖ + ε eⱼ, uₖ) − Φ(sₖ − ε eⱼ, uₖ) ]ᵢ / (2ε)
Bₖ(i,j) = [ Φ(sₖ, uₖ + ε eⱼ) − Φ(sₖ, uₖ − ε eⱼ) ]ᵢ / (2ε)
```

with `ε = 10⁻⁶`. The key advantage over analytical Jacobians is **consistency**: the
finite-difference Jacobian is guaranteed to be consistent with the same RK4 step
that the plant uses, so the LTV error model `e_{k+1} ≈ Aₖ eₖ + Bₖ Δuₖ + dₖ` has
no modelling mismatch. Analytical Jacobians require deriving the chain rule through
the RK4 stages, which introduces sign errors and is invalidated whenever the
integration method changes.

**Verification.** For a typical operating point `(s₀, u₀)` the linearisation
residual satisfies:

```
‖Φ(s₀ + δs, u₀) − (Φ(s₀, u₀) + Aₖ δs)‖ / ‖δs‖  ≈  2.3 × 10⁻⁶   (second-order)
```

confirming that the Jacobians are correct to the expected O(ε²) accuracy of central
differences.

---

## 3. Linear Algebra Backend

All numerical work uses a minimal header-only dense matrix library (`linalg.hpp`)
with no external dependencies (no Eigen, BLAS, or LAPACK). The design trade-off:
portability and zero setup cost in exchange for O(n³) dense algorithms, acceptable
because all matrices are at most ~30 × 30 (N·m = 30 for the condensed MPC QP with
N = 15, m = 2).

**`solve(A, b)` — Gauss–Jordan with partial pivoting.** For each column `c`:

1. Find the pivot row `p = argmax_{i≥c} |A(i,c)|`.
2. Swap rows `c` and `p` in `A` and `b`.
3. Divide row `c` by the diagonal entry.
4. Eliminate column `c` from all other rows.

A small regularisation `A(c,c) += 10⁻⁹` is added if the pivot is below `10⁻¹²`
to handle near-singular cases gracefully (this occurs for the DARE at very early
iterations when `P` is still close to `Q`).

**`inv(A)` = `solve(A, I)`.** Used only for tracker innovation covariance inversion
(2×2 matrices), so the cost is negligible.

---

## 4. Control Layer

### 4.1 PID with Anti-Windup

Standard three-term controller with integral clamping:

```
u(t) = Kₚ e(t) + Kᵢ ∫₀ᵗ e(τ)dτ + Kd ė(t)
```

Discretised with Euler integration for the integral and a backward difference for
the derivative. Two saturation levels are applied independently:

- **Integral clamp** `[i_min, i_max]`: limits the accumulated integral before it
  feeds into the sum, preventing windup when the actuator saturates.
- **Output clamp** `[out_min, out_max]`: limits the final command.

This is the classical *conditional anti-windup* scheme (Åström & Wittenmark §3.5).
The integral is clamped before summing, not after, so the windup is prevented
rather than corrected.

### 4.2 Discrete LQR via DARE

**Problem.** Given the discrete time-invariant linear system `e_{k+1} = A eₖ + B uₖ`,
find the state-feedback gain `K` that minimises the infinite-horizon quadratic cost:

```
J = Σ_{k=0}^{∞} (eₖᵀ Q eₖ + uₖᵀ R uₖ),    u_k = −K eₖ
```

**DARE solution.** The optimal gain satisfies:

```
P  =  Q + AᵀPA − AᵀPB (R + BᵀPB)⁻¹ BᵀPA        (Discrete Algebraic Riccati Equation)
K  =  (R + BᵀPB)⁻¹ BᵀPA
```

The implementation solves the DARE by **value iteration** (backward recursion in a
finite-horizon approximation that converges to the infinite-horizon solution):

```
P₀ = Q
Pₙ₊₁ = Q + AᵀPₙA − AᵀPₙB (R + BᵀPₙB)⁻¹ BᵀPₙA
```

This converges to the stabilising solution `P*` quadratically near the fixed point.
Convergence criterion: `‖Pₙ₊₁ − Pₙ‖₁ < 10⁻¹⁰` (typically in < 30 iterations for
the 2-state model). `iters = 1000` is a conservative upper bound.

**Lateral path-tracking model.** The LQR is applied to the 2-state lateral error
model `e = [eᵧ, e_ψ]ᵀ`:

```
eᵧ'  = v · e_ψ
e_ψ' = (v/L) δ  −  v κ_ref
```

Discretised with Euler over `dt`:

```
A = [[1,  v·dt ],    B = [[    0    ],    Q = diag(qₑᵧ, qₑᵨ),    R = [r_δ]
     [0,    1  ]]         [(v/L)·dt ]]
```

The feedforward `δ_ff = arctan(L κ_ref)` cancels the curvature term, so the LQR
only needs to regulate the residual errors. The gain `K` is recomputed at each step
with the current speed `v` (clamped to 0.5 m/s to avoid singularity at standstill),
making this an **online gain-scheduled LQR** rather than a fixed gain.

**Steady-state performance.** For the double-lane-change trajectory at 12 m/s, the
lateral LQR achieves a steady-state cross-track error of **8.6 × 10⁻¹⁴ m** (machine
precision), confirming that the curvature feedforward and the LQR feedback together
track the reference exactly for this constant-speed trajectory.

### 4.3 Linear Time-Varying MPC with FISTA

#### 4.3.1 Problem formulation

Track a reference trajectory `{(s_ref_k, u_ref_k)}_{k=0}^{N}` over a receding
horizon `N = 15` steps (`dt = 0.1 s`, 1.5 s look-ahead) subject to box constraints:

```
min  Σ_{k=1}^{N}  eₖᵀ Q̄ eₖ  +  Σ_{k=0}^{N-1}  Δuₖᵀ R̄ Δuₖ

s.t.  e_{k+1} = Aₖ eₖ + Bₖ Δuₖ + dₖ,    eₖ = sₖ − s_ref_k
      a_min ≤ u_ref_k(0) + Δuₖ(0) ≤ a_max
      δ_min ≤ u_ref_k(1) + Δuₖ(1) ≤ δ_max
```

where `Q̄ = diag(q_x, q_y, q_ψ, q_v)` and `R̄ = diag(r_a, r_δ)` are repeated
block-diagonal along the horizon, and `dₖ = Φ(s_ref_k, u_ref_k) − s_ref_{k+1}` is
the affine defect from the nonlinear reference trajectory.

#### 4.3.2 Per-stage linearisation

At stage `k`, linearise the RK4 step around `(s_ref_k, u_ref_k)` via central
finite differences (Section 2.3) to obtain `(Aₖ, Bₖ)`. This gives the **linear
time-varying (LTV)** error dynamics:

```
e_{k+1} = Aₖ eₖ + Bₖ Δuₖ + dₖ
```

The defect `dₖ` absorbs the Taylor remainder and ensures that `eₖ = 0` is
consistent with `sₖ = s_ref_k` for all `k` — a necessary property for the
constraint to be correct.

#### 4.3.3 Condensing

Define the stacked state vector `E = [e₁ᵀ, …, eₙᵀ]ᵀ ∈ ℝ^{Nn}` and input
perturbation `ΔU = [Δu₀ᵀ, …, Δu_{N-1}ᵀ]ᵀ ∈ ℝ^{Nm}`. By forward recursion:

```
E = Sx e₀ + Su ΔU + offset
```

The **condensed sensitivity matrix** `Su ∈ ℝ^{Nn × Nm}` has block-lower-triangular
structure:

```
Su[(k)n+i, (j)m+c]  =  [ Aₖ₋₁ ··· Aⱼ₊₁ Bⱼ ](i,c)   for j ≤ k−1
```

and the offset `offset[k·n : (k+1)·n]` propagates both `e₀` and all defects
`d₀, …, d_{k-1}` through the product of `A` matrices.

**Condensed QP** (eliminating E):

```
min_{ΔU}  ½ ΔUᵀ H ΔU + gᵀ ΔU
s.t.       lo ≤ ΔU ≤ hi

H = 2(Suᵀ Q̄_blk Su + R̄_blk)           (positive definite)
g = 2 Suᵀ Q̄_blk offset
lo[k·m+j] = u_min[j] − u_ref_k[j]
hi[k·m+j] = u_max[j] − u_ref_k[j]
```

`H` is always positive definite because `R̄_blk` has strictly positive diagonal.

#### 4.3.4 Solver: FISTA with box projection

**Why FISTA?** The condensed QP has a box constraint set, for which the projection
`Π_{[lo,hi]}` is the componentwise clamp — a closed-form O(Nm) operation. For
strongly convex objectives with closed-form projection, the Fast Iterative
Shrinkage-Thresholding Algorithm (Beck & Teboulle, 2009) achieves O(1/k²) convergence
in the objective gap, compared to O(1/k) for vanilla projected gradient. Interior-point
methods would achieve faster asymptotic convergence but require an LP/QP solver
dependency and are overkill for the problem sizes here (Nm = 30).

**Algorithm** (FISTA with momentum restart not implemented — plain FISTA suffices):

```
Initialise:  x₀ = y₀ = 0 ∈ ℝ^{Nm},   t₀ = 1
For i = 0, 1, …, I−1:
    grad = 2 H yᵢ + g                              (gradient at yᵢ)
    x_{i+1} = Π_{[lo,hi]}( yᵢ − α · grad )        (proximal step + projection)
    t_{i+1} = (1 + √(1 + 4tᵢ²)) / 2
    y_{i+1} = x_{i+1} + ((tᵢ−1)/t_{i+1})(x_{i+1} − xᵢ)   (Nesterov momentum)
```

**Step size.** The step size `α = 1/L_f` where `L_f = λ_max(2H)` is the Lipschitz
constant of the gradient `2Hy + g`. The largest eigenvalue is estimated by 60
iterations of the **power method**:

```
v₀ = (1/√Nm) 1_{Nm}
wₖ₊₁ = H vₖ / ‖H vₖ‖,    λ ≈ ‖H vₖ‖
```

Power iteration converges at rate `(λ₁/λ₂)^k`; for well-conditioned `H` (which is
typical because `R̄` regularises) 60 iterations is conservative. Adding `10⁻⁹` to
the power estimate guarantees a finite step size.

**Convergence.** For `I = 200` iterations the optimality gap satisfies:

```
f(xᵢ) − f(x*) ≤ L_f ‖x₀ − x*‖² / (2i²)
```

(Beck & Teboulle Theorem 4.4). In practice, 200 iterations is far more than needed:
for the horizon and weights used here the iterate changes by less than `10⁻⁸` after
~40 iterations.

---

## 5. Sensor Models

All three sensors are modelled at the **object-detection level** (not raw signal level).
Each sensor checks visibility (range `r ≤ r_max`, bearing `|β| ≤ θ_fov/2`), applies a
miss probability `p_miss`, and returns a detection with measurement `z ∈ ℝ²` (world
frame position) and noise covariance `R ∈ ℝ^{2×2}`.

### 5.1 LiDAR

LiDAR has near-isotropic spatial resolution; the noise is modelled as isotropic:

```
z = [x, y]ᵀ + n,    n ~ N(0, σ²_L I₂),    σ_L = 0.15 m

R_L = σ²_L I₂
```

Parameters: `r_max = 80 m`, `θ_fov = 120°`, `p_miss = 0.05`.

### 5.2 Radar

Radar has high range resolution but poor cross-range (azimuth) resolution — the
classic tradeoff for frequency-modulated continuous-wave (FMCW) systems. The noise
is anisotropic along the line-of-sight (LOS):

```
Noise in LOS frame:  n_LOS ~ N(0, diag(σ²_r, σ²_lat))
Noise in world frame: n = Rot(α) n_LOS,    α = ψ_ego + β_detection

R_Radar = Rot(α) · diag(σ²_r, σ²_lat) · Rot(α)ᵀ
```

with `σ_r = 0.4 m` (range), `σ_lat = 1.2 m` (lateral). Additionally, radar measures
the **Doppler radial velocity** `v_r = (vₓ cos α + vᵧ sin α) + n_v`, `n_v ~ N(0, σ²_vr)`,
`σ_vr = 0.1 m/s`. Parameters: `r_max = 120 m`, `θ_fov = 90°`, `p_miss = 0.05`.

### 5.3 Camera

Camera has high angular (bearing) resolution but poor range resolution, which grows
with distance (apparent-size-based range estimation):

```
Bearing noise:      σ_b = 0.6°   (σ_bearing in radians)
Range relative error: σ_range / r = 10%

Noise in polar frame: (n_r, n_b) ~ N(0, diag((0.10 r)², r² σ²_b))
Noise in world frame: n = Rot(α) · [n_r, n_b]ᵀ

R_Camera = Rot(α) · diag((0.10 r)², (r σ_b)²) · Rot(α)ᵀ
```

Parameters: `r_max = 70 m`, `θ_fov = 70°`, `p_miss = 0.04`. Camera also provides
the object class label `kind ∈ {car, truck, …}`.

---

## 6. Perception — Information-Form Sensor Fusion

### 6.1 Inter-sensor clustering

Detections from all three sensors are first clustered to associate measurements that
refer to the same physical object. Greedy nearest-neighbour with **Mahalanobis gating**:

```
d²(i, j) = Δzᵢⱼᵀ (Rᵢ + Rⱼ)⁻¹ Δzᵢⱼ < γ_gate = 9.21

Δzᵢⱼ = zᵢ − zⱼ
```

The gate `γ_gate = 9.21` is the 99th percentile of the χ²(2) distribution, which is
the chi-squared distribution with 2 degrees of freedom (the dimension of `z`). Using
the **sum covariance** `Rᵢ + Rⱼ` rather than a fixed Euclidean distance is critical
for camera detections: a camera at 50 m has `σ_range ≈ 5 m`, so naive Euclidean
gating would reject correct associations. The sum-covariance gate treats the camera's
uncertainty correctly, associating it with a LiDAR detection even when they disagree
by several metres in range.

### 6.2 Information-form fusion

Within each cluster, detections are fused by the **information-form weighted average**,
which is the maximum-likelihood estimate under the assumption of independent Gaussian
noise:

```
R_fused⁻¹ = Σᵢ Rᵢ⁻¹                  (sum of Fisher informations)
z_fused    = R_fused · Σᵢ Rᵢ⁻¹ zᵢ    (information-weighted mean)
```

This is equivalent to the Kalman update with multiple independent measurements applied
sequentially (order-invariant). Properties:
- The fused covariance is always smaller than any individual `Rᵢ`.
- Sensors with larger uncertainty (smaller `Rᵢ⁻¹`) contribute less to the fused mean.
- No linearisation is required because all measurements are in the same state space `ℝ²`.

### 6.3 Velocity prior from Doppler

If the cluster contains a Radar detection, its Doppler reading `v_r` provides a rough
velocity prior for initialising a new Kalman track:

```
α_LOS = arctan2(z_fused[1], z_fused[0])           (LOS angle from origin, approximate)
v_prior = v_r · [cos α_LOS, sin α_LOS]ᵀ
```

This warm-starts the Kalman filter's velocity state, reducing the cold-start velocity
error from O(v_true) to O(σ_vr · angular_error).

---

## 7. World Model — IMM Kalman Tracker

### 7.1 Constant-velocity Kalman filter (CV-KF)

Each track's single-model filter uses a constant-velocity prediction model:

```
State:   x = [xₚ, yₚ, vₓ, vᵧ]ᵀ ∈ ℝ⁴
Measurement:  z = [xₚ, yₚ]ᵀ   (position only)
```

**Process model** (exact discretisation of constant-velocity continuous-time model):

```
F = [[1, 0, dt, 0 ],
     [0, 1, 0,  dt],
     [0, 0, 1,  0 ],
     [0, 0, 0,  1 ]]
```

**Process noise** (continuous white-noise acceleration model, discretised):

```
Q(dt) = σ²_a · [[dt⁴/4,   0,    dt³/2,    0  ],
                 [0,    dt⁴/4,   0,    dt³/2  ],
                 [dt³/2,   0,    dt²,     0   ],
                 [0,    dt³/2,   0,     dt²   ]]
```

This is the standard **Singer model** (Bar-Shalom et al., §6.3) where `σ_a` is the
acceleration standard deviation. The power-spectral-density matrix is `q(t) = σ²_a · G G ᵀ`,
`G = [0, 0, 1, 0; 0, 0, 0, 1]ᵀ`, integrated over `dt`.

**Measurement model**:

```
H = [[1, 0, 0, 0],
     [0, 1, 0, 0]]
```

**Kalman update** (Joseph form for numerical stability is not used here; small
matrices make catastrophic cancellation unlikely):

```
ν = z − H x̂                       (innovation)
S = H P̂ Hᵀ + R                    (innovation covariance)
K = P̂ Hᵀ S⁻¹                      (Kalman gain)
x ← x̂ + K ν
P ← (I − K H) P̂
```

The update also returns the **innovation likelihood** for IMM weighting:

```
L(z | x̂, P̂, R) = N(ν; 0, S)  =  exp(−½ νᵀ S⁻¹ ν) / (2π |S|^½)
```

### 7.2 Interacting Multiple Model (IMM) filter

The IMM (Blom & Bar-Shalom, 1988) maintains two CV models in parallel with different
process noises:

| Model | σ_a | Behaviour |
|---|---|---|
| Cruise | 0.5 m/s² | Smooth highway cruising |
| Manoeuvre | 6.0 m/s² | Hard braking or swerving |

The IMM probability vector `μ = [μ₀, μ₁]ᵀ` (sum to 1) reflects which model is
currently active. A Markov transition matrix governs mode switching:

```
Π = [[p_stay,    1−p_stay],    p_stay = 0.95
     [1−p_stay,  p_stay  ]]
```

**One IMM cycle** (per timestep):

**Step 1 — Interaction (mixing).** Compute the predicted model probabilities:

```
c̄ⱼ = Σᵢ Πᵢⱼ μᵢ      (normaliser for mode j's mixing)
```

Compute the mixed initial condition for model `j`:

```
x̂⁰ⱼ = Σᵢ (Πᵢⱼ μᵢ / c̄ⱼ) xᵢ
P̂⁰ⱼ = Σᵢ (Πᵢⱼ μᵢ / c̄ⱼ) [Pᵢ + (xᵢ − x̂⁰ⱼ)(xᵢ − x̂⁰ⱼ)ᵀ]
```

**Step 2 — Mode-conditioned prediction.** Each filter `j` predicts from `(x̂⁰ⱼ, P̂⁰ⱼ)`.

**Step 3 — Mode-conditioned update.** Each filter `j` updates with measurement `z` and
returns likelihood `Lⱼ = L(z | model j)`.

**Step 4 — Model probability update.**

```
μⱼ(new) = Lⱼ c̄ⱼ / Σₖ Lₖ c̄ₖ
```

**Step 5 — Fused estimate.**

```
x̂_IMM = Σⱼ μⱼ x̂ⱼ
P_IMM  = Σⱼ μⱼ [Pⱼ + (x̂ⱼ − x̂_IMM)(x̂ⱼ − x̂_IMM)ᵀ]
```

The **manoeuvre probability `μ₁`** is itself a useful output: it quantifies how
aggressively the agent is manoeuvring and can gate downstream cost functions
(e.g. the planner can penalise proximity to high-`μ₁` agents more heavily).

### 7.3 Multi-object tracking (MOT)

The `MultiObjectTracker` wraps a vector of `Track` objects, each containing one IMM
filter and lifecycle counters (`hits`, `misses`, `confirmed`).

**One tracker cycle:**

1. **Predict.** All tracks run `IMM::predict(dt)`.
2. **Associate.** Build a list of `(d², track_idx, det_idx)` pairs for all (track, detection)
   pairs within the gate `γ = 9.21`. Sort ascending and greedily assign in O(n log n):
   the pair with the smallest `d²` is matched first; both indices are then locked.
3. **Update.** Matched tracks run `IMM::update(z, R)`. Unmatched tracks increment `misses`.
4. **Spawn.** Unmatched detections spawn tentative new tracks; velocity is initialised
   from `v_prior` if available (Radar Doppler), otherwise zero.
5. **Prune.** Tracks with `misses > max_miss = 5` are deleted. Tracks with
   `hits ≥ min_hits = 3` are promoted to `confirmed`.

The **Mahalanobis gate** `d² = νᵀ S⁻¹ ν < γ` prevents false associations beyond
the sensor noise level. `γ = 9.21` corresponds to 99% containment under the
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
`v`, the gap to the lead `s`, and the lead speed `v_lead`:

```
s*(v, Δv) = s₀ + max(0, v T + v Δv / (2 √(a b)))

a_IDM = a [1 − (v/v_des)⁴ − (s*/s)²]
```

Parameters: `a = 1.5 m/s²` (max acceleration), `b = 2.0 m/s²` (comfortable deceleration),
`T = 1.5 s` (desired time headway), `s₀ = 5.0 m` (minimum jam distance),
`v_des = 13.0 m/s` (desired speed). The IDM produces a continuous speed profile over
the planning horizon:

```
v[k+1] = max(0, v[k] + a_IDM(v[k], s[k], v_lead[k]) · dt)
x[k+1] = x[k] + v[k] · dt
```

For each candidate lane, the nearest predicted in-lane lead is identified at each
step `k`; if no lead exists, the free-road model is used (`s → ∞`).

**Why IDM over a simpler PID?** IDM intrinsically encodes the social force model: it
does not produce stop-and-go chatter at the boundary between following and free-flow,
because `s*` creates a smooth desired gap that grows with speed. A PID speed controller
with a fixed target speed cannot model this regime transition.

### 8.3 Lateral: smoothstep lane-change profile

The lateral profile interpolates smoothly between the current lane `y₀` and the
target lane `y_target`:

```
σ(t; a, b) = clamp((t−a)/(b−a), 0, 1)
y(t) = y₀ + (y_target − y₀) · [ 3σ² − 2σ³ ]   (smoothstep, C¹ continuous)
```

`t_change = 3.0 s` is the lane-change duration. The smoothstep has zero first derivative
at `t = a` and `t = b`, so the lateral jerk is bounded (no impulse at the start or
end of the manoeuvre). The lateral curvature `κ = ÿ / (1 + ẏ²)^{3/2}` implied by
the smoothstep is then used by the controller.

### 8.4 Multi-lane candidate evaluation

For each candidate lane `y_target ∈ {0.0, LANE}`, a trajectory `(x[k], y[k], v[k])`
is generated and scored. The cost function is:

```
cost = w₁ E[‖v − v_des‖²]            (speed keeping)
     − w₂ (x[M] − x[0])             (progress reward, negative = maximise)
     + w₃ max_k(v[k]² |κ[k]|)       (comfort: max lateral acceleration)
     + w₄ [target ≠ ego_lane]        (lane-change cost)
     + w₅ |target_lane|             (right-lane preference, positive y = left)
     + w₆ / max(min_clear, 10⁻³)    (margin penalty, soft)

w = [1.0, 0.4, 4.0, 6.0, 1.0, 30.0]
```

**Safety rejection (hard constraint).** The minimum clearance to any predicted agent
at any horizon step is computed. If `min_clear < safe_radius = 4.5 m`, the candidate
is set to `None` (infeasible). This is the planner's own collision avoidance layer,
independent of the guardrail.

**Emergency fallback.** If all candidates are infeasible, the planner returns a
decelerating trajectory at `1.5b` in the current lane with label `EMERGENCY_SLOW`.
The guardrail then takes over with RSS-derived braking.

---

## 9. Safety Guardrail — RSS Monitor

The guardrail is an **independent monitor** that re-derives safety from the world
model without trusting the planner's reasoning. It operates on the proposed actuator
command `(a, δ)` and may veto it, substituting a fail-safe response.

### 9.1 Longitudinal RSS (Shalev-Shwartz et al., 2017)

**Worst-case distance.** Consider the ego vehicle at speed `v_ego` and a lead vehicle
at speed `v_lead`. In the worst case:

- The lead brakes immediately at maximum deceleration `b_lead`.
- The ego accelerates for its reaction time `ρ` before braking at minimum capability `b_min`.

The minimum safe following distance (stopping-distance argument) is:

```
d_RSS(v_ego, v_lead) = v_ego ρ + ½ a_ego ρ² + (v_ego + ρ a_ego)² / (2 b_min)
                      − v_lead² / (2 b_lead)
```

clamped to 0. Parameters: `ρ = 0.4 s`, `a_ego = 1.0 m/s²`, `b_min = 4.0 m/s²`,
`b_lead = 8.0 m/s²`. If the actual gap `< d_RSS`, the guardrail triggers.

**Physical interpretation.** `v_ego ρ + ½ a_ego ρ²` is the distance the ego travels
during reaction time while still accelerating. `(v_ego + ρ a_ego)² / (2 b_min)` is
the subsequent stopping distance. `v_lead² / (2 b_lead)` is the lead vehicle's
stopping distance (subtracted because the lead also stops). The formula guarantees
no collision under the stated worst-case model.

### 9.2 Time-to-collision (TTC)

```
TTC = gap / (v_ego − v_lead)    [if v_ego > v_lead]
```

Triggers when `TTC < 2.5 s`. TTC is a complementary check to RSS: it catches close
following at matched speeds where the RSS distance is satisfied (no closing velocity)
but the absolute gap is dangerously small.

### 9.3 Lateral RSS for cut-in detection

A cut-in is dangerous only when **both** the lateral and longitudinal safe distances
are simultaneously violated (Shalev-Shwartz et al., §4). The guardrail checks both:

**Lateral RSS distance:**

```
d_lat(v_lat) = μ + v_lat ρ + ½ a_lat_max ρ² + (v_lat + ρ a_lat_max)² / (2 b_lat_min)
```

where `μ = 0.5 m` is a clearance buffer, `a_lat_max = 0.5 m/s²`, `b_lat_min = 1.0 m/s²`.

**Cut-in criterion** (both must hold simultaneously):

1. Agent is approaching laterally: `(y_agent − y_ego) · vy_agent < 0`.
2. Lateral gap `|y_agent − y_ego| < d_lat(|vy_agent|)`.
3. Longitudinal gap within the RSS band: `-L_veh ≤ (x_agent − x_ego) ≤ d_RSS + L_veh`.

When both 2 and 3 hold, the correct RSS response — no safe lateral evasion — is
**longitudinal braking**.

### 9.4 Latch mechanism

When any check triggers:

```
latch ← hold = 8    (0.8 s at dt = 0.1 s)
response: a = −b_emergency = −6 m/s², δ = δ_previous (hold steering)
```

Each step, `latch` decrements. The guardrail overrides until `latch = 0`. This
prevents the boundary oscillation that would occur if the override deactivated as
soon as the RSS condition cleared (the ego would still be closing at the boundary).
By the time the latch expires, the ego has decelerated by `b_emergency · 0.8 ≈ 4.8 m/s`,
typically enough to restore a safe gap margin.

---

## 10. Probabilistic Occupancy Prediction

The occupancy grid provides a **continuous probabilistic map** of the road ahead,
alternative to or in conjunction with the discrete trajectory predictions used by
the planner.

### 10.1 Occupancy model

Each predicted agent trajectory `{(xₖ, yₖ)}_{k=0}^{K}` is splatted as an
axis-aligned Gaussian representing the agent's footprint with time-growing uncertainty:

```
σ(k) = σ₀ + γ k       (uncertainty inflation)

Pᵢ(cell | horizon k) = exp[−½ ((X − xₖ)² / (car_l + σ)² + (Y − yₖ)² / (car_w + σ)²)]
```

with `σ₀ = 0.6 m`, `γ = 0.06 m/step`, `car_l = 2.2 m` (half-length), `car_w = 0.9 m`
(half-width). The Gaussian spread grows linearly with horizon, reflecting the
increasing uncertainty in predicted agent positions.

### 10.2 Aggregation: probabilistic OR

Multiple agents are combined by the **probabilistic OR** (union of independent events):

```
P(cell occupied) = 1 − ∏ᵢ (1 − Pᵢ(cell))
```

This is equivalent to treating each agent as independently occupying the cell, which
is conservative (it overestimates risk when agents are correlated). The probabilistic
OR ensures that the map saturates to 1 in regions occupied by multiple agents without
double-counting.

### 10.3 Downstream use

- **Soft cost** (planner): integrate `P(occupied)` along a candidate trajectory to
  augment the clearance term.
- **Hard veto** (guardrail): block any planned trajectory where `max_k P(cell) > threshold`.
- **Visualisation** (`run_pipeline.py`): display the map at each timestep.

**Correctness guarantee.** The `test_occupancy` test verifies:
- Peak occupancy > 0.8 within 0.5 s of a confirmed agent.
- The area of cells with `P > 0.5` grows from horizon 5 to horizon 30 (uncertainty inflation).

---

## 11. Verification and Numerical Results

All numbers quoted below are reproduced by the test suite (`pytest tests/ -v`) and
the entry-point scripts.

| Test | Metric | Result |
|---|---|---|
| Bicycle linearisation residual | `‖Φ(s+δs) − (Φ(s) + A δs)‖ / ‖δs‖` | **2.3 × 10⁻⁶** (second-order ✓) |
| LQR steady-state cross-track | `|e_y|` at steady state, double-LC | **8.6 × 10⁻¹⁴ m** |
| MPC speed tracking | `|v − v_des|` at steady state | **< 10⁻⁴ m/s** |
| MPC max acceleration | `|a|` during speed-up | **3.00 m/s²** (limit = 3.0 ✓) |
| MPC max steering | `|δ|` during lateral recovery | **0.600 rad** (limit = 0.6 ✓) |
| IMM tracking position error | mean position RMSE after warm-up (3 s) | **< 0.6 m** |
| IMM tracking velocity error | mean velocity RMSE | **< 1.2 m/s** |
| IMM manoeuvre detection | `p_man` before/after hard brake | rises significantly ✓ |
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
