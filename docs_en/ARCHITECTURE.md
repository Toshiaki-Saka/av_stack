# Architecture & module walkthrough

> Japanese version: [`../docs_ja/ARCHITECTURE.md`](../docs_ja/ARCHITECTURE.md)

A guided tour of the codebase for a reader who wants to understand *how* each
piece works and *why* it is built that way. The stack is **C++-first**: every
pipeline algorithm — the controllers, the IMM tracker, sensors, perception,
planning, the safety guardrail, and occupancy — is implemented as a C++17 header
under `cpp/include/` and exposed to Python through pybind11 (`*_cpp` modules).
The `python/*.py` layer is the visualisation, demo, and scenario-driver frontend,
plus a safety case that ties the whole argument together.

```
            ┌──────────────────── safety case (GSN) ─────────────────────────┐
            │  SAFETY_CASE.md · safety/guardrail.hpp · run_pipeline.py        │
   ┌────────┴── safety monitor ─────────────────────────────────────────────┐│
   │  safety/guardrail.hpp  (RSS long + TTC + RSS lat)                       ││
   │ ┌──── planning & perception (C++) ───────────────────────────────────┐ ││
   │ │  planning/*.hpp · perception/fusion.hpp · world/occupancy.hpp ·     │ ││
   │ │  sensors/sensors.hpp                                                │ ││
   │ │ ┌──── C++ numerics core ────────────────────────────────────────┐  │ ││
   │ │ │ control/bicycle.hpp · lqr.hpp · mpc.hpp · world/tracking.hpp │  │ ││
   │ │ └──────────────────────────────────────────────────────────────┘  │ ││
   │ └────────────────────────────────────────────────────────────────────┘ ││
   └──────────────────────────────────────────────────────────────────────────┘│
   pybind11 → *_cpp modules · python/ frontend: animation_demo.py · run_*.py    │
```

All boxes above are C++ (`cpp/include/**.hpp`) reached from Python via the
pybind11 `*_cpp` modules; the names below trace each module to its C++ header.

---

## 1. C++ numerics core

### `cpp/include/control/bicycle.hpp` — kinematic bicycle model

The kinematic bicycle is the prediction and simulation model used everywhere:
$state = [x, y, \psi, v]$, $input = [a, \delta]$.

- **Design choice — RK4, not Euler.** At `dt = 0.1 s` and highway speeds,
  Euler integration accumulates ~0.5 m/s error per second; RK4 drops this to
  negligible. The MPC prediction accuracy depends directly on this.
- **Design choice — finite-difference Jacobians.** Rather than deriving the RK4
  chain rule analytically (error-prone and must be re-derived if the step
  changes), central finite differences $A(i,j) = (f(s+\varepsilon e_j)-f(s-\varepsilon e_j))/(2\varepsilon)$
  give Jacobians that are *provably consistent* with the nonlinear step.
- **Verification.** `control_test` checks that the linearisation residual
  $\lVert step(s_0+\delta s, u_0) - (step(s_0,u_0) + A\,\delta s) \rVert$ is $O(\varepsilon^2)$ (residual $2.3\times10^{-6}$),
  which confirms the Jacobians match the nonlinear step to second order.

### `cpp/include/control/linalg.hpp` — dense matrix

A minimal header-only dense matrix class (row-major, `double`). Provides
`operator*`, transpose `T()`, and a Gauss-Jordan `solve(A, B)` (no partial
pivoting: matrices here are at most $30\times30$ and always well-conditioned). No
external BLAS dependency — the repo builds everywhere a C++17 compiler exists.

### `cpp/include/control/lqr.hpp` — PID and LQR

**PID.** Standard proportional-integral-derivative with integral clamping
(anti-windup via `[i_min, i_max]`) and output saturation. Used for longitudinal
speed-keeping and, in `animation_demo.py`, for lateral lane-keeping.

**`dlqr`.** Discrete LQR gain `K` via backward **DARE iteration**:

```math
\begin{aligned}
P &\leftarrow Q + A^\top P A - A^\top P B (R + B^\top P B)^{-1} B^\top P A \quad \text{(until } \lVert \Delta P \rVert < tol) \\
K &= (R + B^\top P B)^{-1} B^\top P A
\end{aligned}
```

Convergence is quadratic near the solution; `iters = 1000` and $tol = 1\times10^{-10}$
are far more than needed for the 2-state model here.

**`LateralLQR`.** Wraps `dlqr` for the 2-state error model $[e_y, e_\psi]$ at speed
$v$, with feedforward $\delta_{ff} = \arctan(L \kappa)$ to cancel the reference curvature.
Speed enters the linearisation so the gain is recomputed each step (the LTV
equivalent of gain scheduling). **Verification:** `test_controllers_track` drives
the ego along a double lane-change; LQR steady-state cross-track error $= 8.6\times10^{-14}$ m.

### `cpp/include/control/mpc.hpp` — linear time-varying MPC

**Problem.** Track a reference trajectory $(s_{ref,k}, u_{ref,k})$ for $k = 0 \dots N$,
with actuator constraints $a \in [a_{min}, a_{max}]$, $\delta \in [\delta_{min}, \delta_{max}]$.

**Linearisation.** At each stage, linearise the RK4 step to get $(A_k, B_k)$ and
affine defect $d_k = step(s_{ref,k}, u_{ref,k}) - s_{ref,k+1}$. In error coordinates
$e_k = s_k - s_{ref,k}$: $e_{k+1} = A_k e_k + B_k \Delta u_k + d_k$.

**Condensing.** Forward-recursing gives $E = S_x e_0 + S_u \Delta U + offset$, so the QP
is:

$$\min \tfrac{1}{2} \Delta U^\top H \Delta U + g^\top \Delta U \quad \text{s.t.} \quad lo \le \Delta U \le hi, \qquad H = S_u^\top \bar{Q} S_u + \bar{R}$$

**Design choice — FISTA, not an interior-point solver.** The condensed QP is
always strongly convex (H is PD) and box-constrained, which is FISTA's ideal
case: closed-form projection, $O(N_m N_n)$ per iteration, no dependency on an LP
solver. The Lipschitz constant is estimated by 60-step **power iteration** on H.
200 FISTA iterations reliably converge for the horizon and weights used here.

**Verification.** `test_cpp_control_test` calls `control_test.exe`:
- speed tracking: $v = 12.00$ m/s, $\max|a| = 3.00$ m/s² (limit is 3.0) ✓
- lateral recovery: $y = 0.000$ m, $\max|\delta| = 0.600$ rad (limit is 0.6) ✓

### `cpp/include/world/tracking.hpp` — multi-object IMM tracker

An **IMM (Interacting Multiple Model)** filter per object (`struct IMM`) that mixes
two constant-velocity Kalman filters with different process noise: a low-noise
*cruise* model ($q = 0.5$) and a high-noise *manoeuvre* model ($q = 6.0$). State
$[x, y, v_x, v_y]$; each constituent filter runs the standard CV predict/update:

```math
\begin{aligned}
\text{Predict:} \quad & \hat{x} = F x, \quad \hat{P} = F P F^\top + Q \quad \text{(} F = I + dt\cdot block,\ Q = \text{process noise)} \\
\text{Update:} \quad & K = \hat{P} H^\top (H \hat{P} H^\top + R)^{-1}, \quad x \leftarrow \hat{x} + K(z - H \hat{x}), \quad P \leftarrow (I - KH) \hat{P}
\end{aligned}
```

**Mode mixing.** Each step the IMM interacts the two models (mixing their states by
the Markov transition probabilities), runs both filters, then updates the mode
probabilities $\mu$ from each model's measurement likelihood and forms the combined
estimate as the $\mu$-weighted mixture. `p_manoeuvre()` exposes the manoeuvre-mode
probability. This keeps the estimate tight while an agent cruises yet reacts
quickly when it brakes or cuts in — better than a single fixed-noise CV filter.

**Data association.** Nearest-neighbour with Mahalanobis gate. Unmatched
detections initialise tentative tracks; tracks are *confirmed* only after
`min_hits` consistent updates to suppress clutter.

**Prediction for the planner.** `predict(k)` forward-propagates each confirmed
track for `k` steps with the constant-velocity model, returning `(x, y)` lists
that the planner uses to compute avoidance costs.

**Verification.** `test_tracking`: after a 3 s cold-start the tracker maintains
a lead vehicle with mean position error < 0.6 m and velocity error < 0.4 m/s.

---

## 2. Sensor models (`cpp/include/sensors/sensors.hpp` → `sensors_cpp`)

Three object-level sensor models. Each operates on `objects = [(id, state, kind)]`
returned by the world, checks visibility (range + half-FOV), applies miss
probability, and returns `Detection` objects with measurement `z` and covariance `R`.

| Sensor | Noise model | R shape |
|---|---|---|
| **LiDAR** | Isotropic Gaussian $\sigma = 0.15$ m on $[x, y]$ | $\sigma^2 I_2$ |
| **Radar** | Range noise $\sigma_r = 0.4$ m, lateral noise $\sigma_{lat} = 1.2$ m, rotated by LOS angle | $\mathrm{Rot} \cdot \mathrm{diag}(\sigma_r^2, \sigma_{lat}^2) \cdot \mathrm{Rot}^\top$ |
| **Camera** | Bearing noise $\sigma = 0.6^\circ$, range relative error 10 %, rotated by LOS | $\mathrm{Rot} \cdot \mathrm{diag}((r\cdot err)^2, (r\cdot\sigma_b)^2) \cdot \mathrm{Rot}^\top$ |

**Design choice — anisotropic R for Radar/Camera.** Euclidean gating would fail
for the camera (range error can be several metres); using $(R_i + R_j)$ in the
Mahalanobis distance correctly weights the sensor's directional uncertainty.

---

## 3. Perception / sensor fusion (`cpp/include/perception/fusion.hpp` → `perception_cpp`)

**Clustering.** Greedy association: for each unpaired detection, collect all
detections within Mahalanobis distance $\sqrt{d^2} < \sqrt{9.21}$ ($\chi^2$ gate, 2 DOF, 99 %).

**Fusion.** Within each cluster, combine in information form:

$$R_{fused} = \left( \sum_i R_i^{-1} \right)^{-1}, \qquad z_{fused} = R_{fused} \cdot \sum_i R_i^{-1} z_i$$

This is the maximum-likelihood estimate assuming independent sensors. The fused
covariance is always tighter than any individual sensor's.

**Velocity prior.** If a Radar detection is in the cluster, its Doppler reading
is projected onto the estimated LOS bearing to form a $[v_x, v_y]$ warm-start for
new tracks, avoiding the Kalman filter's cold-start velocity error.

---

## 4. Trajectory and path (`cpp/include/planning/{trajectory,path}.hpp` → `planning_cpp`)

**`Trajectory`.** Stores $(x[k], y[k], v[k])$ and derives heading $\psi[k]$ and
curvature $\kappa[k]$ by numerical differentiation. Used as the planner output and
the MPC reference.

**`Path`.** A double lane-change parameterised by forward distance using
`smoothstep(x, a, b)`. Provides:
- `errors(state)` — signed cross-track $e_y$, heading error $e_\psi$, local $\kappa$, $v_{ref}$.
- `mpc_window(state, N)` — $(s_{ref}, u_{ref})$ flat arrays for the MPC, with
  feedforward $u_{ff} = [\Delta v / dt, \arctan(L \kappa)]$ at each stage.

---

## 5. Planner (`cpp/include/planning/planner.hpp` → `planning_cpp`)

The planner implements the classic **decoupled behaviour/trajectory** structure:

1. **Candidate generation.** For each candidate lane (ego-lane, left-lane), call
   `_gen(ego, target_lane, preds)`:
   - Longitudinal: IDM against the nearest in-lane predicted lead; free-road IDM
     if no lead.
   - Lateral: smoothstep from current `y` to `target_lane` over `t_change`.

2. **Safety rejection.** `_cost` computes minimum clearance over all predicted
   agents × all trajectory steps. If `min_clear < safe_radius`, the candidate
   is `None` (rejected). This is the planner's own collision avoidance.

3. **Scoring.** Accepted candidates are scored:

```math
\begin{aligned}
   cost = {} & \text{speed-keeping} + (-\text{progress}) + \text{comfort}(\max v^2\kappa) + \text{lane-change penalty} \\
   & + \text{lane-preference} + 30/\text{min\_clear}
   \end{aligned}
```

   The soft `30/min_clear` term pushes the planner away from close agents without
   hard-rejecting them at large margins.

4. **Fallback.** If all candidates are rejected, the planner decelerates at
   `1.5b` in the current lane and returns `EMERGENCY_SLOW`. The guardrail then
   takes over for the actual braking.

**`fault` mode.** Setting `fault=True` simulates a degraded planner channel
(tiny headway `T = 0.2 s`, minimal braking `b = 1.0`, `safe_radius = 0`). This
is the scenario in `run_pipeline.py` that demonstrates the guardrail catching a
primary-channel failure.

---

## 6. Guardrail (`cpp/include/safety/guardrail.hpp` → `guardrail_cpp`)

The guardrail is the most important component for functional safety and also the
simplest. That is by design: a safety monitor must be *independently implementable
and verifiable*, not a complex module that itself requires a monitor.

### Longitudinal RSS

$$d_{RSS}(v_{ego}, v_{lead}) = v_{ego}\cdot\rho + \tfrac{1}{2}a\cdot\rho^2 + \frac{(v_{ego} + \rho a)^2}{2b} - \frac{v_{lead}^2}{2b_{lead}}$$

Parameters: $\rho = 0.4$ s (reaction time), $a = 1.0$ m/s² (ego max accel during $\rho$),
$b = 4.0$ m/s² (ego min braking), $b_{lead} = 8.0$ m/s² (lead max braking).

The formula gives the *minimum gap* such that even if the lead brakes at its
maximum and ego takes $\rho$ seconds to react (during which it might still accelerate),
no collision occurs if ego then brakes at its minimum capability.

### TTC check

$$TTC = gap / (v_{ego} - v_{lead}) \quad \text{if } v_{ego} > v_{lead}$$

Flags when $TTC < 2.5$ s.

### Lateral RSS

Detects a dangerous cut-in by checking both conditions simultaneously:
1. Lateral gap $< \mu + \text{travel}(v_{vy,other}) + \text{travel}(v_{vy,ego})$ (lateral RSS distance).
2. Longitudinal gap within the RSS band (same formula as above).

Per the RSS model, a situation is dangerous only when *both* safe distances are
violated; the correct response (when no safe lateral evasion is available) is to
brake longitudinally.

### Latch

When any check fires, `self.latch` is set to `hold` (default 8 steps = 0.8 s) and
decremented each step. The guardrail substitutes $-b_{emergency} = -6$ m/s² and
holds the last steering command (for stability) until the latch expires.

**Design rationale for the latch.** A guardrail that turns off the moment the
RSS condition clears would oscillate at the boundary. The latch ensures the vehicle
has actually decelerated to a safe state before returning authority to the planner.

---

## 7. Occupancy grid (`cpp/include/world/occupancy.hpp` → `occupancy_cpp`)

Provides a probabilistic view of the road ahead for the planner and guardrail.

Each predicted agent trajectory is splatted as a **vehicle-footprint Gaussian**:
the agent's centre is a 2-D Gaussian with $\sigma(k) = \sigma_0 + growth\cdot k$ (uncertainty
inflating with prediction horizon), convolved with a $car_l \times car_w$ rectangular
footprint. Combined across agents:

$$P(\text{cell occupied}) = 1 - \prod_i (1 - P_i(\text{cell})) \quad \text{(probabilistic OR)}$$

This gives a smooth, differentiable map that can serve as a soft cost (integrate
occupancy along the planner's candidate trajectory) or a hard veto (block any
trajectory that passes through a cell above a threshold).

---

## 8. Safety case

`SAFETY_CASE.md` assembles the above into a **Goal Structuring Notation** assurance
argument: a top safety goal, a strategy decomposing into nominal performance /
guardrail fault mitigation / bounded residual risk, and leaf solutions each pointing
at a reproduced result in this repo.

The honest structure of the argument: the planner is a **nominal performance
channel**, not a safety mechanism. Safety is carried entirely by the independent
guardrail. The residual is made explicit (calibration errors in the RSS parameters
translate directly into gap errors) rather than hidden.

---

## Reproducing everything

```bash
pip install -r requirements.txt
./build.sh                     # cmake + build; runs control_test
pytest tests/ -v               # 28 passed (5 files, exercising the C++ modules)
cd python && python run_pipeline.py   # full closed-loop with guardrail (C++ modules)
```

Every quoted number in the README and `SAFETY_CASE.md` is produced by the test
suite (which exercises the C++ `*_cpp` modules) and the `python/*.py` entry points.
