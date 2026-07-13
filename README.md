# av-stack

<!-- On publish, replace OWNER/REPO below and uncomment the CI badge line. -->
<!-- [![CI](https://github.com/OWNER/REPO/actions/workflows/ci.yml/badge.svg)](https://github.com/OWNER/REPO/actions/workflows/ci.yml) -->
![Python](https://img.shields.io/badge/python-3.9%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)

A modular autonomous-vehicle stack — the classical decoupled-pipeline design used
in production AV systems: sensors → perception → world model → planning → control,
with an independent safety guardrail that enforces **RSS** safe-distance guarantees
regardless of what the planner does.

Everything is self-contained: no ROS, no external dataset, no GPU. A small synthetic
multi-lane driving world generates the scenarios and closed-loop test bed.

```
Sensors ──► Perception (fusion) ──► Tracker ──► Planner (IDM) ──► Controller ──► Vehicle
    LiDAR        inv-cov fuse         IMM         lane-change     PID / LQR / MPC   bicycle
    Radar                                         score + IDM
    Camera
                                              ▲                                       │
                                    RSS + TTC Guardrail ◄──────────────────────────────┘
```

The stack is **C++-first**: every pipeline algorithm — sensors, perception, the IMM
tracker, the IDM/lane-change planner, the RSS guardrail, occupancy, and the
controllers — is implemented in C++17 headers under `cpp/include/` and exposed to
Python through pybind11 (`control_cpp`, `world_cpp`, `perception_cpp`, `planning_cpp`,
`guardrail_cpp`, `occupancy_cpp`, `sensors_cpp`, `scenario_cpp`, `localization_cpp`,
`safety_cpp`, `hybrid_cpp`). The Python layer (`python/*.py`) is a thin
visualisation / demo / scenario-driver layer on top of those modules.

```
cpp/include/**.hpp  ──pybind11──►  *_cpp modules  ──►  python/*.py (viz, demos, tests)
```

Every C++ number is verified by `control_test` (standalone C++ unit test) and the
Python test suite, and every guardrail intervention is reproduced by the closed-loop
scenarios.

### Quick start

**Windows (MSVC) — one command.** `build_and_run.ps1` in the repo root installs the
Python dependencies, configures and builds the C++ core, runs the C++ and Python test
suites, then launches a demo — end to end:

```powershell
.\build_and_run.ps1                          # build + test + full pipeline demo (default)
.\build_and_run.ps1 -SkipBuild               # skip the build, just run the demo
.\build_and_run.ps1 -SkipBuild -SkipTests    # skip build and tests
.\build_and_run.ps1 -Demo acc                # pick a demo (see below)
```

`-Demo` selects the scenario. Every C++ scenario is reachable:

| `-Demo` | scenario | planner | what you see |
|---|---|---|---|
| `pipeline` (default) | hard brake, 8 m/s² | faulty | guardrail ON/OFF comparison — collision without it, safe stop with it |
| `animation` | hard brake + cut-in | — | pure-Python real-time animation, no C++ build needed |
| `lead_brake` | moderate brake, 4 m/s² | nominal | planner follows then overtakes; guardrail never fires |
| `mixed` | slow lead + faster car left | nominal | IMM tracks both agents; ego settles into following |
| `acc` | lead cruising at 8 m/s | nominal | ACC car-following, guardrail as a silent safety net |
| `avoidance` | slow obstacle at 3 m/s | nominal | `CHANGE_LANE` to the left, overtake, back to `CRUISE` |
| `cut_in` | neighbour merges from the left | faulty | planner ignores the merge; guardrail brakes |
| `safety_stop` | hard brake + left lane blocked | faulty | no escape route; RSS guardrail brakes to a stop |

**Any platform — no build required:**

```bash
pip install -r requirements.txt
python python/animation_demo.py          # real-time animation, no C++ required
```

**Further reading** — all documentation lives in [`docs_en/`](docs_en/) (English) and
[`docs_ja/`](docs_ja/) (Japanese):
[`ARCHITECTURE.md`](docs_en/ARCHITECTURE.md) (module-by-module walkthrough) ·
[`SAFETY_CASE.md`](docs_en/SAFETY_CASE.md) (the RSS/guardrail safety argument) ·
[`TECHNICAL.md`](docs_en/TECHNICAL.md) (derivations and implementation notes) ·
[`hara.md`](docs_en/hara.md) · [`requirements_traceability.md`](docs_en/requirements_traceability.md).

**日本語:** [`README.ja.md`](README.ja.md) · [`docs_ja/`](docs_ja/)

---

## Architecture

All algorithms are C++ header implementations under `cpp/include/`, exposed to
Python via the pybind11 modules in `cpp/src/bindings_*.cpp`. The `python/*.py`
files are the visualisation / demo / test layer.

```
# --- C++ implementations (cpp/include/) ---
control/bicycle.hpp                 # kinematic bicycle: RK4 step + linearise (for LQR/MPC)
control/linalg.hpp                  # minimal dense matrix (no external BLAS dep)
control/lqr.hpp                     # PID (anti-windup) + discrete LQR (DARE backward iter)
control/mpc.hpp                     # linear time-varying MPC (FISTA box-constrained QP)
world/tracking.hpp                  # multi-object IMM tracker (cruise/manoeuvre CV mix)
world/occupancy.hpp                 # probabilistic occupancy grid (Gaussian agent splat)
world/scenario.hpp                  # multi-lane road + scripted agents (brake, cut-in)
sensors/sensors.hpp                 # LiDAR / Radar / Camera object models + covariance R
perception/fusion.hpp               # information-form sensor fusion, Mahalanobis gating
planning/{planner,path,trajectory}.hpp  # IDM planner + lane-change scorer, ref path, traj
safety/guardrail.hpp                # independent RSS + TTC + lateral safety monitor
ad/common/types.hpp                 # shared AD types
ad/localization/{localizer,ekf_localizer}.hpp  # EKF localiser (→ localization_cpp)
ad/safety/speed_governor.hpp        # speed governor / safety envelope
hybrid/{arbiter,confidence}.hpp     # hybrid arbiter + confidence (→ hybrid_cpp)

# --- pybind11 bindings (cpp/src/) ---
bindings_{control,world,scenario,sensors,perception,planning,guardrail,occupancy,
          safety,localization,hybrid}.cpp
                                    # → import control_cpp, world_cpp, scenario_cpp,
                                    #   sensors_cpp, perception_cpp, planning_cpp,
                                    #   guardrail_cpp, occupancy_cpp, safety_cpp,
                                    #   localization_cpp, hybrid_cpp
control_test.cpp                    # standalone C++ unit tests: bicycle, LQR, MPC

# --- Python: visualisation / demo / scenario layer (python/) ---
python/animation_demo.py            # real-time bird's-eye animation + time-series plots
python/run_control_demo.py          # PID vs LQR vs MPC comparison on the double lane-change
python/run_pipeline.py              # full closed-loop pipeline (C++ modules) on hard-brake
python/world.py · path.py · trajectory.py   # support: constants + reference paths for demos
python/{planning,guardrail,perception,sensors,occupancy}.py
                                    # legacy/reference pure-Python implementations; the
                                    #   production code is the *_cpp modules above

# --- tests (pytest tests/, uses the C++ modules) ---
tests/test_control.py · test_modules.py · test_localization.py
tests/test_safety.py · test_hybrid.py

README.md / README.ja.md            # overview (English / Japanese)
docs_en/                            # documentation, English
  ARCHITECTURE.md                   #   in-depth module-by-module walkthrough
  SAFETY_CASE.md                    #   GSN safety-case argument for the RSS guardrail
  TECHNICAL.md                      #   derivations and implementation notes
  hara.md                           #   hazard analysis and risk assessment
  requirements_traceability.md      #   safety goal -> requirement -> component -> test
docs_ja/                            # the same five documents, Japanese
LICENSE                             # MIT
requirements.txt / build.sh         # dependencies + one-command build & self-check
assets/                             # generated figures
```

The layout is C++-first: every algorithm lives in `cpp/include/` and is exposed
through pybind11; `python/` is the user-facing visualisation and scenario layer.

---

## Vehicle model (`bicycle.hpp`)

The kinematic bicycle is the prediction and simulation model throughout the stack:

$$\begin{aligned}
&s = [x, y, \psi, v] \qquad u = [a, \delta] \\
&\dot{x} = v\cos\psi, \quad \dot{y} = v\sin\psi, \quad \dot{\psi} = (v/L)\tan\delta, \quad \dot{v} = a
\end{aligned}$$

Integrated with **RK4** (not Euler) — critical for the MPC's prediction accuracy
at $dt = 0.1$ s. Jacobians $A = \partial f/\partial s$, $B = \partial f/\partial u$ of the RK4 step are computed
by central finite differences, keeping them provably consistent with the nonlinear
step the plant actually uses. Verified: linearisation residual $O(\varepsilon^2) = 2.3\times10^{-6}$.

---

## Controllers (`lqr.hpp`, `mpc.hpp`)

Three longitudinal+lateral controllers share the same bicycle plant and reference path:

### PID

Longitudinal speed-keeping (PID on speed error, anti-windup, output saturation) +
lateral path-tracking (P on cross-track + heading, feedforward curvature). Simple
and fast; the reference for trade-off comparison.

### LQR

Discrete LQR on the linearised 2-state lateral error model $[e_y, e_\psi]$:

$$\dot{e}_y = v\,e_\psi, \quad \dot{e}_\psi = (v/L)\,\delta - v\kappa$$

Gain computed by **backward DARE iteration** until convergence ($\lVert \Delta P \rVert < 1\times10^{-10}$),
with feedforward $\delta_{ff} = \arctan(L\kappa)$. Verified: cross-track residual after 8 s = $8.6\times10^{-14}$ m.

### MPC

Linear time-varying MPC over a 15-step horizon. At each stage the RK4 step is
linearised around the reference to get $(A_k, B_k, d_k)$, then condensed into a
strongly-convex box-constrained QP:

$$\min\ E^\top \bar{Q} E + \Delta U^\top \bar{R} \Delta U \quad \text{s.t.}\quad u_{min} \le u_{ref} + \Delta U \le u_{max}$$

Solved from scratch with **FISTA** (200 iters, step = 1/L_f via power iteration).
Handles actuator limits $a \in [-6, 3]$ m/s², $\delta \in [-0.6, 0.6]$ rad explicitly.
Verified: target speed reached exactly, limit respected within 1 %.

---

## Sensors and perception (`cpp/include/sensors/sensors.hpp`, `cpp/include/perception/fusion.hpp`)

Three sensor models, each returning detections with a measurement covariance $R$:

| Sensor | Noise model | Special output |
|---|---|---|
| **LiDAR** | isotropic $\sigma = 0.15$ m | position only |
| **Radar** | anisotropic: $\sigma_r = 0.4$ m, $\sigma_{lat} = 1.2$ m (rotated) | Doppler radial velocity |
| **Camera** | $\sigma_{bearing} = 0.6$°, range relative error 10 % (rotated) | object class label |

Fusion (**`perception/fusion.hpp`**) groups cross-sensor detections by Mahalanobis distance
gating ($\chi^2 < 9.21$, 2 DOF, 99 % gate) and combines each cluster in information form:

$$R_{fused} = \left(\sum_i R_i^{-1}\right)^{-1}, \quad z_{fused} = R_{fused} \sum_i R_i^{-1} z_i$$

This is the maximum-likelihood combination of independent Gaussian measurements.
Radar Doppler primes the velocity estimate for new tracks.

---

## Multi-object tracker (`cpp/include/world/tracking.hpp`)

An **IMM (Interacting Multiple Model)** tracker per object (`struct IMM`), mixing
two constant-velocity Kalman filters with different process noise — a low-noise
*cruise* model and a high-noise *manoeuvre* model — so it stays tight while cruising
yet reacts quickly when an agent brakes or cuts in. State $[x, y, v_x, v_y]$; the
mode probabilities are updated from each model's measurement likelihood and the
combined estimate is the probability-weighted mixture. Fused position detections
(with covariance $R$) drive the update; nearest-neighbour data association with a
Mahalanobis gate; tracks are *confirmed* after `min_hits` consistent updates.
Verified: mean position error < 0.6 m, velocity error < 0.4 m/s after a 3 s
cold-start.

---

## Planner (`cpp/include/planning/planner.hpp`)

For each candidate lane (keep / change left / change right) the planner:

1. **IDM rollout** — rolls out the ego longitudinally against the predicted
   in-lane lead using the **Intelligent Driver Model**:

   $$\begin{aligned}
   s^* &= s_0 + vT + \frac{v\cdot\Delta v}{2\sqrt{a\cdot b}} \\
   a_{IDM} &= a\left[1 - (v/v_{des})^4 - (s^*/gap)^2\right] \quad \text{clipped to } [-1.5b, a]
   \end{aligned}$$

2. **Smoothstep lateral** — blends the current lane to the target over $t_{change} = 3$ s.

3. **Cost** — scores on speed-keeping, progress, lateral comfort (max $v^2 \kappa$), lane preference, and minimum clearance to predicted agents. Trajectories that enter the safe radius of any agent are **rejected**.

Falls back to `EMERGENCY_SLOW` (decelerate at 1.5b) if all candidates are unsafe, deferring to the guardrail. Verified: IDM headway maintained; lane-change accepted only when the neighbouring slot is clear.

---

## Guardrail (`cpp/include/safety/guardrail.hpp`)

An **independent doer-checker** that does not trust the planner. Three checks on
every timestep:

### 1. RSS longitudinal (Shalev-Shwartz et al., 2017)

Minimum safe following distance: the gap such that, even if the lead brakes at
$b_{max}$ while ego accelerates for $\rho$ seconds (reaction time) then brakes at $b_{min}$:

$$d_{RSS} = v_{ego}\,\rho + \tfrac{1}{2} a \rho^2 + \frac{(v_{ego} + \rho a)^2}{2b} - \frac{v_{lead}^2}{2b_{lead}}$$

### 2. Time-to-collision

$TTC = gap / (v_{ego} - v_{lead})$ below a threshold (2.5 s).

### 3. Lateral RSS

Detects a dangerous cut-in: a neighbour whose lateral gap is below the lateral
RSS safe distance *and* whose longitudinal position is within the RSS band.
Per RSS, both conditions must hold simultaneously.

When any check fires, the guardrail **latches** for `hold` steps and substitutes
emergency braking ($-6$ m/s²) for the planner's command.

An honest finding about guardrail limits: the RSS check uses worst-case parameters
($b_{max} = 8$ m/s² for the lead); a calibration error in those parameters
translates directly into a gap error. The safety case makes this explicit.

---

## Occupancy grid (`cpp/include/world/occupancy.hpp`)

Splatts each predicted agent trajectory as a vehicle-footprint Gaussian over the
road grid ahead, with uncertainty inflation $\sigma(k) = \sigma_0 + growth\cdot k$ per step.
Combined across agents by probabilistic OR. Provides a **soft cost** for the planner
and a **hard veto** threshold for the guardrail — a continuous alternative to
discrete collision checks. Verified: a 10 m² occupied cell scores
> 0.9 probability at the true agent location.

---

## An honest result about the guardrail (and why it matters)

Testing the stack on the `scenario_hard_brake` (lead decelerates at 8 m/s² — the
maximum parameterised): the IDM planner reacts but its comfortable-deceleration
assumption means it plans to close the gap before the full brake is predicted.
The guardrail re-derives safety with worst-case RSS assumptions and brakes
in time. Running with the guardrail disabled vs. enabled:

| | rear-end | min gap | outcome |
|---|---|---|---|
| planner only (no guardrail) | **yes** | **−0.8 m** | collision |
| with guardrail | **no** | **+3.1 m** | safe stop |

For the cut-in scenario, the lateral RSS check fires ~1.5 s before the neighbour
reaches the ego lane, giving time to brake before lateral overlap.

The guardrail is the simplest, most auditable component in the stack. That is
deliberate: for functional safety, a safety monitor must be *independently
implementable and verifiable*, not a complex module that itself needs a monitor.

---

## Build

Requires a C++17 compiler, CMake $\ge 3.18$, and Python $\ge 3.9$ with pybind11.

```bash
pip install pybind11 numpy matplotlib pytest
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Or use the provided script (Linux / macOS):

```bash
./build.sh
```

On Windows (MSVC), run from the Visual Studio developer prompt:

```bat
cmake -S . -B build
cmake --build build --config Release
```

Verify:

```bash
./build/control_test              # Linux / macOS
build\Release\control_test.exe   # Windows
```

Expected:

```
CONTROL TEST PASS
```

---

## Run

Animation demo (pure Python, no build required):

```bash
python python/animation_demo.py
```

Controller comparison (PID vs LQR vs MPC on a double lane-change):

```bash
cd python && python run_control_demo.py
```

Full closed-loop pipeline (sensors → perception → tracking → planning → guardrail → MPC):

```bash
cd python && python run_pipeline.py
```

Test suite:

```bash
pytest tests/ -v
```

---

## Validated results

| check | result |
|---|---|
| Bicycle RK4 linearisation residual | $O(\varepsilon^2) = 2.3\times10^{-6}$ (consistent with nonlinear step) |
| LQR cross-track residual after 8 s | $8.6\times10^{-14}$ m (effectively zero) |
| MPC speed tracking | $v = 12.00$ m/s (target 12), $\max\lvert a\rvert = 3.00$ (limit 3.0) |
| MPC lateral recovery | $y = 0.000$ m (target 0), $\max\lvert\delta\rvert = 0.600$ (limit 0.6) |
| Tracker position error (steady state) | < 0.6 m (mean), < 0.4 m/s (velocity) |
| Guardrail — hard brake scenario | 0 rear-ends (vs. collision without guardrail) |
| Guardrail — cut-in scenario | lateral RSS fires ~1.5 s before overlap |
| Python test suite | 28 passed (`pytest tests/`, 5 files, exercising the C++ modules) |

---

## Extending it

- **Longer horizon or nonlinear MPC.** `MPC::solve` is a self-contained FISTA loop; replace the linearisation with a sequential QP step to get SQP.
- **Real sensors.** Swap `SensorSuite` for ROS topic subscribers; the fusion and tracking interfaces are sensor-agnostic.
- **More scenarios.** Add an agent with a `brake` or `cut_in` profile in `cpp/include/world/scenario.hpp`; the planner and guardrail are scenario-independent.
- **Probabilistic planning.** `OccupancyGrid` already produces a soft cost; wire it into `Planner._cost` as a weighted term.

---

## References

- Shalev-Shwartz, Shammah & Shashua, *On a Formal Model of Safe and Scalable Self-Driving Cars* (RSS), 2017.
- Treiber, Hennecke & Helbing, *Congested Traffic States in Empirical Observations and Microscopic Simulations* (IDM), 2000.
- Mayne, Rawlings, Rao & Scokaert, *Constrained Model Predictive Control: Stability and Optimality*, Automatica 2000.
- Beck & Teboulle, *A Fast Iterative Shrinkage-Thresholding Algorithm for Linear Inverse Problems* (FISTA), SIAM J. Imaging Sci. 2009.
- Werling, Ziegler, Kammel & Thrun, *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame*, ICRA 2010.
