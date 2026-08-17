# Requirements Traceability

> Japanese version: [`../docs_ja/requirements_traceability.md`](../docs_ja/requirements_traceability.md)

Mapping from the safety goals derived in the HARA (`hara.md`) through technical
requirements to the implementing components. This is an illustrative traceability
document, not the deliverable of an actual safety review.

Chain: safety goal (SG) → requirement (REQ) → design element → verification

**Design elements name the production path.** Every pipeline algorithm ships as C++
(`cpp/include/**`, exposed through pybind11); the `python/*.py` equivalents are
reference implementations and are *not* what runs. Requirements therefore trace to the
headers. See residual R3 in [`SAFETY_CASE.md`](SAFETY_CASE.md) for why this
distinction is load-bearing.

## Matrix

| Req ID     | Safety goal | Requirement                                                                       | Design element (production component)                     | Verification                            |
|------------|-------------|-----------------------------------------------------------------------------------|-----------------------------------------------------------|-----------------------------------------|
| REQ-PER-01 | SG1         | Obstacle detection accuracy $\ge 90$%                                              | `perception/fusion.hpp`, `world/tracking.hpp`             | `tests/test_modules.py` (perception)    |
| REQ-SAF-01 | SG1         | Guaranteed stopping distance: always stop short of a known obstacle                | `safety/guardrail.hpp` (RSS/TTC), `ad/safety/speed_governor.hpp` | `tests/test_safety.py` (case 1, 2)      |
| REQ-SAF-02 | SG2         | Crossing-obstacle prediction: pre-emptively avoid collisions within $T_{horizon}$  | `ad/safety/speed_governor.hpp` (constant-velocity prediction) | `tests/test_safety.py` (case 2)         |
| REQ-SAF-03 | SG2         | Emergency braking response $\le 0.5$ s                                             | `safety/guardrail.hpp` (veto + brake)                     | `tests/test_modules.py` (guardrail)     |
| REQ-SAF-05 | SG2         | Cut-in detection must fire before the neighbour enters the ego lane                | `safety/guardrail.hpp::_cut_in_threat` (lateral RSS)      | `tests/test_safety.py::test_guardrail_lateral_rss_matches_documented_formula`, `::test_guardrail_lateral_rss_exceeds_half_lane`, `::test_guardrail_cut_in_fires_before_lane_encroachment`, `tests/test_modules.py::test_lateral_rss` |
| REQ-CTL-01 | SG3         | Lateral error $`\lt 0.5`$ m (path tracking)                                            | `control/mpc.hpp`, `control/lqr.hpp`                      | `tests/test_control.py` (LQR/MPC)       |
| REQ-LOC-01 | SG3         | Localisation accuracy $`\lt 0.3`$ m (under $0.3$ m position noise)                     | `ad/localization/ekf_localizer.hpp`                       | `tests/test_localization.py` (case 1)   |
| REQ-SAF-04 | SG4         | RSS false-positive rate $`\lt 1`$% (do not stop when the road ahead is clear)          | `safety/guardrail.hpp` (TTC threshold; ghost-track and lateral-approach filters) | `tests/test_safety.py` (case 1, 3), `::test_guardrail_no_cut_in_for_departing_neighbour`, `::test_guardrail_ignores_ghost_track_lateral_speed` |

## Verification status (for reference)

- REQ-CTL-01: LQR cross-track error $3.12\times10^{-31}$ m (straight), MPC speed-keeping error $0$ m/s
- REQ-LOC-01: EKF — error $`\lt 0.01`$ m on a noise-free straight run (test_localization, case 1)
- REQ-SAF-01: no obstacle → target_v unchanged (test_safety, case 1)
- REQ-SAF-02: obstacle directly ahead → speed reduction confirmed (test_safety, case 2)
- REQ-SAF-05: on `scenario_cut_in` with the planner fault injected, the lateral check reacts at 1.2 s versus 1.7 s for the longitudinal check alone, and holds 13.08 m versus 11.16 m of clearance. $d_{lat} = 3.82$ m at $v_y = 2$ m/s, against $\text{LANE}/2 = 1.75$ m — so the check fires while the neighbour is still in the adjacent lane.

## Limitations

- Verification is at unit and simulation level only (no HIL, no physical vehicle)
- ASIL decomposition and independence analysis have not been performed
- Trigger conditions for SG2 (occlusion, sensor dropout) are not bounded (SOTIF issue)
- REQ-SAF-04 is stated as a rate but is verified only by directed cases (a departing neighbour and a ghost track must not trigger a brake). No false-positive rate over a scenario population has been measured, so the $`\lt 1`$% figure is a target, not evidence.
- The guardrail used to be implemented twice (`safety/guardrail.hpp` and `python/guardrail.py`) and the two drifted twice before the Python copy was deleted; the history is kept as residual R3 in [`SAFETY_CASE.md`](SAFETY_CASE.md)
