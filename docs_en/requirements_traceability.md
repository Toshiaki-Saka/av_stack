# Requirements Traceability

> Japanese version: [`../docs_ja/requirements_traceability.md`](../docs_ja/requirements_traceability.md)

Mapping from the safety goals derived in the HARA (`hara.md`) through technical
requirements to the implementing components. This is an illustrative traceability
document, not the deliverable of an actual safety review.

Chain: safety goal (SG) → requirement (REQ) → design element → verification

## Matrix

| Req ID     | Safety goal | Requirement                                                                       | Design element (component)                          | Verification                            |
|------------|-------------|-----------------------------------------------------------------------------------|-----------------------------------------------------|-----------------------------------------|
| REQ-PER-01 | SG1         | Obstacle detection accuracy $\ge 90$%                                              | `perception.py`, `tracking.hpp`                     | `tests/test_modules.py` (perception)    |
| REQ-SAF-01 | SG1         | Guaranteed stopping distance: always stop short of a known obstacle                | `guardrail.py` (RSS/TTC), `speed_governor.hpp`      | `tests/test_safety.py` (case 1, 2)      |
| REQ-SAF-02 | SG2         | Crossing-obstacle prediction: pre-emptively avoid collisions within $T_{horizon}$  | `speed_governor.hpp` (constant-velocity prediction)  | `tests/test_safety.py` (case 2)         |
| REQ-SAF-03 | SG2         | Emergency braking response $\le 0.5$ s                                             | `guardrail.py` (veto + brake)                       | `tests/test_modules.py` (guardrail)     |
| REQ-CTL-01 | SG3         | Lateral error $< 0.5$ m (path tracking)                                            | `mpc.hpp`, `lqr.hpp`                                | `tests/test_control.py` (LQR/MPC)       |
| REQ-LOC-01 | SG3         | Localisation accuracy $< 0.3$ m (under $0.3$ m position noise)                     | `ekf.hpp` (EKF localiser)                           | `tests/test_localization.py` (case 1)   |
| REQ-SAF-04 | SG4         | RSS false-positive rate $< 1$% (do not stop when the road ahead is clear)          | `guardrail.py` (TTC threshold tuning)               | `tests/test_safety.py` (case 1, 3)      |

## Verification status (for reference)

- REQ-CTL-01: LQR cross-track error $3.12\times10^{-31}$ m (straight), MPC speed-keeping error $0$ m/s
- REQ-LOC-01: EKF — error $< 0.01$ m on a noise-free straight run (test_localization, case 1)
- REQ-SAF-01: no obstacle → target_v unchanged (test_safety, case 1)
- REQ-SAF-02: obstacle directly ahead → speed reduction confirmed (test_safety, case 2)

## Limitations

- Verification is at unit and simulation level only (no HIL, no physical vehicle)
- ASIL decomposition and independence analysis have not been performed
- Trigger conditions for SG2 (occlusion, sensor dropout) are not bounded (SOTIF issue)
