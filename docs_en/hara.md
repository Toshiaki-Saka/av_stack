# Hazard Analysis and Risk Assessment (HARA)

> Japanese version: [`../docs_ja/hara.md`](../docs_ja/hara.md)

## Item under analysis

**av_stack** autonomous driving stack (demonstrator)

## ODD (Operational Design Domain)

- Speed: $\le 15$ m/s
- Environment: synthetic multi-lane world (no public roads, no physical sensors)

## Classification scheme (ISO 26262-3, abridged)

- **Severity (S)**: S0 none · S1 light/moderate · S2 severe (survival probable) · S3 life-threatening
- **Exposure (E)**: E1 very low · E2 low · E3 medium · E4 high
- **Controllability (C)**: C0 controllable in general · C1 simply controllable · C2 normally controllable · C3 difficult to control or uncontrollable

## Hazardous event table

| ID | Hazard | Driving situation | S | E | C | ASIL (illustrative) | Safety goal | Safe state |
|----|--------|-------------------|---|---|---|---------------------|-------------|------------|
| H1 | Collision with a static obstacle | Approaching a known static obstacle in lane | S1 | E4 | C2 | A | SG1 | Controlled stop |
| H2 | Failure to respond to a dynamic cut-in vehicle | An obstacle is crossing ahead of the ego vehicle | S2 | E3 | C3 | B | SG2 | Stop short of collision |
| H3 | Lateral control departure | Tracking a curved path | S1 | E4 | C2 | A | SG3 | Decelerate / stop |
| H4 | Unnecessary emergency stop | Road ahead clear, no collision risk | S0 | E4 | C1 | QM | SG4 | Not applicable (availability) |

## Safety goals

- **SG1** — Do not collide with a detected static obstacle. Safe state: controlled stop. (from H1)
- **SG2** — Decelerate and stop appropriately for dynamic obstacles. Safe state: stop short of collision. (from H2)
- **SG3** — Keep the trajectory within the travel lane. Safe state: decelerate / stop. (from H3)
- **SG4** — Avoid unnecessary emergency stops (availability / SOTIF; not safety-critical). (from H4)

## Disclaimer

This document is illustrative and is not the outcome of an actual safety review.
The S/E/C classifications and ASIL values are example values for a low-speed
($\le 15$ m/s) demonstrator and must not be reused for a production item.
