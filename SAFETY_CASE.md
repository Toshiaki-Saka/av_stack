# Safety Case — Modular AV Stack Guardrail

A worked, deliberately *modest* assurance argument for the RSS-based safety
guardrail in this repository, structured in **Goal Structuring Notation (GSN)**.
It is a teaching artifact, not a certification: the operating context is a
synthetic two-lane simulator, and the point is to show how the evidence produced
here slots into an ISO 26262 / ISO 21448 (SOTIF) argument — and where the
argument runs out.

## Context, assumptions, justification

- **C1 (context).** Operating Design Domain = straight two-lane road at highway
  speed in the synthetic simulator, with a fixed set of scripted agents
  (lead brake, cut-in, mixed). Speed range 0–20 m/s. Scope covers the functional
  safety of the guardrail layer, not the nominal performance of the planner.
- **A1 (assumption).** The RSS parameters (`ρ`, `b`, `b_lead`) are calibrated
  for the agent braking profiles used in the simulator. Mis-calibration is the
  main residual risk (see G4).
- **J1 (justification).** The planner (IDM + lane-change scorer) is argued as a
  **nominal performance channel** — it produces comfortable, progress-maximising
  trajectories under normal conditions. Safety is carried by the independent
  guardrail, which re-derives collision risk from first principles without
  trusting the planner's output.

## G1 — top goal

> The ego vehicle does not rear-end a lead vehicle or suffer a collision from a
> lateral cut-in within its ODD.

**S1 (strategy).** Decompose into (G2) nominal performance is adequate in
non-hazardous conditions, (G3) hazardous conditions are detected and mitigated
by the guardrail, and (G4) residual risk is identified and bounded.

---

## G2 — nominal performance is adequate

**Sn1 (evidence).** In `scenario_lead_brake` (lead decelerates at 4 m/s²) and
`scenario_mixed` (slow lead + overtaking neighbour), the IDM planner alone
maintains safe headway and selects comfortable trajectories with no guardrail
intervention. Reproduced by `tests/test_modules.py::test_planning` and
`python/run_pipeline.py`.

| scenario | guardrail interventions | min gap |
|---|---|---|
| lead_brake (4 m/s²) | 0 | > 8 m |
| mixed (cruise + overtake) | 0 | > 5 m |

---

## G3 — hazardous conditions are detected and mitigated

**S2 (strategy).** Defence in depth across independent hazard classes — no single
check is trusted alone. Each sub-goal has its own trigger condition and a common
fallback (emergency braking at −6 m/s², steering held for stability, latch for
0.8 s).

| Sub-goal | Hazard | Solution (evidence) | Result |
|---|---|---|---|
| G3.1 | Sudden lead braking | Sn2 RSS longitudinal safe distance | guardrail fires before contact; min gap +3.1 m |
| G3.2 | Very hard lead braking (8 m/s²) | Sn2 + Sn3 TTC < 2.5 s | both checks fire; planner alone would rear-end |
| G3.3 | Lateral cut-in | Sn4 RSS lateral + longitudinal dual condition | fires ~1.5 s before lateral overlap |
| G3.4 | Degraded planner (fault=True) | Sn2 + Sn3 + Sn4 all active | guardrail overrides a tailgating faulted planner |

**G3.1 / G3.2 detail (longitudinal).**

`scenario_hard_brake`: lead brakes at 8 m/s² (maximum parameterised). The IDM
planner reacts but its comfortable-deceleration assumption (`b = 2.0 m/s²`) means
it closes the gap before the full brake is projected. The guardrail uses worst-case
parameters (`b_lead = 8.0 m/s²`) and fires the RSS check before contact:

```
without guardrail:  rear-end at t ≈ 4.2 s   (gap → −0.8 m)
with guardrail:     safe stop at t ≈ 4.8 s  (gap = +3.1 m at closest approach)
```

Reproduced by `tests/test_modules.py::test_guardrail_pipeline`.

**G3.3 detail (lateral cut-in).**

`scenario_cut_in`: neighbour in the left lane moves into the ego lane starting at
`t = 1 s`. The lateral RSS check monitors the combined condition (lateral gap
below `μ + Σ lateral travel` **and** longitudinal gap within the RSS band). It
fires approximately 1.5 s before the neighbour would reach lateral overlap with
the ego footprint, giving the guardrail time to brake before the lateral hazard
becomes unavoidable.

**G3.4 detail (planner fault injection).**

`run_pipeline.py` with `fault=True` simulates a degraded planning channel:
`T = 0.2 s` (tailgating), `b = 1.0 m/s²` (barely braking), `safe_radius = 0`
(collision avoidance disabled). This is the "primary-channel failure" the
independent safety layer exists to catch. The guardrail is not notified of the
fault; it re-derives safety from the world-model tracks and overrides the faulted
planner's commands throughout the hazard window.

---

## G4 — residual risk is identified and bounded

**Sn5 (evidence).** Two residual categories are identified:

### R1 — RSS parameter calibration

The RSS safe-distance formula uses `ρ`, `b`, `b_lead`. If the real lead decelerates
harder than `b_lead`, or if ego's reaction time is longer than `ρ`, the computed
`d_RSS` will be an underestimate. In the simulator, `b_lead = 8.0 m/s²` matches the
maximum scripted agent deceleration, so the parameter is correctly calibrated.

For a real deployment, `b_lead` should be set conservatively (tighter than the
worst expected lead deceleration) to absorb calibration uncertainty. The effect is
linear: a 1 m/s² error in `b_lead` translates to roughly 0.2–0.5 m error in
`d_RSS` at 12 m/s.

### R2 — unmodelled hazard classes

The guardrail monitors:
- In-lane longitudinal threats (detected)
- Cut-in from adjacent lanes (detected)

It does **not** currently monitor:
- Pedestrian or cyclist cross-traffic (out of ODD)
- Lateral collision with a stationary object at the road edge
- Adverse weather reducing braking capability (b_min not modelled)

These are out of scope for the synthetic two-lane ODD but would require additional
sub-goals in a real deployment.

**A2 (assumption / residual).** Coverage is argued only over the scripted scenario
grid and the ODD above. Scenarios outside it — complex multi-agent interactions,
adversarial behaviour, real-world braking variation — require further analysis.
This residual is the explicit output of the safety argument, not a gap hidden by it.

---

## Limits of this case (stated plainly)

- The ODD is a synthetic simulator; real vehicle dynamics, road friction, and
  sensor lag are not modelled.
- The RSS parameters are not derived from vehicle type / road condition; they are
  single fixed values.
- The guardrail has no fallback for the case where emergency braking is itself
  unsafe (e.g. a following vehicle would rear-end the ego). Cooperative RSS
  (considering the follower) is not implemented.
- Several results are demonstrated on a scripted, deterministic simulator; the
  specific numbers do not directly transfer to a real vehicle.

The value here is the *structure*: explicit goals, independent evidence, and a
quantified, visible residual — the shape a credible modular-AV safety argument
must take, with the planner bounded inside it rather than trusted.
