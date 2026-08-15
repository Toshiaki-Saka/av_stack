# Safety Case — Modular AV Stack Guardrail

> Japanese version: [`../docs_ja/SAFETY_CASE.md`](../docs_ja/SAFETY_CASE.md)

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
- **A1 (assumption).** The RSS parameters ($\rho$, $b$, $b_{lead}$) are calibrated
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
| G3.3 | Lateral cut-in | Sn4 RSS lateral + longitudinal band | fires 0.5 s earlier than the longitudinal check alone, and 1.9 m of extra clearance |
| G3.4 | Degraded planner (fault=True) | Sn2 + Sn3 + Sn4 all active | guardrail overrides a tailgating faulted planner |

**G3.1 / G3.2 detail (longitudinal).**

`scenario_hard_brake`: lead brakes at 8 m/s² (maximum parameterised). The IDM
planner reacts but its comfortable-deceleration assumption ($b = 2.0$ m/s²) means
it closes the gap before the full brake is projected. The guardrail uses worst-case
parameters ($b_{lead} = 8.0$ m/s²) and fires the RSS check before contact:

```
without guardrail:  rear-end at t ≈ 4.2 s   (gap → −0.8 m)
with guardrail:     safe stop at t ≈ 4.8 s  (gap = +3.1 m at closest approach)
```

Reproduced by `tests/test_modules.py::test_guardrail_pipeline`.

**G3.3 detail (lateral cut-in).**

`scenario_cut_in`: a neighbour in the left lane moves into the ego lane between
$t = 0.5$ s and $t = 3.5$ s. The cut-in check fires only when **all** of the
following hold simultaneously (`safety/guardrail.hpp::_cut_in_threat`):

1. The track is confirmed and its lateral speed is credible ($|v_y| \le 3$ m/s).
2. It is approaching laterally — $`\Delta y \cdot v_y \lt 0`$.
3. The lateral gap is below the RSS lateral safe distance, $`|\Delta y| \lt d_{lat}(v_y)`$.
4. The longitudinal gap is inside the RSS band, $-L_{veh} \le \Delta x \le d_{RSS} + L_{veh}$.

Conditions 3 and 4 are the RSS pair: per Shalev-Shwartz, a situation is dangerous
only when the lateral *and* longitudinal safe distances are violated together, and
the correct response — there being no safe lateral evasion — is longitudinal braking.
Conditions 1 and 2 are false-positive controls and carry no safety credit; they exist
to protect availability (SG4, REQ-SAF-04) by refusing to brake for a sensor ghost or
for a neighbour that is departing rather than merging.

The load-bearing property is that $d_{lat}$ reaches **beyond half a lane width**
($d_{lat} = 3.82$ m at $v_y = 2$ m/s, versus $\text{LANE}/2 = 1.75$ m). This is what
lets the check fire while the neighbour is *still in the adjacent lane* — before it
becomes an in-lane lead, which is the only reason the lateral branch buys anything
over the longitudinal one.

Measured on `scenario_cut_in` with the planner fault injected, so that the guardrail
is the sole remaining safety layer:

| | reaction | overrides | min distance |
|---|---|---|---|
| longitudinal + TTC only (`use_lateral=False`) | 1.7 s | 19 | 11.16 m |
| with lateral RSS (`use_lateral=True`) | **1.2 s** | 20 | **13.08 m** |

The lateral check reacts 0.5 s earlier and holds 1.9 m more clearance. Reproduced by
`tests/test_modules.py::test_lateral_rss`, which asserts the improvement is *strict* —
a lateral check that merely duplicated the longitudinal one would tie, and the test
would fail.

**Availability cost (honest).** With the lateral check active the guardrail also
intervenes in `scenario_cut_in` under a *nominal* planner (18 override steps, where
previously there were none), improving min gap from 8.43 m to 9.54 m. This is not a
false positive: at that moment the ego is at 12 m/s behind a 9 m/s merging neighbour
with an 18 m gap, against $d_{RSS} = 19.0$ m — the RSS distance is genuinely violated.
It is the expected consequence of a worst-case monitor sitting above a planner that
reasons with comfortable-deceleration assumptions. The guardrail is doing its job; the
cost is that the ego brakes in a situation the planner would have survived unaided.

**G3.4 detail (planner fault injection).**

`run_pipeline.py` with `fault=True` simulates a degraded planning channel:
$T = 0.2$ s (tailgating), $b = 1.0$ m/s² (barely braking), $`safe\_radius = 0`$
(collision avoidance disabled). This is the "primary-channel failure" the
independent safety layer exists to catch. The guardrail is not notified of the
fault; it re-derives safety from the world-model tracks and overrides the faulted
planner's commands throughout the hazard window.

---

## G4 — residual risk is identified and bounded

**Sn5 (evidence).** Three residual categories are identified:

### R1 — RSS parameter calibration

The RSS safe-distance formula uses $\rho$, $b$, $b_{lead}$. If the real lead decelerates
harder than $b_{lead}$, or if ego's reaction time is longer than $\rho$, the computed
$d_{RSS}$ will be an underestimate. In the simulator, $b_{lead} = 8.0$ m/s² matches the
maximum scripted agent deceleration, so the parameter is correctly calibrated.

For a real deployment, $b_{lead}$ should be set conservatively (tighter than the
worst expected lead deceleration) to absorb calibration uncertainty. The effect is
linear: a 1 m/s² error in $b_{lead}$ translates to roughly 0.2–0.5 m error in
$d_{RSS}$ at 12 m/s.

### R2 — unmodelled hazard classes

The guardrail monitors:
- In-lane longitudinal threats (detected)
- Cut-in from adjacent lanes (detected)

It does **not** currently monitor:
- Pedestrian or cyclist cross-traffic (out of ODD)
- Lateral collision with a stationary object at the road edge
- Adverse weather reducing braking capability ($b_{min}$ not modelled)

These are out of scope for the synthetic two-lane ODD but would require additional
sub-goals in a real deployment.

### R3 — the guardrail exists twice

The monitor is implemented in both `cpp/include/safety/guardrail.hpp` (the production
path, exposed as `guardrail_cpp`) and `python/guardrail.py` (a reference
implementation). Nothing structurally forces the two to agree, and they have already
drifted: the C++ lateral RSS distance was missing the lateral braking term for some
time, which silently reduced $d_{lat}$ to roughly a third of its specified value and
left the entire cut-in branch subsumed by the longitudinal check. The closed-loop
scenarios did not catch it, because the longitudinal check fired anyway and the
outcome still looked safe.

This is the most instructive residual in this document. A safety monitor that is
*duplicated* rather than *single-sourced* can fail silently in the copy that ships,
while the copy that is read during review remains correct. Current mitigation is
`tests/test_safety.py::test_guardrail_lateral_rss_matches_documented_formula`, which
pins the production formula to the closed form in TECHNICAL.md §9.3, plus
`test_lateral_rss`, which asserts the lateral branch produces a *strict* improvement
over the longitudinal one and therefore cannot quietly become dead code again. A real
deployment should eliminate the duplication rather than test around it.

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
