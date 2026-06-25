"""Quick non-animated verification of the three demo scenarios."""
import sys
sys.path.insert(0, "python")
sys.path.insert(0, "build/Release")

import world as W
from run_pipeline import run

SCENARIOS = [
    ("acc",          W.scenario_acc_follow,    True, 150),
    ("avoidance",    W.scenario_lane_avoidance, True, 130),
    ("safety_stop",  W.scenario_safety_stop,    True, 120),
]

all_pass = True
for name, sc, guardrail, steps in SCENARIOS:
    print(f"\n=== DEMO: {name} ===")
    r = run(sc, use_guardrail=guardrail, steps=steps)
    behs = list(dict.fromkeys(r["behaviour"]))
    print(f"  collided    : {r['collided']}")
    print(f"  overrides   : {r['overrides']}")
    print(f"  min_gap     : {r['min_gap']:.2f} m")
    print(f"  final_speed : {r['v'][-1]:.2f} m/s")
    print(f"  behaviours  : {behs}")
    if r["react_t"] is not None:
        print(f"  guardrail   : fired at t={r['react_t']:.1f} s")
    else:
        print(f"  guardrail   : never fired")

    # acceptance criteria
    ok = True
    if name == "acc":
        # should follow without collision, no override needed
        if r["collided"]:
            print("  FAIL: collision"); ok = False
        if r["overrides"] > 0:
            print("  WARN: unexpected guardrail override in ACC demo")
        if r["v"][-1] < 6.0:
            print("  FAIL: ego too slow at end"); ok = False
        if "FOLLOW/SLOW" not in behs and "CRUISE" not in behs:
            print("  FAIL: expected FOLLOW/SLOW or CRUISE"); ok = False

    elif name == "avoidance":
        if r["collided"]:
            print("  FAIL: collision"); ok = False
        if "CHANGE_LANE" not in behs:
            print("  FAIL: lane change never triggered"); ok = False

    elif name == "safety_stop":
        if r["collided"]:
            print("  FAIL: collision despite guardrail"); ok = False
        if r["overrides"] == 0:
            print("  FAIL: guardrail never fired"); ok = False
        if r["v"][-1] > 2.0:
            print("  FAIL: ego did not stop (v={:.2f})".format(r["v"][-1])); ok = False

    print(f"  RESULT: {'PASS' if ok else 'FAIL'}")
    if not ok:
        all_pass = False

print(f"\n{'ALL PASS' if all_pass else 'SOME FAILED'}")
