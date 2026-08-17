"""Quick non-animated verification of the demo scenarios.

Each scenario is run with exactly the configuration the animated demo uses:
the parameters come from ``run_pipeline._DEMO_CONFIGS``, not from a copy kept
here. That matters — this script used to hardcode its own settings and had
drifted from the demos it claims to verify. Most visibly it ran ``safety_stop``
with the *nominal* planner, while the demo (and the README) specify a faulty
one. Without the fault the planner simply changes lane around the stopped lead,
so the guardrail's straight-line emergency stop — the entire point of that
scenario — was never exercised, and the "ego did not stop" check failed against
behaviour that was in fact perfectly safe.
"""
import sys

sys.path.insert(0, "python")
sys.path.insert(0, "build/Release")
sys.path.insert(0, "build")

from run_pipeline import _DEMO_CONFIGS, run  # noqa: E402

# Scenarios covered here, and what each one has to demonstrate. Anything in
# _DEMO_CONFIGS without an entry is exercised by the animated demo only.
CHECKS = {
    "acc": "steady following, no override needed",
    "avoidance": "overtake a slow obstacle, guardrail stays quiet",
    "safety_stop": "faulty planner + blocked escape, guardrail brakes to a stop",
}


def run_from_config(name):
    """Run `name` exactly as the animated demo would."""
    cfg = _DEMO_CONFIGS[name]
    return run(cfg["scenario"],
               use_guardrail=cfg["use_guardrail"],
               planner_fault=cfg["planner_fault"],
               steps=cfg["max_steps"],
               lanes=cfg.get("lanes"))


def check(name, r):
    """Return a list of failure strings (empty means the scenario behaved)."""
    fails = []
    if name == "acc":
        # Pure ACC gap-keeping: follow without collision, no override needed,
        # and do not crawl to a halt behind an 8 m/s lead.
        if r["collided"]:
            fails.append("collision")
        if r["overrides"] > 0:
            print("  WARN: unexpected guardrail override in ACC demo")
        if r["v"][-1] < 6.0:
            fails.append(f"ego too slow at end (v={r['v'][-1]:.2f})")
        behs = set(r["behaviour"])
        if not behs & {"FOLLOW/SLOW", "CRUISE"}:
            fails.append("expected FOLLOW/SLOW or CRUISE")

    elif name == "avoidance":
        if r["collided"]:
            fails.append("collision")
        if "CHANGE_LANE" not in set(r["behaviour"]):
            fails.append("lane change never triggered")

    elif name == "safety_stop":
        # The faulty planner tailgates and the left lane is blocked, so the
        # guardrail is the only thing left: it must fire and bring ego to rest
        # without a collision.
        if r["collided"]:
            fails.append("collision despite guardrail")
        if r["overrides"] == 0:
            fails.append("guardrail never fired")
        if r["v"][-1] > 2.0:
            fails.append(f"ego did not stop (v={r['v'][-1]:.2f})")

    return fails


def main():
    all_pass = True
    for name, purpose in CHECKS.items():
        cfg = _DEMO_CONFIGS[name]
        print(f"\n=== DEMO: {name} ===")
        print(f"  purpose     : {purpose}")
        print(f"  config      : planner_fault={cfg['planner_fault']}, "
              f"steps={cfg['max_steps']}, guardrail={cfg['use_guardrail']}"
              + (f", lanes={cfg['lanes']}" if cfg.get("lanes") else ""))

        r = run_from_config(name)
        behs = list(dict.fromkeys(r["behaviour"]))
        print(f"  collided    : {r['collided']}")
        print(f"  overrides   : {r['overrides']}")
        print(f"  min_gap     : {r['min_gap']:.2f} m")
        print(f"  final_speed : {r['v'][-1]:.2f} m/s")
        print(f"  behaviours  : {behs}")
        if r["react_t"] is not None:
            print(f"  guardrail   : fired at t={r['react_t']:.1f} s")
        else:
            print("  guardrail   : never fired")

        fails = check(name, r)
        for f in fails:
            print(f"  FAIL: {f}")
        print(f"  RESULT: {'PASS' if not fails else 'FAIL'}")
        all_pass &= not fails

    print(f"\n{'ALL PASS' if all_pass else 'SOME FAILED'}")
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
