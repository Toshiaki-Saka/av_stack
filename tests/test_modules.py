"""Verification for perception / world-model / planning / guardrail / occupancy.
Uses C++ modules exclusively — Python pipeline modules are visualization-only.
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "build"))
sys.path.insert(0, os.path.join(HERE, "..", "build", "Release"))
sys.path.insert(0, os.path.join(HERE, "..", "python"))


def test_tracking():
    import scenario_cpp as W
    import sensors_cpp  as S
    import perception_cpp as P
    import world_cpp as WC

    S.seed_rng(1)
    wd = W.scenario_lead_brake(); suite = S.SensorSuite()
    mot = WC.MultiObjectTracker(); mot.dt = 0.1; mot.min_hits = 2
    ego = [0.0, 0.0, 0.0, 11.0]; pos, vel, man_b, man_a = [], [], [], []

    for k in range(70):
        objs = wd.true_objects(); dets = []
        for f in P.perceive(suite.measure(ego, objs)):
            # Tuple: (z0,z1, R00,R01,R10,R11, kind, has_v, vx,vy, sensors)
            z0, z1, R00, R01, R10, R11, _kind, has_v, fvx, fvy, _sens = f
            dets.append((z0, z1, R00, R01, R10, R11, int(has_v), float(fvx), float(fvy)))
        tr = mot.step(dets)

        # Agent.s returns [x, y, vx, vy]
        gx, gy, gvx, gvy = wd.agents[0].s
        conf = [t for t in tr if t[6]]
        if conf:
            t = min(conf, key=lambda t: np.hypot(t[1] - gx, t[2] - gy))
            pos.append(np.hypot(t[1] - gx, t[2] - gy)); vel.append(abs(t[3] - gvx))
            wd_t = wd.agents[0].t
            (man_a if wd_t > 3.3 else man_b).append(t[5])
        ego[0] += ego[3] * 0.1; wd.step(0.1)

    assert np.mean(pos) < 0.6 and np.mean(vel) < 1.2
    assert np.mean(man_a) > np.mean(man_b)
    print(f"tracking: OK  (pos {np.mean(pos):.2f} m, vel {np.mean(vel):.2f} m/s, "
          f"p_man {np.mean(man_b):.2f}->{np.mean(man_a):.2f})")


def test_planning():
    import scenario_cpp as W
    from planning_cpp import Planner

    pl = Planner(dt=0.1, horizon=4.0, v_des=13.0)
    wd = W.scenario_mixed()
    # Preds: constant-velocity prediction list of (x,y) pairs
    preds = [[(a.s[0] + a.s[2] * 0.1 * k, a.s[1] + a.s[3] * 0.1 * k)
              for k in range(1, pl.M + 1)]
             for a in wd.agents]
    traj, beh = pl.plan([30.0, 0.0, 0.0, 12.0], preds)

    # Minimum clearance between planned trajectory and predicted agents
    mc = min(
        min(np.hypot(traj.x[k] - p[k][0], traj.y[k] - p[k][1])
            for k in range(min(len(p), traj.n)))
        for p in preds
    )
    assert beh in ("CRUISE", "FOLLOW/SLOW", "CHANGE_LANE", "EMERGENCY_SLOW")
    assert mc >= pl.safe_radius - 1e-6 or beh == "EMERGENCY_SLOW"
    print(f"planning (IDM): OK  (behaviour={beh}, min clearance {mc:.1f} m)")


def test_lateral_rss():
    import run_pipeline as RP
    import scenario_cpp as W
    # planner_fault=True: degraded planner ignores cut-in so guardrail is the sole safety layer
    lon  = RP.run(W.scenario_cut_in, True, steps=90, use_lateral=False, planner_fault=True)
    full = RP.run(W.scenario_cut_in, True, steps=90, use_lateral=True,  planner_fault=True)
    assert full["react_t"] is not None and lon["react_t"] is not None
    assert full["react_t"] <= lon["react_t"] + 1e-9, "lateral RSS should react no later"
    assert full["min_dist"] >= lon["min_dist"] - 0.5, "lateral RSS should not lose margin"
    print(f"lateral RSS: OK  (react {full['react_t']:.1f}s vs {lon['react_t']:.1f}s, "
          f"min dist {full['min_dist']:.1f} vs {lon['min_dist']:.1f} m)")


def test_guardrail_pipeline():
    import run_pipeline as RP
    import scenario_cpp as W
    nom = RP.run(W.scenario_hard_brake, use_guardrail=False, steps=70)
    off = RP.run(W.scenario_hard_brake, use_guardrail=False, steps=70, planner_fault=True)
    on  = RP.run(W.scenario_hard_brake, use_guardrail=True,  steps=70, planner_fault=True)
    assert not nom["collided"], "nominal IDM planner should cope with the hard brake"
    assert off["collided"],     "faulty planner should rear-end without the guardrail"
    assert not on["collided"],  f"guardrail failed to catch the fault (min gap {on['min_gap']:.1f})"
    assert on["overrides"] > 0
    print(f"guardrail pipeline: OK  (nominal safe; faulty OFF {off['min_gap']:.1f} m collided, "
          f"ON {on['min_gap']:.1f} m safe, {on['overrides']} overrides)")


def test_occupancy():
    import scenario_cpp   as W
    import sensors_cpp    as S
    import perception_cpp as P
    import world_cpp      as WC
    from occupancy_cpp import OccupancyGrid

    S.seed_rng(0)
    wd = W.scenario_mixed(); suite = S.SensorSuite()
    mot = WC.MultiObjectTracker(); mot.dt = 0.1; mot.min_hits = 2
    ego = [0.0, 0.0, 0.0, 12.0]

    for _ in range(8):
        dets = []
        for f in P.perceive(suite.measure(ego, wd.true_objects())):
            z0, z1, R00, R01, R10, R11, _kind, has_v, fvx, fvy, _sens = f
            dets.append((z0, z1, R00, R01, R10, R11, int(has_v), float(fvx), float(fvy)))
        tr = mot.step(dets); wd.step(0.1)

    preds = [p for p in (mot.predict(t[0], 40) for t in tr if t[6]) if p]
    grid = OccupancyGrid(x_min=ego[0], x_range=100.0)
    g_near, g_far = grid.predict(preds, [5, 30])

    assert max(g_near) > 0.8, "occupancy should peak near a predicted agent"
    area_near = sum(1 for x in g_near if x > 0.5)
    area_far  = sum(1 for x in g_far  if x > 0.5)
    assert area_far > area_near, "uncertainty should grow with horizon"
    print(f"occupancy: OK  (peak {max(g_near):.2f}, area>0.5 grows {area_near}->{area_far} cells)")


if __name__ == "__main__":
    test_tracking()
    test_planning()
    test_lateral_rss()
    test_guardrail_pipeline()
    test_occupancy()
    print("STACK TESTS PASS")
