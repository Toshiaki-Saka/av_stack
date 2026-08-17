"""Step-by-step trace of the lane-avoidance scenario.

Prints one line per control step (ego pose, behaviour, guardrail verdict, gap to
the obstacle, MPC steering and the first heading reference) so a planner or
guardrail regression can be diagnosed without opening the animation.

Every stage runs in the C++ core through its pybind11 module; this script only
wires them together and formats the output.
"""
import sys

sys.path.insert(0, "python")
sys.path.insert(0, "build/Release")
sys.path.insert(0, "build")

import numpy as np

import control_cpp as C
import world_cpp as WC
import scenario_cpp as W
import sensors_cpp as S
import perception_cpp as P
from planning_cpp import Planner
from guardrail_cpp import Guardrail

DT, L = 0.1, 2.7
S.seed_rng(0)

wd = W.scenario_lane_avoidance()
suite = S.SensorSuite()
mot = WC.MultiObjectTracker(); mot.dt = DT; mot.min_hits = 2
planner = Planner(dt=DT, horizon=4.0, v_des=13.0, L=L)
guard = Guardrail(use_lateral=True)

m = C.MPC(); m.N = 15; m.dt = DT; m.L = L
m.q_x = 0.5; m.q_y = 10.0; m.q_psi = 5.0; m.q_v = 2.0
m.r_a = 1.0; m.r_delta = 4.0
m.a_max = 3.0; m.a_min = -8.0; m.delta_max = 0.6; m.delta_min = -0.6
car = C.Bicycle(); car.L = L

ego = [0.0, 0.0, 0.0, 11.0]

for k in range(80):
    objs = wd.true_objects()
    fused = P.perceive(suite.measure(ego, objs))
    dets = []
    for f in fused:
        # Tuple: (z0,z1, R00,R01,R10,R11, kind, has_v, vx,vy, sensors)
        z0, z1, R00, R01, R10, R11, _kind, has_v, fvx, fvy, _sens = f
        dets.append((z0, z1, R00, R01, R10, R11, int(has_v), float(fvx), float(fvy)))
    tracks = mot.step(dets)
    preds = [mot.predict(t[0], planner.M) for t in tracks if t[6]]
    preds = [p for p in preds if p]

    traj, behaviour = planner.plan(ego, preds)
    sref, uref = traj.mpc_window(ego, m.N)
    a_mpc, delta_mpc = m.solve(ego, sref, uref)
    a, delta, status, reason = guard.check(ego, [a_mpc, delta_mpc], tracks)
    a = float(np.clip(a, -8.0, 3.0)); delta = float(np.clip(delta, -0.6, 0.6))
    ego = car.step(ego, a, delta, DT)

    obj_x = objs[0][1][0]
    gap = obj_x - ego[0] - 4.5
    sref_arr = np.array(sref).reshape(-1, 4)
    psi_ref0 = sref_arr[0, 2]
    track_str = ' '.join(f'tr({t[1]:.0f},{t[2]:.2f})' for t in tracks if t[6])
    print(f't={k*DT:.1f}s ego=({ego[0]:.1f},{ego[1]:.3f},psi={ego[2]:.4f}) '
          f'beh={behaviour} guard={status}/{reason[:20]} gap={gap:.1f} '
          f'delta_mpc={delta_mpc:.4f} psiref0={psi_ref0:.4f} {track_str}')
    wd.step(DT)
