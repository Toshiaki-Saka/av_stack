"""Behaviour + motion planning with a continuous-speed (ACC) longitudinal model.

Note: the production implementation is the C++ version exposed as ``planning_cpp``
(built via pybind11). This pure-Python module is a legacy/reference implementation
retained for the older debug/demo scripts (e.g. debug_avoidance.py).


For each candidate lane the planner rolls out the ego using the Intelligent Driver
Model (IDM) against the predicted in-lane lead, giving a smooth, gap-aware speed
profile (no discrete-speed stop-and-go chatter), and a smoothstep lateral profile.
Candidates are scored on progress / comfort / speed-keeping / lane preference and
rejected if they enter the safe radius of a predicted agent; the lowest-cost
trajectory is handed to the controller. This is the classic decoupled
behaviour/trajectory planner used in modular AV stacks.
"""
import numpy as np
from trajectory import Trajectory
from world import LANE


def _smoothstep(t, a, b):
    x = np.clip((t - a) / (b - a), 0, 1)
    return x * x * (3 - 2 * x)


class Planner:
    def __init__(self, dt=0.1, horizon=4.0, v_des=13.0, L=2.7, safe_radius=4.5,
                 t_change=3.0, lanes=(0.0, LANE),
                 idm_a=1.5, idm_b=2.0, idm_T=1.5, idm_s0=5.0, veh_len=4.5, fault=False):
        self.dt, self.L = dt, L
        self.M = int(horizon / dt)
        self.v_des = v_des
        self.safe_radius, self.t_change = safe_radius, t_change
        self.lanes = lanes
        self.idm_a, self.idm_b, self.idm_T, self.idm_s0 = idm_a, idm_b, idm_T, idm_s0
        self.veh_len = veh_len
        # `fault` simulates a degraded planning channel: it tailgates (tiny headway)
        # and barely brakes -- the kind of primary-channel failure the independent
        # safety guardrail exists to catch.
        self.fault = fault
        if fault:
            self.idm_T, self.idm_b, self.idm_s0 = 0.2, 1.0, 2.0
            self.safe_radius = 0.0      # the fault: its own safety check is gone too
        # Lane commitment: once a lane change begins, hold the target until arrival.
        # Without this, per-step cost noise causes the planner to oscillate between
        # CHANGE_LANE and FOLLOW/SLOW, producing zero net lateral movement.
        self._committed_lane: float | None = None
        # Cache the trajectory at commit time and reuse it throughout the maneuver.
        # Re-generating a new smoothstep every step resets psi_ref[0] to nearly zero,
        # causing the MPC feedforward to regress and the heading to stall.
        self._committed_traj = None

    def _lead_in_lane(self, preds, x_ego, target_lane, k):
        """Nearest predicted agent ahead in `target_lane` at horizon step k."""
        best = None
        for pred in preds:
            j = min(k, len(pred) - 1)
            px, py = pred[j]
            if abs(py - target_lane) > LANE * 0.5:
                continue
            if px <= x_ego:
                continue
            gap = px - x_ego - self.veh_len
            jp = max(0, j - 1)
            v_lead = (pred[j][0] - pred[jp][0]) / max(self.dt, 1e-6) if j > 0 else 0.0
            if best is None or gap < best[0]:
                best = (gap, v_lead)
        return best

    def _idm_accel(self, v, gap, v_lead):
        dv = v - v_lead
        s_star = self.idm_s0 + max(0.0, v * self.idm_T
                                   + v * dv / (2 * np.sqrt(self.idm_a * self.idm_b)))
        gap = max(gap, 0.1)
        a = self.idm_a * (1 - (v / max(self.v_des, 0.1)) ** 4 - (s_star / gap) ** 2)
        return float(np.clip(a, -self.idm_b * 1.5, self.idm_a))

    def _gen(self, ego, target_lane, preds):
        M, dt = self.M, self.dt
        x = np.zeros(M + 1); v = np.zeros(M + 1)
        x[0], y0, _, v[0] = ego[0], ego[1], ego[2], ego[3]
        for k in range(M):
            lead = self._lead_in_lane(preds, x[k], target_lane, k)
            if lead is not None:
                a = self._idm_accel(v[k], lead[0], max(0.0, lead[1]))
            else:
                a = self._idm_accel(v[k], 1e6, v[k])      # free road
            v[k + 1] = max(0.0, v[k] + a * dt)
            x[k + 1] = x[k] + v[k] * dt
        t = np.arange(M + 1) * dt
        y = y0 + (target_lane - y0) * _smoothstep(t, 0.0, self.t_change)
        return Trajectory(x, y, v=v, dt=dt, L=self.L)

    def _cost(self, traj, preds, target_lane, ego_lane):
        min_clear = 1e9
        unsafe = False
        for pred in preds:
            m = min(len(pred), traj.n)
            for k in range(m):
                dx = abs(traj.x[k] - pred[k][0])
                dy = abs(traj.y[k] - pred[k][1])
                d = np.hypot(dx, dy)
                min_clear = min(min_clear, d)
                # Hard reject only when BOTH longitudinal and lateral gaps are too small.
                # A pure Euclidean check with safe_radius = veh_len would block legitimate
                # adjacent-lane overtakes (d ≈ LANE = 3.5 m < veh_len = 4.5 m).
                if dx < self.safe_radius and dy < LANE * 0.5:
                    unsafe = True
        if unsafe:
            return None
        cost = 0.0
        cost += 1.0 * np.mean((traj.v - self.v_des) ** 2)        # keep desired speed
        cost += -0.4 * (traj.x[-1] - traj.x[0])                  # progress (reward)
        lat_acc = traj.v ** 2 * np.abs(traj.kappa)
        cost += 4.0 * np.max(lat_acc)                            # comfort
        if abs(target_lane - ego_lane) > 0.1:
            cost += 6.0                                          # lane-change effort
        cost += 1.0 * abs(target_lane)                           # mild right-lane preference
        cost += 30.0 / max(min_clear, 1e-3)                      # margin (soft)
        return cost

    def plan(self, ego, preds):
        """ego:[x,y,psi,v]; preds: list of predicted (x,y) lists. -> (Trajectory, behaviour)."""
        if self.fault:
            # Faulty channel: drives at v_des ignoring all traffic.
            v0 = max(ego[3], self.v_des)
            x = np.array([ego[0] + v0 * k * self.dt for k in range(self.M + 1)])
            y = np.full(self.M + 1, ego[1])
            v = np.full(self.M + 1, v0)
            return Trajectory(x, y, v=v, dt=self.dt, L=self.L), "CRUISE"

        ego_lane = min(self.lanes, key=lambda L: abs(L - ego[1]))

        # If mid-maneuver, return the cached trajectory so the MPC window slides
        # forward along the original psi/y profile rather than resetting every step.
        if self._committed_lane is not None:
            if abs(ego[1] - self._committed_lane) < 0.25:
                self._committed_lane = None
                self._committed_traj = None
            else:
                beh = self._behaviour(ego, self._committed_lane, ego_lane,
                                      self._committed_traj)
                return self._committed_traj, beh

        candidate_lanes = self.lanes

        best = None
        for target_lane in candidate_lanes:
            traj = self._gen(ego, target_lane, preds)
            c = self._cost(traj, preds, target_lane, ego_lane)
            if c is None:
                continue
            if best is None or c < best[0]:
                best = (c, traj, self._behaviour(ego, target_lane, ego_lane, traj))
        if best is None:
            # all candidates unsafe -> decelerate in lane (let the guardrail act)
            x = np.zeros(self.M + 1); v = np.zeros(self.M + 1)
            x[0], v[0] = ego[0], ego[3]
            for k in range(self.M):
                v[k + 1] = max(0.0, v[k] - self.idm_b * 1.5 * self.dt)
                x[k + 1] = x[k] + v[k] * self.dt
            y = np.full(self.M + 1, ego[1])
            return Trajectory(x, y, v=v, dt=self.dt, L=self.L), "EMERGENCY_SLOW"

        # Commit to a new target lane the first time a lane change is chosen.
        chosen_target = min(self.lanes, key=lambda l: abs(best[1].y[-1] - l))
        if (self._committed_lane is None
                and abs(chosen_target - ego_lane) > 0.1):
            self._committed_lane = chosen_target
            self._committed_traj = best[1]   # freeze trajectory for the whole maneuver

        return best[1], best[2]

    def _behaviour(self, ego, target_lane, ego_lane, traj):
        if abs(target_lane - ego_lane) > 0.1:
            return "CHANGE_LANE"
        if traj.v[-1] < self.v_des - 0.5:
            return "FOLLOW/SLOW"
        return "CRUISE"
