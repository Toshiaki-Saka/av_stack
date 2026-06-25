"""Safety Guardrail — an independent doer-checker around the planner/controller.

Note: the production implementation is the C++ version exposed as ``guardrail_cpp``
(built via pybind11). This pure-Python module is a legacy/reference implementation
retained for the older debug/demo scripts (e.g. debug_avoidance.py).


It does not trust the planner's reasoning; it re-derives safety from the world
model and vetoes the proposed command if any of three independent checks fail,
substituting a fail-safe (emergency braking). This is the functional-safety heart
of the stack: a simple, auditable monitor with authority over a complex planner.

Checks (longitudinal-focused, the dominant rear-end risk here):
  1. RSS safe following distance (Shalev-Shwartz et al., 2017): the minimum gap
     such that, even if the lead brakes at b_max while the ego keeps accelerating
     for its reaction time rho and then brakes at b_min, no collision occurs.
  2. Time-to-collision (TTC) below a threshold.
  3. Direct collision check of the planned trajectory against predicted occupancy.
"""
import numpy as np
from world import LANE


class Guardrail:
    def __init__(self, rho=0.4, a_accel_max=1.0, b_min=4.0, b_max_lead=8.0,
                 b_emergency=6.0, ttc_min=2.5, veh_len=4.5, lane=LANE, hard_radius=3.0,
                 hold=15, mu=0.5, a_lat_max=0.5, b_lat_min=1.0, use_lateral=True):
        self.rho, self.a_accel_max = rho, a_accel_max
        self.b_min, self.b_max_lead = b_min, b_max_lead
        self.b_emergency, self.ttc_min = b_emergency, ttc_min
        self.veh_len, self.lane, self.hard_radius = veh_len, lane, hard_radius
        self.hold = hold      # keep braking this many steps after the last trigger
        self.latch = 0
        # lateral RSS parameters (Shalev-Shwartz lateral safe distance)
        self.mu, self.a_lat_max, self.b_lat_min = mu, a_lat_max, b_lat_min
        self.use_lateral = use_lateral

    def rss_min_distance(self, v_ego, v_lead):
        """Longitudinal RSS minimum safe following distance."""
        rho, a, b, bl = self.rho, self.a_accel_max, self.b_min, self.b_max_lead
        d = (v_ego * rho + 0.5 * a * rho ** 2
             + (v_ego + rho * a) ** 2 / (2 * b)
             - v_lead ** 2 / (2 * bl))
        return max(0.0, d)

    def _lat_travel(self, v_toward):
        """Lateral distance a vehicle covers if it accelerates laterally at a_lat_max
        for rho then brakes at b_lat_min to a lateral stop (RSS worst case)."""
        rho, a, b = self.rho, self.a_lat_max, self.b_lat_min
        v = max(0.0, v_toward)
        return v * rho + 0.5 * a * rho ** 2 + (v + rho * a) ** 2 / (2 * b)

    def rss_lateral_min_distance(self, v_other_lat, v_ego_lat=0.0):
        """Lateral RSS minimum safe distance: the ego (laterally ~stationary) and the
        other vehicle (closing laterally at v_other_lat) each react for rho then brake
        laterally; below the sum (plus buffer mu) the lateral situation is unsafe."""
        return self.mu + self._lat_travel(v_ego_lat) + self._lat_travel(v_other_lat)

    # Maximum credible lateral velocity for a vehicle that is genuinely in the lane
    # (not a ghost track or a vehicle rapidly transiting from another lane).
    _MAX_VY = 3.0   # m/s

    def _in_lane_lead(self, ego, tracks):
        ego_lane = round(ego[1] / self.lane) * self.lane
        best = None
        for t in tracks:                       # t = (id, x, y, vx, vy, p_man, confirmed)
            _, x, y, vx, vy, _, conf = t
            if not conf:
                continue
            if abs(y - ego_lane) > self.lane * 0.5:
                continue
            # Reject tracks with unrealistic lateral speeds (ghost/sensor artifact).
            if abs(vy) > self._MAX_VY:
                continue
            if x <= ego[0]:
                continue
            gap = x - ego[0] - self.veh_len
            if best is None or gap < best[0]:
                best = (gap, vx)
        return best

    def _cut_in_threat(self, ego, tracks):
        """Detect a dangerous cut-in: a neighbour encroaching laterally into the ego
        lane whose lateral gap is below the lateral RSS distance *and* whose
        longitudinal position is within the longitudinal RSS band. Per RSS, a
        situation is dangerous only when both the longitudinal and lateral safe
        distances are violated; the proper response (no safe lateral evasion) is to
        brake. Returns a reason string or None."""
        for t in tracks:
            _, x, y, vx, vy, _, conf = t
            if not conf:
                continue
            # Reject unrealistic lateral speeds: no real vehicle cuts in at >_MAX_VY.
            # Ghost tracks (sensor artifacts) often get large vy estimates and would
            # otherwise trigger the RSS lateral formula, which explodes for high v.
            if abs(vy) > self._MAX_VY:
                continue
            lat_gap = abs(y - ego[1])
            # approaching laterally: lateral velocity points toward the ego
            if (y - ego[1]) * vy >= -0.05:
                continue
            d_lat = self.rss_lateral_min_distance(abs(vy))
            if lat_gap >= d_lat:
                continue
            long_gap = x - ego[0]
            d_long = self.rss_min_distance(ego[3], max(0.0, vx))
            if -self.veh_len <= long_gap <= d_long + self.veh_len:
                return (f"cut-in lat {lat_gap:.1f}<{d_lat:.1f}m "
                        f"(long {long_gap:.1f}<{d_long + self.veh_len:.1f}m)")
        return None

    def check(self, ego, proposed, tracks, traj_pred_clear=None):
        """ego:[x,y,psi,v]; proposed:[a,delta]; tracks: world-model tracks.
        Returns (a, delta, status, reason)."""
        a, delta = proposed
        reasons = []

        lead = self._in_lane_lead(ego, tracks)
        if lead is not None:
            gap, v_lead = lead
            d_rss = self.rss_min_distance(ego[3], max(0.0, v_lead))
            if gap < d_rss:
                reasons.append(f"RSS gap {gap:.1f}<{d_rss:.1f}m")
            closing = ego[3] - v_lead
            if closing > 0.1:
                ttc = gap / closing
                if ttc < self.ttc_min:
                    reasons.append(f"TTC {ttc:.1f}s")

        # lateral RSS: dangerous cut-in from an adjacent lane
        if self.use_lateral:
            cut = self._cut_in_threat(ego, tracks)
            if cut is not None:
                reasons.append(cut)

        # trajectory collision (planner-independent): min clearance precomputed
        if traj_pred_clear is not None and traj_pred_clear < self.hard_radius:
            reasons.append(f"traj clearance {traj_pred_clear:.1f}m")

        if reasons:
            self.latch = self.hold
        if self.latch > 0:
            self.latch -= 1
            # fail-safe: emergency braking + zero steer (straight-line stop).
            # Holding the planner's proposed delta during an override allows a
            # mid-lane-change trajectory to steer the ego out of the lead's lane,
            # which causes _in_lane_lead to lose the lead and prematurely release
            # the guardrail before ego has actually stopped.
            return -self.b_emergency, 0.0, "OVERRIDE", "; ".join(reasons) or "latched"
        return a, delta, "OK", ""
