"""Synthetic multi-agent driving world for the modular stack.

A straight multi-lane road along +x. Lane centres are y = 0 (ego lane) and
y = +LANE (left). Other agents are vehicles with state [x, y, vx, vy]; some
manoeuvre (a lead that brakes, or a neighbour that cuts in). The ego state is
owned by the pipeline (the bicycle plant); the world only steps the agents.
"""
import numpy as np

LANE = 3.5  # lane width [m]


class Agent:
    def __init__(self, aid, x, y, vx, vy=0.0, kind="car", brake=None, cut_in=None):
        self.id = aid
        self.s = np.array([x, y, vx, vy], float)
        self.kind = kind
        self.brake = brake      # (t_start, decel) -> longitudinal braking
        self.cut_in = cut_in    # (t_start, t_end, y_target) -> lateral lane change
        self.t = 0.0

    def step(self, dt):
        x, y, vx, vy = self.s
        if self.brake is not None:
            t0, dec = self.brake
            if self.t >= t0:
                vx = max(0.0, vx - dec * dt)
        if self.cut_in is not None:
            t0, t1, ytgt = self.cut_in
            if t0 <= self.t <= t1:
                frac = (self.t - t0) / max(1e-6, t1 - t0)
                # smooth lateral move
                y_des = self.s[1]  # current
                # velocity toward target (simple proportional lateral)
                vy = 2.0 * np.tanh((ytgt - y) * 0.8)
            else:
                vy = 0.0
        x += vx * dt; y += vy * dt
        self.s = np.array([x, y, vx, vy], float)
        self.t += dt


class World:
    def __init__(self, agents):
        self.agents = agents
        self.t = 0.0

    def step(self, dt):
        for a in self.agents:
            a.step(dt)
        self.t += dt

    def true_objects(self):
        return [(a.id, a.s.copy(), a.kind) for a in self.agents]


# ---- scenarios ----
def scenario_lead_brake():
    """A lead car in the ego lane, cruising then braking moderately (decel 4)."""
    return World([Agent(1, x=45.0, y=0.0, vx=8.0, brake=(3.0, 4.0), kind="car")])


def scenario_hard_brake():
    """A close lead at matched speed that slams on the brakes (decel 8 = max).

    The IDM planner follows at a comfortable headway and only decelerates within
    comfort limits, so this sudden maximal brake exceeds what it plans for -- the
    safety guardrail's emergency braking is what prevents the rear-end."""
    return World([Agent(1, x=26.0, y=0.0, vx=12.0, brake=(2.0, 8.0), kind="car")])


def scenario_cut_in():
    """A neighbour in the left lane that cuts into the ego lane ahead."""
    return World([Agent(2, x=18.0, y=LANE, vx=9.0, cut_in=(0.5, 3.5, 0.0), kind="car")])


def scenario_mixed():
    """A slow lead in-lane plus a neighbour in the left lane (overtake setup)."""
    return World([
        Agent(1, x=40.0, y=0.0, vx=7.0, kind="car"),
        Agent(3, x=70.0, y=LANE, vx=13.0, kind="car"),
    ])


def scenario_acc_follow():
    """Steady lead at 8 m/s in ego lane; ego settles into ACC following mode.

    No braking event — demonstrates pure ACC gap-keeping at comfortable headway
    (IDM T=1.5 s, s0=5 m).  No guardrail override expected."""
    return World([Agent(1, x=35.0, y=0.0, vx=8.0, kind="car")])


def scenario_lane_avoidance():
    """Very slow obstacle (3 m/s) in ego lane; left lane is clear.

    The IDM cost of crawling behind the obstacle exceeds the lane-change penalty,
    so the planner switches to the left lane and overtakes.  The guardrail stays
    quiet because the manoeuvre keeps safe clearances throughout."""
    return World([Agent(1, x=45.0, y=0.0, vx=3.0, kind="car")])


def scenario_safety_stop():
    """Hard-braking lead + blocker in the left lane → no safe escape route.

    Lead (x=25 m) slams on the brakes at t=1.5 s (decel 8 m/s²).
    A parallel vehicle in the left lane (x=5 m, vx=10 m/s) would block any
    lane-change escape.  The RSS guardrail detects the closing gap to the lead,
    applies emergency braking (-6 m/s²) with zero steering (straight-line stop),
    and brings ego to rest before a collision occurs."""
    return World([
        Agent(1, x=25.0, y=0.0,  vx=12.0, brake=(1.5, 8.0), kind="car"),
        Agent(2, x=5.0,  y=LANE, vx=10.0, kind="car"),
    ])
