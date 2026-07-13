"""
av-stack animation demo (pure Python — no C++ build required)

Reimplements the whole pipeline — perception -> world model -> planning (IDM) ->
safety guardrail -> controller — in Python and visualises it as a real-time
animation.

Scenarios:
  * The ego (blue) is cruising when the lead vehicle (orange) brakes hard
    -> the guardrail commands emergency braking.
  * A vehicle in the left lane (green) cuts in -> lateral RSS fires.
"""
import sys
import numpy as np
import matplotlib
# Try TkAgg first; fall back to the system default if Tk is not available.
try:
    matplotlib.use("TkAgg")
    import tkinter  # noqa: F401 — just checking availability
except Exception:
    matplotlib.use("Agg")   # headless fallback (saves no window; useful in CI)
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec

# ─────────────────────────── constants ────────────────────────────
DT        = 0.05          # simulation timestep [s]
LANE      = 3.5           # lane width [m]
VEH_L     = 4.5           # vehicle length [m]
VEH_W     = 1.9           # vehicle width [m]
V_DES     = 13.0          # desired ego speed [m/s] (~47 km/h)
A_MAX     = 3.0
A_MIN     = -8.0
DELTA_MAX = 0.5
WHEELBASE = 2.7

# ─────────────────────────── bicycle model ────────────────────────
def bicycle_step(s, a, delta, dt):
    """RK4 kinematic bicycle.  s = [x, y, psi, v]"""
    def f(s):
        x, y, psi, v = s
        d = np.clip(delta, -DELTA_MAX, DELTA_MAX)
        return np.array([
            v * np.cos(psi),
            v * np.sin(psi),
            v / WHEELBASE * np.tan(d),
            np.clip(a, A_MIN, A_MAX),
        ])
    k1 = f(s)
    k2 = f(s + dt / 2 * k1)
    k3 = f(s + dt / 2 * k2)
    k4 = f(s + dt * k3)
    ns = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    ns[3] = max(0.0, ns[3])
    return ns

# ─────────────────────────── PID controller ───────────────────────
class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, ilim=5.0):
        self.kp, self.ki, self.kd, self.ilim = kp, ki, kd, ilim
        self._i = self._prev = 0.0

    def step(self, err, dt):
        self._i = np.clip(self._i + err * dt, -self.ilim, self.ilim)
        d = (err - self._prev) / max(dt, 1e-9)
        self._prev = err
        return self.kp * err + self.ki * self._i + self.kd * d

# ─────────────────────────── IDM planner ──────────────────────────
def idm_accel(v, gap, v_lead, v_des=V_DES, a=1.5, b=2.0, T=1.5, s0=5.0):
    dv   = v - v_lead
    s_st = s0 + max(0.0, v * T + v * dv / (2 * np.sqrt(a * b)))
    gap  = max(gap, 0.1)
    return float(np.clip(a * (1 - (v / max(v_des, 0.1)) ** 4 - (s_st / gap) ** 2),
                         -b * 1.5, a))

# ─────────────────────────── safety guardrail ─────────────────────
class Guardrail:
    def __init__(self):
        self.latch = 0

    def rss_long(self, v_ego, v_lead, rho=0.4, a=1.0, b=4.0, bl=8.0):
        d = v_ego * rho + 0.5 * a * rho**2 + (v_ego + rho * a)**2 / (2 * b) \
            - v_lead**2 / (2 * bl)
        return max(0.0, d)

    def check(self, ego, a_cmd, agents):
        reasons = []
        x_e, y_e, _, v_e = ego
        for ag in agents:
            x_a, y_a, vx_a, vy_a = ag["s"]
            # longitudinal lead
            if abs(y_a - y_e) < LANE * 0.5 and x_a > x_e:
                gap = x_a - x_e - VEH_L
                d_rss = self.rss_long(v_e, max(0.0, vx_a))
                if gap < d_rss:
                    reasons.append(f"RSS {gap:.1f}<{d_rss:.1f}m")
                closing = v_e - vx_a
                if closing > 0.1:
                    ttc = gap / closing
                    if ttc < 2.5:
                        reasons.append(f"TTC {ttc:.1f}s")
            # lateral cut-in RSS
            lat_gap = abs(y_a - y_e)
            approaching = (y_a - y_e) * vy_a < -0.05
            if approaching and lat_gap < 2.5:
                reasons.append(f"lateral {lat_gap:.1f}m")

        if reasons:
            self.latch = 12
        if self.latch > 0:
            self.latch -= 1
            return -6.0, "OVERRIDE", "; ".join(reasons) or "latched"
        return a_cmd, "OK", ""

# ─────────────────────────── scenario ────────────────────────────
class AgentSim:
    def __init__(self, aid, x, y, vx, vy=0.0, color="orange",
                 brake=None, cut_in=None):
        self.id   = aid
        self.s    = np.array([x, y, vx, vy], float)
        self.color = color
        self.brake  = brake    # (t_start, decel)
        self.cut_in = cut_in   # (t_start, t_end, y_target)
        self.t    = 0.0

    def step(self, dt):
        x, y, vx, vy = self.s
        if self.brake and self.t >= self.brake[0]:
            vx = max(0.0, vx - self.brake[1] * dt)
        if self.cut_in:
            t0, t1, ytgt = self.cut_in
            if t0 <= self.t <= t1:
                vy = 2.0 * np.tanh((ytgt - y) * 0.8)
            else:
                vy = 0.0
        x += vx * dt; y += vy * dt
        self.s = np.array([x, y, vx, vy], float)
        self.t += dt

# ─────────────────────────── simulation state ────────────────────
agents = [
    AgentSim(1, x=40.0, y=0.0,    vx=12.0, color="tomato",
             brake=(3.0, 8.0)),
    AgentSim(2, x=30.0, y=LANE,   vx=11.0, color="limegreen",
             cut_in=(2.0, 5.0, 0.0)),
]

ego       = np.array([0.0, 0.0, 0.0, 10.0])   # [x, y, psi, v]
spd_pid   = PID(kp=1.5, ki=0.4, kd=0.0)
steer_pid = PID(kp=0.25, ki=0.0, kd=0.3)
guardrail = Guardrail()

# history buffers
MAX_HIS = 400
t_hist   = []
v_hist   = []
a_hist   = []
gap_hist = []
status_hist = []
ego_x_hist  = []
ego_y_hist  = []
ag_his   = {ag.id: {"x": [], "y": []} for ag in agents}

t_sim = 0.0

# ─────────────────────────── figure setup ────────────────────────
fig = plt.figure(figsize=(15, 9), facecolor="#1a1a2e")
fig.suptitle("av-stack  |  perception -> planning (IDM) -> guardrail -> controller",
             color="white", fontsize=13, fontweight="bold", y=0.98)
gs = GridSpec(3, 2, figure=fig, left=0.06, right=0.97,
              top=0.93, bottom=0.07, hspace=0.55, wspace=0.35)

ax_road  = fig.add_subplot(gs[0, :])    # top: bird's-eye road view
ax_speed = fig.add_subplot(gs[1, 0])
ax_accel = fig.add_subplot(gs[1, 1])
ax_gap   = fig.add_subplot(gs[2, 0])
ax_stat  = fig.add_subplot(gs[2, 1])

for ax in [ax_road, ax_speed, ax_accel, ax_gap, ax_stat]:
    ax.set_facecolor("#0f0f23")
    ax.tick_params(colors="white", labelsize=8)
    for sp in ax.spines.values():
        sp.set_edgecolor("#444")

ROAD_HALF = 8.0    # y range for road view

def _road_bg(ax):
    ax.set_facecolor("#222244")
    for ly in [-LANE * 1.5, LANE * 1.5]:
        ax.axhline(ly, color="#555", lw=1.5, ls="-")   # road edge
    ax.axhline(0,    color="#888", lw=0.8, ls="--")     # lane boundary
    ax.axhline(LANE, color="#888", lw=0.8, ls="--")
    ax.set_ylim(-ROAD_HALF, ROAD_HALF)

def draw_car(ax, x, y, psi, color, label=None):
    """Draw a simple rectangle for a vehicle."""
    cos_p, sin_p = np.cos(psi), np.sin(psi)
    corners_local = np.array([
        [ VEH_L / 2,  VEH_W / 2],
        [-VEH_L / 2,  VEH_W / 2],
        [-VEH_L / 2, -VEH_W / 2],
        [ VEH_L / 2, -VEH_W / 2],
    ])
    R = np.array([[cos_p, -sin_p], [sin_p, cos_p]])
    corners = corners_local @ R.T + np.array([x, y])
    patch = plt.Polygon(corners, closed=True, color=color, alpha=0.85, zorder=4)
    ax.add_patch(patch)
    if label:
        ax.text(x, y + VEH_W * 0.6 + 0.3, label, color="white",
                fontsize=7, ha="center", va="bottom", zorder=5)

# ─────────────────────────── animation init ───────────────────────
def init():
    for ax in [ax_road, ax_speed, ax_accel, ax_gap, ax_stat]:
        ax.cla()
        ax.set_facecolor("#0f0f23")
        ax.tick_params(colors="white", labelsize=8)
        for sp in ax.spines.values():
            sp.set_edgecolor("#444")

    _road_bg(ax_road)
    ax_road.set_title("Bird's-eye view", color="white", fontsize=9)
    ax_road.set_xlabel("x [m]", color="white", fontsize=8)
    ax_road.set_ylabel("y [m]", color="white", fontsize=8)
    ax_road.yaxis.label.set_color("white")

    ax_speed.set_title("Ego speed", color="white", fontsize=9)
    ax_speed.set_ylabel("v [m/s]", color="white", fontsize=8)
    ax_speed.set_xlabel("t [s]",   color="white", fontsize=8)

    ax_accel.set_title("Acceleration command", color="white", fontsize=9)
    ax_accel.set_ylabel("a [m/s²]", color="white", fontsize=8)
    ax_accel.set_xlabel("t [s]",    color="white", fontsize=8)

    ax_gap.set_title("Gap to lead vehicle", color="white", fontsize=9)
    ax_gap.set_ylabel("gap [m]", color="white", fontsize=8)
    ax_gap.set_xlabel("t [s]",   color="white", fontsize=8)

    ax_stat.set_title("Guardrail status", color="white", fontsize=9)
    ax_stat.set_ylabel("override (1=yes)", color="white", fontsize=8)
    ax_stat.set_xlabel("t [s]",            color="white", fontsize=8)
    return []

# ─────────────────────────── animation update ─────────────────────
def update(frame):
    global ego, t_sim

    # ── run multiple physics steps per frame for smooth animation
    SUBSTEPS = 3
    for _ in range(SUBSTEPS):
        _physics_step()

    # ── redraw
    for ax in [ax_road, ax_speed, ax_accel, ax_gap, ax_stat]:
        ax.cla()
        ax.set_facecolor("#0f0f23")
        ax.tick_params(colors="white", labelsize=8)
        for sp in ax.spines.values():
            sp.set_edgecolor("#444")

    # --- road view ---
    _road_bg(ax_road)
    view_x = ego[0]
    WIN = 55.0
    ax_road.set_xlim(view_x - 10, view_x + WIN)
    ax_road.set_ylim(-ROAD_HALF, ROAD_HALF)

    # dashed lane markings (moving)
    for ox in np.arange(-200, 600, 10):
        for ly in [LANE / 2]:
            if view_x - 10 < ox < view_x + WIN:
                ax_road.plot([ox, ox + 4], [ly, ly], color="#aaa", lw=0.5, ls="-", alpha=0.4)

    # trails
    if len(ego_x_hist) > 2:
        ax_road.plot(ego_x_hist[-120:], ego_y_hist[-120:],
                     color="#4488ff", lw=1.2, alpha=0.5, zorder=2)
    for ag in agents:
        xh = ag_his[ag.id]["x"][-120:]
        yh = ag_his[ag.id]["y"][-120:]
        if len(xh) > 2:
            ax_road.plot(xh, yh, color=ag.color, lw=1.0, alpha=0.4, zorder=2)

    # agents
    for ag in agents:
        draw_car(ax_road, ag.s[0], ag.s[1], 0.0, ag.color,
                 label=f"v={ag.s[2]:.1f}")
    draw_car(ax_road, ego[0], ego[1], ego[2], "#4499ff", label=f"ego {ego[3]:.1f}")

    ax_road.set_title(
        f"t={t_sim:.1f}s  ego={ego[3]:.1f}m/s  "
        f"guardrail:{status_hist[-1] if status_hist else 'OK'}",
        color="white", fontsize=9)
    ax_road.set_xlabel("x [m]", color="white", fontsize=8)
    ax_road.set_ylabel("y [m]", color="white", fontsize=8)

    # lane labels
    ax_road.text(view_x - 9, LANE / 2, "Left lane", color="#aaa", fontsize=7, va="center")
    ax_road.text(view_x - 9, -LANE / 2, "Ego lane", color="#aaa", fontsize=7, va="center")

    # --- time-series plots ---
    t  = np.array(t_hist)
    v  = np.array(v_hist)
    a  = np.array(a_hist)
    g  = np.array(gap_hist)
    st = np.array([1 if s == "OVERRIDE" else 0 for s in status_hist])

    def _plot(ax, xs, ys, color, ylabel, xlabel, title, hlines=None):
        if len(xs) > 1:
            ax.plot(xs, ys, color=color, lw=1.5)
        if hlines:
            for yh, hc, hl in hlines:
                ax.axhline(yh, color=hc, lw=0.8, ls="--", label=hl)
        ax.set_title(title, color="white", fontsize=9)
        ax.set_ylabel(ylabel, color="white", fontsize=8)
        ax.set_xlabel(xlabel, color="white", fontsize=8)
        ax.tick_params(colors="white")
        for sp in ax.spines.values():
            sp.set_edgecolor("#444")
        if len(xs) > 1:
            ax.set_xlim(max(0, t_sim - 20), t_sim + 0.5)

    _plot(ax_speed, t, v, "#4499ff",
          "v [m/s]", "t [s]", "Ego speed",
          hlines=[(V_DES, "#ff8800", f"v_des={V_DES}")])
    ax_speed.legend(fontsize=7, facecolor="#222", edgecolor="#444",
                    labelcolor="white", loc="lower right")

    _plot(ax_accel, t, a, "#ff9933",
          "a [m/s²]", "t [s]", "Acceleration command",
          hlines=[(A_MIN, "#ff4444", "A_MIN")])
    if len(t) > 1:
        ov_mask = np.array(st, dtype=bool)
        if ov_mask.any():
            ax_accel.scatter(t[ov_mask], a[ov_mask], s=18,
                             color="crimson", zorder=5, label="guardrail")
            ax_accel.legend(fontsize=7, facecolor="#222", edgecolor="#444",
                            labelcolor="white")

    _plot(ax_gap, t, g, "#66dd88",
          "gap [m]", "t [s]", "Gap to lead",
          hlines=[(0, "#ff4444", "contact")])
    ax_gap.set_ylim(bottom=-3)

    _plot(ax_stat, t, st, "#ff4466",
          "override", "t [s]", "Guardrail OVERRIDE")
    ax_stat.set_ylim(-0.1, 1.3)
    if len(t) > 1:
        ax_stat.fill_between(t, st, alpha=0.4, color="#ff4466", step="post")

    return []


def _physics_step():
    global ego, t_sim

    # agent step
    for ag in agents:
        ag.step(DT)
        ag_his[ag.id]["x"].append(ag.s[0])
        ag_his[ag.id]["y"].append(ag.s[1])

    # find in-lane lead for IDM
    in_lane_lead = None
    for ag in agents:
        if abs(ag.s[1] - ego[1]) < LANE * 0.5 and ag.s[0] > ego[0]:
            gap = ag.s[0] - ego[0] - VEH_L
            if in_lane_lead is None or gap < in_lane_lead[0]:
                in_lane_lead = (gap, ag.s[2])   # (gap, v_lead)

    # longitudinal: IDM or free cruise
    if in_lane_lead:
        a_idm = idm_accel(ego[3], in_lane_lead[0], in_lane_lead[1])
    else:
        a_idm = idm_accel(ego[3], 1e6, ego[3])  # free road

    # lateral: simple P-controller toward lane center (y=0)
    e_lat = -(ego[1] - 0.0)
    e_hdg = -ego[2]
    delta = float(np.clip(steer_pid.step(e_lat + 3.0 * e_hdg, DT),
                          -DELTA_MAX, DELTA_MAX))

    # guardrail
    a_cmd, status, reason = guardrail.check(
        ego, a_idm, [{"s": ag.s} for ag in agents])

    ego[:] = bicycle_step(ego, a_cmd, delta, DT)

    # compute gap for logging
    gap = 1e9
    for ag in agents:
        if abs(ag.s[1] - ego[1]) < LANE * 0.6 and ag.s[0] > ego[0]:
            g = ag.s[0] - ego[0] - VEH_L
            gap = min(gap, g)

    t_hist.append(t_sim)
    v_hist.append(ego[3])
    a_hist.append(a_cmd)
    gap_hist.append(gap if gap < 1e8 else float("nan"))
    status_hist.append(status)
    ego_x_hist.append(ego[0])
    ego_y_hist.append(ego[1])

    t_sim += DT


ani = FuncAnimation(
    fig, update, init_func=init,
    interval=50,           # ~20 fps
    blit=False,
    cache_frame_data=False,
)

print("=" * 60)
print(" av-stack Animation Demo")
print("=" * 60)
print(f"  Scenario : hard brake (decel 8 m/s^2) + cut-in from left")
print(f"  Ego speed: {V_DES} m/s target (IDM ACC)")
print(f"  Safety   : RSS long+lat / TTC (Guardrail)")
print(f"  Control  : PID speed + PID steering")
print()
print("  Legend")
print("   blue  : Ego vehicle")
print("   red   : Lead vehicle (hard brake)")
print("   green : Cut-in vehicle (left -> right)")
print("   red dots: Guardrail OVERRIDE")
print()
print("  Close the window to exit.")
print("=" * 60)

plt.show()
