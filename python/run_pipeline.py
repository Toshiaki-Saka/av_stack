"""Full modular pipeline, closed-loop on a hard-braking-lead scenario.

  sensors -> perception (fusion) -> world model (IMM tracker + prediction)
          -> planning (sampling) -> SAFETY GUARDRAIL -> MPC -> actuator -> vehicle

The planner predicts other agents with a constant-velocity model, so it is slow to
react to a *hard* brake; the guardrail re-derives safety with conservative RSS
worst-case assumptions and brakes in time. Running with the guardrail disabled vs.
enabled shows its value: the same stack rear-ends the lead without it, stays safe
with it.
"""
import argparse
import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build", "Release"))
import control_cpp   as C    # noqa: E402
import world_cpp     as WC   # noqa: E402
import scenario_cpp  as W    # noqa: E402
import sensors_cpp   as S    # noqa: E402
import perception_cpp as P   # noqa: E402
from planning_cpp  import Planner   # noqa: E402
from guardrail_cpp import Guardrail # noqa: E402

DT, L = 0.1, 2.7
A_MAX, A_MIN, DELTA_MAX = 3.0, -8.0, 0.6
VEH_LEN = 4.5


def build_mpc():
    m = C.MPC(); m.N = 15; m.dt = DT; m.L = L
    m.q_x, m.q_y, m.q_psi, m.q_v = 0.5, 10.0, 5.0, 2.0
    m.r_a, m.r_delta = 1.0, 4.0
    m.a_max, m.a_min, m.delta_max, m.delta_min = A_MAX, A_MIN, DELTA_MAX, -DELTA_MAX
    return m


def run(scenario, use_guardrail, seed=0, steps=110, v_des=13.0, use_lateral=True,
        planner_fault=False):
    S.seed_rng(seed)
    wd = scenario(); suite = S.SensorSuite()
    mot = WC.MultiObjectTracker(); mot.dt = DT; mot.min_hits = 2
    planner = Planner(dt=DT, horizon=4.0, v_des=v_des, L=L, fault=planner_fault)
    guard = Guardrail(use_lateral=use_lateral)
    mpc = build_mpc()

    ego = [0.0, 0.0, 0.0, 11.0]
    car = C.Bicycle(); car.L = L
    log = {"x": [], "v": [], "a": [], "gap": [], "status": [], "behaviour": []}
    min_gap = 1e9; min_dist = 1e9; collided = False; overrides = 0; react_t = None

    for k in range(steps):
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
        sref, uref = traj.mpc_window(ego, mpc.N)
        a, delta = mpc.solve(ego, sref, uref)

        status = "OK"
        if use_guardrail:
            a, delta, status, reason = guard.check(ego, [a, delta], tracks)
            if status == "OVERRIDE":
                overrides += 1
                if react_t is None:
                    react_t = k * DT

        a = float(np.clip(a, A_MIN, A_MAX)); delta = float(np.clip(delta, -DELTA_MAX, DELTA_MAX))
        if ego[3] <= 0.0 and a < 0.0:
            a = 0.0  # prevent reversing when stopped
        ego = car.step(ego, a, delta, DT)

        # ground-truth proximity (evaluation only)
        gap = 1e9
        for _, s, _ in objs:
            d = np.hypot(s[0] - ego[0], s[1] - ego[1])
            min_dist = min(min_dist, d)
            if abs(s[1] - ego[1]) < 1.5 and s[0] > ego[0]:
                gap = min(gap, s[0] - ego[0] - VEH_LEN)
            if abs(s[0] - ego[0]) < VEH_LEN and abs(s[1] - ego[1]) < 1.0:
                collided = True
        if gap < min_gap:
            min_gap = gap
        log["x"].append(ego[0]); log["v"].append(ego[3]); log["a"].append(a)
        log["gap"].append(gap if gap < 1e8 else np.nan)
        log["status"].append(status); log["behaviour"].append(behaviour)
        wd.step(DT)

    return ({k: np.array(v, dtype=object) if k in ("status", "behaviour") else np.array(v)
             for k, v in log.items()}
            | {"min_gap": min_gap, "min_dist": min_dist, "collided": collided,
               "overrides": overrides, "react_t": react_t})


class _SimState:
    """Step-by-step simulation for FuncAnimation (faulty planner + guardrail ON)."""

    AGENT_COLORS = ["tomato", "limegreen", "orange", "violet"]

    def __init__(self, scenario, use_guardrail=True, planner_fault=True,
                 v_des=13.0, seed=0, max_steps=110, lanes=None):
        S.seed_rng(seed)
        self.wd = scenario()
        self.suite = S.SensorSuite()
        self.mot = WC.MultiObjectTracker()
        self.mot.dt = DT
        self.mot.min_hits = 2
        planner_kwargs = dict(dt=DT, horizon=4.0, v_des=v_des, L=L, fault=planner_fault)
        if lanes is not None:
            planner_kwargs["lanes"] = list(lanes)
        self.planner = Planner(**planner_kwargs)
        self.guard = Guardrail()
        self.mpc = build_mpc()
        self.car = C.Bicycle()
        self.car.L = L
        self.use_guardrail = use_guardrail
        self.ego = [0.0, 0.0, 0.0, 11.0]
        self.t = 0.0
        self.max_steps = max_steps
        self.agent_objs = []
        self.tracks = []
        self.last_traj = None
        self.last_status = "OK"
        self.last_behaviour = "CRUISE"
        self.t_hist = []
        self.v_hist = []
        self.a_hist = []
        self.gap_hist = []
        self.rss_hist = []
        self.status_hist = []

    def step(self):
        self.agent_objs = self.wd.true_objects()
        fused = P.perceive(self.suite.measure(self.ego, self.agent_objs))
        dets = []
        for f in fused:
            z0, z1, R00, R01, R10, R11, _kind, has_v, fvx, fvy, _sens = f
            dets.append((z0, z1, R00, R01, R10, R11, int(has_v), float(fvx), float(fvy)))
        self.tracks = self.mot.step(dets)
        preds = [p for p in (self.mot.predict(t[0], self.planner.M)
                              for t in self.tracks if t[6]) if p]
        traj, beh = self.planner.plan(self.ego, preds)
        self.last_traj = traj
        self.last_behaviour = beh
        sref, uref = traj.mpc_window(self.ego, self.mpc.N)
        a, delta = self.mpc.solve(self.ego, sref, uref)
        status = "OK"
        if self.use_guardrail:
            a, delta, status, _ = self.guard.check(self.ego, [a, delta], self.tracks)
        self.last_status = status
        a = float(np.clip(a, A_MIN, A_MAX))
        delta = float(np.clip(delta, -DELTA_MAX, DELTA_MAX))
        if self.ego[3] <= 0.0 and a < 0.0:
            a = 0.0
        self.ego = self.car.step(self.ego, a, delta, DT)
        gap, v_lead = 1e9, 0.0
        for _, s, _ in self.agent_objs:
            if abs(s[1] - self.ego[1]) < 1.5 and s[0] > self.ego[0]:
                g = s[0] - self.ego[0] - VEH_LEN
                if g < gap:
                    gap, v_lead = g, s[2]
        rss = self.guard.rss_min_distance(self.ego[3], max(0.0, v_lead)) \
              if gap < 1e8 else float("nan")
        self.t_hist.append(self.t)
        self.v_hist.append(self.ego[3])
        self.a_hist.append(a)
        self.gap_hist.append(gap if gap < 1e8 else float("nan"))
        self.rss_hist.append(rss)
        self.status_hist.append(status)
        self.wd.step(DT)
        self.t += DT


def _run_animated(state,
                  title_suffix="[scenario: faulty planner + guardrail ON]",
                  scenario_desc="faulty planner tailgates; guardrail applies emergency braking",
                  console_scenario="Scenario : hard brake (decel 8 m/s^2),  FAULTY planner"):
    import matplotlib
    try:
        matplotlib.use("TkAgg")
        import tkinter  # noqa: F401
    except Exception:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.gridspec import GridSpec

    LANE = W.LANE
    ROAD_HALF = 7.0
    VEH_W = 1.9

    fig = plt.figure(figsize=(15, 9), facecolor="#1a1a2e")
    fig.suptitle(
        "av-stack  |  sensors → IMM tracker → planning (IDM+MPC) → safety guardrail → actuator"
        f"   {title_suffix}",
        color="white", fontsize=11, fontweight="bold", y=0.985)
    gs = GridSpec(3, 3, figure=fig, left=0.06, right=0.97,
                  top=0.94, bottom=0.07, hspace=0.60, wspace=0.38)
    ax_road  = fig.add_subplot(gs[0, :])
    ax_speed = fig.add_subplot(gs[1, 0])
    ax_gap   = fig.add_subplot(gs[1, 1])
    ax_accel = fig.add_subplot(gs[1, 2])
    ax_stat  = fig.add_subplot(gs[2, :])

    def _style(ax):
        ax.set_facecolor("#0f0f23")
        ax.tick_params(colors="white", labelsize=8)
        for sp in ax.spines.values():
            sp.set_edgecolor("#444")

    def _road_bg(ax):
        ax.set_facecolor("#222244")
        for ly in [-LANE * 1.5, LANE * 1.5]:
            ax.axhline(ly, color="#555", lw=1.5)
        ax.axhline(LANE / 2, color="#888", lw=0.8, ls="--")

    def _draw_car(ax, x, y, psi, color, label=None):
        cos_p, sin_p = np.cos(psi), np.sin(psi)
        corners = np.array([[ VEH_LEN / 2,  VEH_W / 2],
                             [-VEH_LEN / 2,  VEH_W / 2],
                             [-VEH_LEN / 2, -VEH_W / 2],
                             [ VEH_LEN / 2, -VEH_W / 2]])
        R = np.array([[cos_p, -sin_p], [sin_p, cos_p]])
        corners = corners @ R.T + np.array([x, y])
        ax.add_patch(plt.Polygon(corners, closed=True, color=color, alpha=0.85, zorder=4))
        if label:
            ax.text(x, y + VEH_W * 0.6 + 0.3, label,
                    color="white", fontsize=7, ha="center", va="bottom", zorder=5)

    def update(frame):
        if len(state.t_hist) < state.max_steps:
            state.step()

        for ax in [ax_road, ax_speed, ax_gap, ax_accel, ax_stat]:
            ax.cla()
            _style(ax)

        _road_bg(ax_road)
        ex = state.ego[0]
        WIN = 60.0
        ax_road.set_xlim(ex - 10, ex + WIN)
        ax_road.set_ylim(-ROAD_HALF, ROAD_HALF)

        for ox in np.arange(-200, 800, 10):
            if ex - 10 < ox < ex + WIN:
                ax_road.plot([ox, ox + 4], [LANE / 2, LANE / 2],
                             color="#aaa", lw=0.5, alpha=0.35)

        if state.last_traj is not None:
            traj = state.last_traj
            n = min(traj.n, 25)
            ax_road.plot(traj.x[:n], traj.y[:n], color="#4488ff",
                         lw=1.2, ls="--", alpha=0.65, zorder=3)

        first_track = True
        for tk in state.tracks:
            if not tk[6]:
                continue
            lbl = "tracker (IMM)" if first_track else None
            ax_road.plot(tk[1], tk[2], "o", color="#ffcc00", ms=5,
                         alpha=0.8, zorder=5, label=lbl)
            first_track = False

        for i, (aid, s, kind) in enumerate(state.agent_objs):
            c = _SimState.AGENT_COLORS[i % len(_SimState.AGENT_COLORS)]
            _draw_car(ax_road, s[0], s[1], 0.0, c, label=f"{s[2]:.1f}m/s")

        ego_color = "#ff4444" if state.last_status == "OVERRIDE" else "#4499ff"
        _draw_car(ax_road, state.ego[0], state.ego[1], state.ego[2],
                  ego_color, label=f"ego {state.ego[3]:.1f}m/s")

        title_color = "#ff8844" if state.last_status == "OVERRIDE" else "white"
        ax_road.set_title(
            f"t = {state.t:.1f} s   behaviour: {state.last_behaviour}"
            f"   guardrail: {state.last_status}",
            color=title_color, fontsize=9)
        ax_road.set_xlabel("x [m]", color="white", fontsize=8)
        ax_road.set_ylabel("y [m]", color="white", fontsize=8)
        ax_road.text(ex - 9,  LANE * 0.75, "Left lane", color="#aaa", fontsize=7, va="center")
        ax_road.text(ex - 9, -LANE * 0.50, "Ego lane",  color="#aaa", fontsize=7, va="center")
        if not first_track:
            ax_road.legend(fontsize=7, facecolor="#222", edgecolor="#444",
                           labelcolor="white", loc="upper right")

        t  = np.array(state.t_hist)
        v  = np.array(state.v_hist)
        a  = np.array(state.a_hist)
        gp = np.array(state.gap_hist)
        rs = np.array(state.rss_hist)
        ov = np.array([s == "OVERRIDE" for s in state.status_hist])
        xlim = (max(0.0, state.t - 12.0), state.t + 0.5) if len(t) > 1 else (0, 1)

        if len(t) > 1:
            ax_speed.plot(t, v, color="#4499ff", lw=1.5)
        ax_speed.axhline(state.planner.v_des, color="#ff8800", lw=0.8, ls="--",
                         label=f"v_des = {state.planner.v_des:.0f} m/s")
        ax_speed.set_title("Ego speed", color="white", fontsize=9)
        ax_speed.set_ylabel("v [m/s]", color="white", fontsize=8)
        ax_speed.set_xlabel("t [s]",   color="white", fontsize=8)
        ax_speed.set_xlim(*xlim)
        ax_speed.legend(fontsize=7, facecolor="#222", edgecolor="#444",
                        labelcolor="white", loc="lower right")

        if len(t) > 1:
            ax_gap.plot(t, gp, color="#66dd88", lw=1.5, label="actual gap")
            valid = ~np.isnan(rs)
            if valid.any():
                ax_gap.plot(t[valid], rs[valid], color="#ff4444",
                            lw=1.0, ls="--", label="RSS min dist")
        ax_gap.axhline(0, color="#ff4444", lw=0.8, ls=":")
        ax_gap.set_title("Gap to lead  (dashed = RSS safe dist)", color="white", fontsize=9)
        ax_gap.set_ylabel("gap [m]", color="white", fontsize=8)
        ax_gap.set_xlabel("t [s]",   color="white", fontsize=8)
        ax_gap.set_xlim(*xlim)
        ax_gap.set_ylim(-3, 30)
        ax_gap.legend(fontsize=7, facecolor="#222", edgecolor="#444",
                      labelcolor="white", loc="upper right")

        if len(t) > 1:
            ax_accel.plot(t, a, color="#ff9933", lw=1.5)
            ov_idx = np.where(ov)[0]
            if len(ov_idx):
                ax_accel.scatter(t[ov_idx], a[ov_idx], s=20, color="crimson",
                                 zorder=5, label="guardrail override")
                ax_accel.legend(fontsize=7, facecolor="#222", edgecolor="#444",
                                labelcolor="white")
        ax_accel.axhline(A_MIN, color="#ff4444", lw=0.8, ls="--", label=f"A_MIN={A_MIN}")
        ax_accel.set_title("Acceleration command", color="white", fontsize=9)
        ax_accel.set_ylabel("a [m/s²]", color="white", fontsize=8)
        ax_accel.set_xlabel("t [s]",    color="white", fontsize=8)
        ax_accel.set_xlim(*xlim)

        ov_f = ov.astype(float)
        if len(t) > 1:
            ax_stat.fill_between(t, ov_f, step="post", alpha=0.45, color="#ff4466")
            ax_stat.plot(t, ov_f, color="#ff4466", lw=1.2, drawstyle="steps-post")
        ax_stat.set_ylim(-0.1, 1.3)
        n_ov = int(ov.sum())
        ax_stat.set_title(
            f"Guardrail OVERRIDE events ({n_ov} so far)  —  {scenario_desc}",
            color="white", fontsize=9)
        ax_stat.set_ylabel("1 = OVERRIDE", color="white", fontsize=8)
        ax_stat.set_xlabel("t [s]", color="white", fontsize=8)
        ax_stat.set_xlim(*xlim)

        return []

    def init():
        for ax in [ax_road, ax_speed, ax_gap, ax_accel, ax_stat]:
            ax.cla()
            _style(ax)
        return []

    print()
    print("=" * 62)
    print(" av-stack Pipeline Animation  (close window to exit)")
    print("=" * 62)
    print(f"  {console_scenario}")
    print("  Pipeline : LiDAR+Radar+Camera -> fusion -> IMM-KF MOT")
    print("             -> IDM planner -> MPC controller -> RSS guardrail")
    print()
    print("  blue rect   : ego  (turns RED when guardrail fires)")
    print("  red rect    : lead vehicle (brakes hard at t~2 s)")
    print("  yellow dot  : IMM tracker estimates")
    print("  blue dashed : planned trajectory (MPC reference)")
    print("  red dashed  : RSS minimum safe following distance")
    print("=" * 62)

    ani = FuncAnimation(fig, update, init_func=init,   # noqa: F841
                        interval=100, blit=False, cache_frame_data=False)
    plt.show()


_DEMO_CONFIGS = {
    "acc": dict(
        scenario=W.scenario_acc_follow,
        use_guardrail=True,
        planner_fault=False,
        max_steps=150,
        lanes=(0.0,),
        title_suffix="[ACC following: lead 8 m/s, ego v_des 13 m/s]",
        scenario_desc="ACC following — nominal planner, guardrail as safety net",
        console_scenario="Scenario : ACC following -- lead cruises at 8 m/s, ego v_des = 13 m/s",
    ),
    "avoidance": dict(
        scenario=W.scenario_lane_avoidance,
        use_guardrail=True,
        planner_fault=False,
        max_steps=130,
        title_suffix="[lane avoidance: slow obstacle (3 m/s), planner overtakes left]",
        scenario_desc="planner detects slow obstacle → CHANGE_LANE to left, then CRUISE",
        console_scenario="Scenario : slow obstacle (3 m/s) in ego lane; planner overtakes to left lane",
    ),
    "safety_stop": dict(
        scenario=W.scenario_safety_stop,
        use_guardrail=True,
        planner_fault=True,
        max_steps=80,
        title_suffix="[safety stop: faulty planner + lead hard-brake + left-lane blocked]",
        scenario_desc="faulty planner tailgates; left lane blocked; guardrail brakes straight to stop",
        console_scenario="Scenario : faulty planner + hard brake (8 m/s^2) + left-lane blocker -- guardrail stops ego",
    ),
}


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="av-stack pipeline demo")
    ap.add_argument("--scenario", choices=list(_DEMO_CONFIGS), default=None,
                    help="run a specific demo scenario (acc / avoidance / safety_stop)")
    args = ap.parse_args()

    if args.scenario is not None:
        cfg = _DEMO_CONFIGS[args.scenario]
        state = _SimState(
            cfg["scenario"],
            use_guardrail=cfg["use_guardrail"],
            planner_fault=cfg["planner_fault"],
            max_steps=cfg["max_steps"],
            lanes=cfg.get("lanes"),
        )
        _run_animated(
            state,
            title_suffix=cfg["title_suffix"],
            scenario_desc=cfg["scenario_desc"],
            console_scenario=cfg["console_scenario"],
        )
    else:
        SC = W.scenario_hard_brake
        nom_off = run(SC, use_guardrail=False, steps=70)
        nom_on  = run(SC, use_guardrail=True,  steps=70)
        flt_off = run(SC, use_guardrail=False, steps=70, planner_fault=True)
        flt_on  = run(SC, use_guardrail=True,  steps=70, planner_fault=True)

        print("Hard-braking lead (decel 8 m/s^2):")
        print(f"  NOMINAL planner  : OFF min gap {nom_off['min_gap']:.2f} m (collided {nom_off['collided']}) | "
              f"ON {nom_on['min_gap']:.2f} m, {nom_on['overrides']} overrides  -> planner copes, guardrail adds margin")
        print(f"  FAULTY planner   : OFF min gap {flt_off['min_gap']:.2f} m (collided {flt_off['collided']}) | "
              f"ON {flt_on['min_gap']:.2f} m (collided {flt_on['collided']}), {flt_on['overrides']} overrides  "
              f"-> guardrail catches the fault")

        try:
            import matplotlib as _mpl
            _mpl.use("Agg")
            import matplotlib.pyplot as _plt
            t = np.arange(len(flt_off["x"])) * DT
            fig, ax = _plt.subplots(3, 1, figsize=(11, 8))
            ax[0].axhline(0, color="r", lw=0.8, ls=":", label="contact")
            ax[0].plot(t, flt_off["gap"], label="faulty planner, guardrail OFF")
            ax[0].plot(t, flt_on["gap"],  label="faulty planner, guardrail ON")
            ax[0].plot(t, nom_on["gap"],  color="green", lw=1, ls="--", label="nominal planner")
            ax[0].set_ylabel("gap to lead [m]"); ax[0].set_ylim(-3, 30)
            ax[0].set_title("Hard brake: an independent guardrail catches a planning-channel fault")
            ax[0].legend(fontsize=8)
            ax[1].plot(t, flt_off["v"], label="OFF"); ax[1].plot(t, flt_on["v"], label="ON")
            ax[1].set_ylabel("ego speed [m/s]"); ax[1].set_title("Ego speed (faulty planner)"); ax[1].legend()
            ax[2].plot(t, flt_off["a"], label="OFF"); ax[2].plot(t, flt_on["a"], label="ON")
            ov = [i for i, s in enumerate(flt_on["status"]) if s == "OVERRIDE"]
            if ov:
                ax[2].scatter(t[ov], flt_on["a"][ov], s=12, color="crimson", zorder=5, label="override")
            ax[2].set_ylabel("accel cmd [m/s^2]"); ax[2].set_xlabel("t [s]")
            ax[2].set_title("Acceleration command (red = guardrail override)"); ax[2].legend()
            out = os.path.join(os.path.dirname(__file__), "..", "assets", "pipeline_guardrail.png")
            _plt.tight_layout(); _plt.savefig(out, dpi=95)
            print(f"saved {out}")
        except Exception as e:
            print("plot skipped:", e)

        flt_state = _SimState(W.scenario_hard_brake, use_guardrail=True, planner_fault=True)
        _run_animated(flt_state)
