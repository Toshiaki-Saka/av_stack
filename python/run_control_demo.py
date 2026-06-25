"""Closed-loop comparison of the three controllers on a double lane-change.

Each controller drives the same kinematic-bicycle plant (with actuator saturation)
along the same reference. We record the trajectory, the cross-track error, and the
steering command, and plot them side by side. This verifies the C++ control core
end to end and shows the qualitative trade-offs (PID's lag/overshoot vs. LQR's
clean linear regulation vs. MPC's constraint-aware, preview-based tracking).
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "build", "Release"))
import control_cpp as C  # noqa: E402
from path import Path     # noqa: E402

DT, L = 0.1, 2.7
A_MAX, DELTA_MAX = 3.0, 0.6


def actuator(cmd, lo, hi):
    return float(np.clip(cmd, lo, hi))


def run(controller, path, steps=140):
    car = C.Bicycle(); car.L = L
    s = [0.0, 0.8, 0.0, 10.0]          # start offset 0.8 m, below target speed
    log = {"x": [], "y": [], "ey": [], "delta": [], "v": [], "a": []}
    spd = C.PID(); spd.kp, spd.ki, spd.kd = 1.2, 0.3, 0.0
    spd.out_min, spd.out_max = A_MAX * -1, A_MAX; spd.i_min, spd.i_max = -2, 2
    for k in range(steps):
        e_y, e_psi, kappa, v_ref = path.errors(s)
        a, delta = controller(s, e_y, e_psi, kappa, v_ref, spd)
        a = actuator(a, -A_MAX, A_MAX); delta = actuator(delta, -DELTA_MAX, DELTA_MAX)
        s = car.step(s, a, delta, DT)
        log["x"].append(s[0]); log["y"].append(s[1]); log["ey"].append(e_y)
        log["delta"].append(delta); log["v"].append(s[3]); log["a"].append(a)
    return {k: np.array(v) for k, v in log.items()}


# --- three controllers, same speed PID, different steering laws ---
def make_pid():
    lat = C.PID(); lat.kp, lat.ki, lat.kd = 0.18, 0.0, 0.0
    def ctl(s, e_y, e_psi, kappa, v_ref, spd):
        a = spd.step(v_ref - s[3], DT)
        # P on cross-track with heading-error damping (no discrete derivative -> no chatter)
        delta = lat.step(-(e_y + 3.0 * e_psi), DT) + np.arctan(L * kappa)
        return a, delta
    return ctl


def make_lqr():
    lqr = C.LateralLQR(); lqr.L = L; lqr.dt = DT
    lqr.q_ey, lqr.q_epsi, lqr.r_delta = 1.0, 1.0, 1.5
    def ctl(s, e_y, e_psi, kappa, v_ref, spd):
        a = spd.step(v_ref - s[3], DT)
        delta = lqr.steer(e_y, e_psi, s[3], kappa)
        return a, delta
    return ctl


def make_mpc(path):
    mpc = C.MPC(); mpc.N = 15; mpc.dt = DT; mpc.L = L
    mpc.q_x, mpc.q_y, mpc.q_psi, mpc.q_v = 0.5, 10.0, 5.0, 2.0
    mpc.r_a, mpc.r_delta = 1.0, 4.0
    mpc.a_max, mpc.a_min = A_MAX, -A_MAX
    mpc.delta_max, mpc.delta_min = DELTA_MAX, -DELTA_MAX
    def ctl(s, e_y, e_psi, kappa, v_ref, spd):
        sref, uref = path.mpc_window(s, mpc.N)
        a, delta = mpc.solve(s, sref, uref)
        return a, delta
    return ctl


if __name__ == "__main__":
    path = Path(v_ref=12.0, dt=DT, L=L)
    runs = {"PID": run(make_pid(), path),
            "LQR": run(make_lqr(), path),
            "MPC": run(make_mpc(path), path)}

    for name, r in runs.items():
        rms = np.sqrt(np.mean(r["ey"] ** 2))
        print(f"{name}: RMS cross-track={rms:.3f} m, max|e_y|={np.max(np.abs(r['ey'])):.3f} m, "
              f"max|delta|={np.max(np.abs(r['delta'])):.3f}, final v={r['v'][-1]:.2f}")

    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        t = np.arange(len(runs["PID"]["x"])) * DT
        fig, ax = plt.subplots(3, 1, figsize=(11, 9))
        ax[0].plot(path.x, path.y, "k--", lw=1, label="reference")
        for name, r in runs.items():
            ax[0].plot(r["x"], r["y"], label=name)
        ax[0].set_title("Trajectory (double lane-change)"); ax[0].set_xlabel("x [m]")
        ax[0].set_ylabel("y [m]"); ax[0].legend(); ax[0].set_xlim(0, 130)
        for name, r in runs.items():
            ax[1].plot(t, r["ey"], label=name)
        ax[1].axhline(0, color="k", lw=0.5); ax[1].set_title("Cross-track error")
        ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("e_y [m]"); ax[1].legend()
        for name, r in runs.items():
            ax[2].plot(t, r["delta"], label=name)
        ax[2].axhline(DELTA_MAX, color="r", lw=0.6, ls=":"); ax[2].axhline(-DELTA_MAX, color="r", lw=0.6, ls=":")
        ax[2].set_title("Steering command (dotted = limit)"); ax[2].set_xlabel("t [s]")
        ax[2].set_ylabel("delta [rad]"); ax[2].legend()
        out = os.path.join(os.path.dirname(__file__), "..", "assets", "control_compare.png")
        plt.tight_layout(); plt.savefig(out, dpi=95)
        print(f"saved {out}")
    except Exception as e:
        print("plot skipped:", e)
