"""Smoke + tracking test for the C++ control core via Python."""
import os
import subprocess
import sys
import numpy as np

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "build"))
sys.path.insert(0, os.path.join(HERE, "..", "build", "Release"))
sys.path.insert(0, os.path.join(HERE, "..", "python"))


def test_cpp_control_test():
    import platform
    suffix = ".exe" if platform.system() == "Windows" else ""
    candidates = [
        os.path.join(HERE, "..", "build", "Release", "control_test" + suffix),
        os.path.join(HERE, "..", "build", "control_test" + suffix),
    ]
    exe = next((p for p in candidates if os.path.isfile(p)), candidates[0])
    out = subprocess.run([exe], capture_output=True, text=True)
    assert "CONTROL TEST PASS" in out.stdout, out.stdout
    print("cpp control_test: OK")


def test_controllers_track():
    import run_control_demo as R
    from path import Path
    path = Path(v_ref=12.0, dt=R.DT, L=R.L)
    budgets = {"PID": 0.4, "LQR": 0.2, "MPC": 0.2}
    ctls = {"PID": R.make_pid(), "LQR": R.make_lqr(), "MPC": R.make_mpc(path)}
    for name, ctl in ctls.items():
        r = R.run(ctl, path)
        rms = float(np.sqrt(np.mean(r["ey"] ** 2)))
        assert rms < budgets[name], f"{name} RMS cross-track {rms:.3f} exceeds {budgets[name]}"
        assert np.max(np.abs(r["delta"])) <= R.DELTA_MAX + 1e-6, f"{name} steering exceeded limit"
        assert np.max(np.abs(r["a"])) <= R.A_MAX + 1e-6, f"{name} accel exceeded limit"
        print(f"{name}: OK  (RMS cross-track {rms:.3f} m)")


if __name__ == "__main__":
    test_cpp_control_test()
    test_controllers_track()
    print("CONTROL TESTS PASS")
