"""Tests for EKF localizer (localization_cpp module)."""
import math
import sys
import os

# Search build output directories for the compiled module.
_build_dirs = [
    os.path.join(os.path.dirname(__file__), "..", "build", "Release"),
    os.path.join(os.path.dirname(__file__), "..", "build"),
]
for _d in _build_dirs:
    _d = os.path.normpath(_d)
    if os.path.isdir(_d):
        sys.path.insert(0, _d)

import localization_cpp  # noqa: E402  (must come after sys.path setup)


def _make_ekf():
    return localization_cpp.EkfLocalizer()


def test_straight_no_noise():
    """No-noise straight run: GT and estimate must agree within 0.01 m."""
    ekf = _make_ekf()
    v = 5.0      # m/s
    dt = 0.1
    steps = 20   # 2 seconds → GT x = 10.0 m

    for _ in range(steps):
        ekf.predict(v, 0.0, dt)
        # Perfect measurement at every step.
        x_gt = v * (_ + 1) * dt
        ekf.update(x_gt, 0.0)

    x, y, yaw = ekf.localize()
    x_gt = v * steps * dt
    assert abs(x - x_gt) < 0.01, f"x error too large: {abs(x - x_gt):.4f} m"
    assert abs(y) < 0.01,         f"y error too large: {abs(y):.4f} m"
    assert abs(yaw) < 0.01,       f"yaw error too large: {abs(yaw):.4f} rad"


def test_noisy_convergence():
    """With noisy measurements, error must be < 0.5 m after 10 steps."""
    import random
    random.seed(42)

    ekf = _make_ekf()
    v = 5.0
    dt = 0.1
    noise_std = 0.3  # matches meas_noise_pos default

    for step in range(10):
        ekf.predict(v, 0.0, dt)
        x_gt = v * (step + 1) * dt
        mx = x_gt + random.gauss(0.0, noise_std)
        my = random.gauss(0.0, noise_std)
        ekf.update(mx, my)

    x, y, yaw = ekf.localize()
    x_gt = v * 10 * dt
    dist = math.hypot(x - x_gt, y - 0.0)
    assert dist < 0.5, f"position error too large after 10 steps: {dist:.4f} m"


def test_reset():
    """After reset(), localize() must return the initial state."""
    ekf = _make_ekf()
    for _ in range(5):
        ekf.predict(5.0, 0.1, 0.1)
        ekf.update(1.0, 0.5)

    ekf.reset(0.0, 0.0, 0.0)
    x, y, yaw = ekf.localize()
    assert x == 0.0 and y == 0.0 and yaw == 0.0, \
        f"reset did not restore initial state: ({x}, {y}, {yaw})"
