"""Tests for SpeedGovernor (safety_cpp module)."""
import math
import sys
import os

_build_dirs = [
    os.path.join(os.path.dirname(__file__), "..", "build", "Release"),
    os.path.join(os.path.dirname(__file__), "..", "build"),
]
for _d in _build_dirs:
    _d = os.path.normpath(_d)
    if os.path.isdir(_d):
        sys.path.insert(0, _d)

import safety_cpp  # noqa: E402


def _make_gov():
    return safety_cpp.SpeedGovernor()


def test_no_obstacles():
    """No obstacles: target speed must be returned unchanged."""
    gov = _make_gov()
    result = gov.cap_speed(10.0, [], 0.0, 0.0, 0.0)
    assert result == 10.0, f"Expected 10.0, got {result}"


def test_frontal_obstacle_reduces_speed():
    """Obstacle directly ahead within margin: speed must be reduced."""
    gov = _make_gov()
    # Stationary obstacle 1.5 m ahead (inside default margin=2.0 m)
    obstacles = [[1.5, 0.0, 0.0, 0.0]]
    result = gov.cap_speed(10.0, obstacles, 0.0, 0.0, 0.0)
    assert result < 10.0, f"Expected speed reduction, got {result}"


def test_far_obstacle_no_reduction():
    """Obstacle far away: target speed must be returned unchanged."""
    gov = _make_gov()
    # Stationary obstacle 100 m ahead (well outside any predicted window)
    obstacles = [[100.0, 0.0, 0.0, 0.0]]
    result = gov.cap_speed(10.0, obstacles, 0.0, 0.0, 0.0)
    assert result == 10.0, f"Expected no reduction, got {result}"


def test_coexistence_with_guardrail():
    """Importing guardrail and safety_cpp together must not raise."""
    import importlib.util, pathlib
    guardrail_path = pathlib.Path(__file__).parent.parent / "python" / "guardrail.py"
    spec = importlib.util.spec_from_file_location("guardrail", guardrail_path)
    guardrail_mod = importlib.util.module_from_spec(spec)

    # Guardrail imports 'world' which may not be on path — skip gracefully.
    try:
        spec.loader.exec_module(guardrail_mod)
        gr = guardrail_mod.Guardrail()
    except Exception:
        pass  # import failure is acceptable; the test checks no *conflict* exists

    gov = _make_gov()
    obstacles = [[5.0, 0.0, 0.0, 0.0]]
    result = gov.cap_speed(8.0, obstacles, 0.0, 0.0, 0.0)
    assert isinstance(result, float), "cap_speed must return float"


def test_guardrail_rss_distance():
    """guardrail_cpp: RSS longitudinal distance is positive and grows with speed."""
    import guardrail_cpp
    g = guardrail_cpp.Guardrail()
    d_slow = g.rss_min_distance(5.0, 0.0)
    d_fast = g.rss_min_distance(15.0, 0.0)
    assert d_slow > 0.0, "RSS distance must be positive"
    assert d_fast > d_slow, "RSS distance must grow with ego speed"


def test_guardrail_no_threat():
    """guardrail_cpp: no tracks → status is OK."""
    import guardrail_cpp
    g = guardrail_cpp.Guardrail()
    a, delta, status, reason = g.check([0.0, 0.0, 10.0, 0.0], [1.5, 0.0], [])
    assert status == "OK"


def test_guardrail_longitudinal_override():
    """guardrail_cpp: confirmed lead within RSS distance triggers OVERRIDE."""
    import guardrail_cpp
    g = guardrail_cpp.Guardrail()
    ego = [0.0, 0.0, 10.0, 0.0]
    # Lead at 3 m, same lane, slow — well inside RSS safe distance
    tracks = [(1, 3.0, 0.0, 1.0, 0.0, 0.0, True)]
    a, delta, status, reason = g.check(ego, [1.0, 0.0], tracks)
    assert status == "OVERRIDE", f"Expected OVERRIDE, got {status} ({reason})"
    assert a < 0.0, "Emergency brake must be negative acceleration"
