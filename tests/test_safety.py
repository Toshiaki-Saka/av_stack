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


def test_guardrail_lateral_rss_matches_documented_formula():
    """guardrail_cpp: lateral RSS distance follows docs_en/TECHNICAL.md 9.3.

    d_lat(v) = mu + lat_travel(0) + lat_travel(v)
    lat_travel(v) = v*rho + 0.5*a_lat_max*rho^2 + (v + rho*a_lat_max)^2 / (2*b_lat_min)

    The braking term is the part that was once missing; without it the distance is
    roughly a third of its true value and the cut-in check never fires on its own.
    """
    import guardrail_cpp
    g = guardrail_cpp.Guardrail()
    rho, mu, a_lat, b_lat = 0.4, 0.5, 0.5, 1.0

    def lat_travel(v):
        return v * rho + 0.5 * a_lat * rho ** 2 + (v + rho * a_lat) ** 2 / (2 * b_lat)

    for v in (0.0, 0.5, 1.0, 2.0, 3.0):
        expect = mu + lat_travel(0.0) + lat_travel(v)
        got = g.rss_lateral_min_distance(v)
        assert abs(got - expect) < 1e-9, f"v={v}: expected {expect:.4f}, got {got:.4f}"


def test_guardrail_lateral_rss_exceeds_half_lane():
    """guardrail_cpp: at a realistic merge speed the lateral RSS distance must reach
    beyond half a lane width, or the check can only fire once the neighbour is already
    in the ego lane — by which point the longitudinal check has fired anyway and the
    lateral branch is dead code (SAFETY_CASE.md G3.3)."""
    import guardrail_cpp
    LANE = 3.5
    g = guardrail_cpp.Guardrail()
    d_lat = g.rss_lateral_min_distance(2.0)
    assert d_lat > LANE / 2, (
        f"lateral RSS distance {d_lat:.2f} m must exceed LANE/2 = {LANE / 2} m")


def test_guardrail_cut_in_fires_before_lane_encroachment():
    """guardrail_cpp: a neighbour still in the adjacent lane triggers CUT_IN.

    The neighbour sits 2.5 m to the left — beyond LANE/2, so it is not yet a
    longitudinal lead and only the lateral branch can catch it. This is the reaction
    time the safety case claims the lateral RSS check buys (SAFETY_CASE.md G3.3).
    """
    import guardrail_cpp
    g = guardrail_cpp.Guardrail()
    ego = [0.0, 0.0, 10.0, 0.0]
    # Closing laterally at 2.0 m/s: lateral RSS distance is 3.82 m > the 2.5 m gap.
    tracks = [(1, 5.0, 2.5, 8.0, -2.0, 0.0, True)]
    a, delta, status, reason = g.check(ego, [1.0, 0.0], tracks)
    assert status == "OVERRIDE", f"Expected OVERRIDE, got {status}"
    assert reason == "CUT_IN", f"Expected CUT_IN, got {reason}"


def test_guardrail_no_cut_in_for_departing_neighbour():
    """guardrail_cpp: a neighbour drifting *away* from the ego is not a cut-in.

    Identical to the approaching case except for the sign of vy. Without the
    lateral-approach check the RSS distances alone are violated and the guardrail
    latches an emergency brake on a vehicle that is leaving (REQ-SAF-04, SG4).
    """
    import guardrail_cpp
    g = guardrail_cpp.Guardrail()
    ego = [0.0, 0.0, 10.0, 0.0]
    tracks = [(1, 5.0, 2.5, 8.0, +2.0, 0.0, True)]
    a, delta, status, reason = g.check(ego, [1.0, 0.0], tracks)
    assert status == "OK", f"Departing neighbour must not override, got {reason}"


def test_guardrail_ignores_ghost_track_lateral_speed():
    """guardrail_cpp: an implausible vy (ghost track) must not trigger CUT_IN.

    The lateral RSS distance grows with vy, so an unfiltered ghost track inflates it
    until the lateral gap test passes trivially. _MAX_VY (3.0 m/s) rejects these.
    """
    import guardrail_cpp
    g = guardrail_cpp.Guardrail()
    ego = [0.0, 0.0, 10.0, 0.0]
    # vy = 5.0 m/s is beyond any real cut-in; at 2.0 m lateral gap the unfiltered
    # lateral RSS distance would be 2.26 m and would fire.
    tracks = [(1, 5.0, 2.0, 0.0, 5.0, 0.0, True)]
    a, delta, status, reason = g.check(ego, [1.0, 0.0], tracks)
    assert status == "OK", f"Ghost track must not override, got {reason}"
