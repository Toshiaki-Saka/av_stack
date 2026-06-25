"""Tests for hybrid_cpp arbiter and confidence functions."""
import sys, os

for _d in [
    os.path.join(os.path.dirname(__file__), "..", "build", "Release"),
    os.path.join(os.path.dirname(__file__), "..", "build"),
]:
    _d = os.path.normpath(_d)
    if os.path.isdir(_d):
        sys.path.insert(0, _d)

import hybrid_cpp


# ---------------------------------------------------------------------------
# confidence_modular
# ---------------------------------------------------------------------------

def test_conf_modular_perfect():
    """Zero residual, zero steer → confidence near 1."""
    c = hybrid_cpp.confidence_modular(0.0, 0.0)
    assert c > 0.9, f"Expected >0.9, got {c}"


def test_conf_modular_large_residual():
    """Large tracking residual → confidence drops."""
    c_good = hybrid_cpp.confidence_modular(0.0, 0.0)
    c_bad  = hybrid_cpp.confidence_modular(3.0, 0.0)
    assert c_bad < c_good, "Confidence must drop with large residual"


def test_conf_modular_saturated_steer():
    """Steering at max → confidence near zero."""
    c = hybrid_cpp.confidence_modular(0.0, 0.6)   # max_steer default = 0.6
    assert c < 0.05, f"Saturated steer → confidence near 0, got {c}"


# ---------------------------------------------------------------------------
# confidence_e2e
# ---------------------------------------------------------------------------

def test_conf_e2e_perfect():
    """Zero uncertainty, zero OOD → confidence near 1."""
    c = hybrid_cpp.confidence_e2e(0.0, 0.0)
    assert c > 0.9, f"Expected >0.9, got {c}"


def test_conf_e2e_high_uncertainty():
    """High epistemic std → low confidence."""
    c = hybrid_cpp.confidence_e2e(2.0, 0.0)
    assert c < 0.01, f"Expected near-zero, got {c}"


def test_conf_e2e_ood():
    """Far OOD → low confidence."""
    c = hybrid_cpp.confidence_e2e(0.0, 3.0)
    assert c < 0.05, f"Expected near-zero for OOD, got {c}"


# ---------------------------------------------------------------------------
# Arbiter
# ---------------------------------------------------------------------------

def test_arbiter_failsafe_both_dead():
    """Both channels below failsafe threshold → FAILSAFE, steer = 0."""
    arb = hybrid_cpp.Arbiter()
    r = arb.arbitrate(0.3, 0.05, -0.3, 0.05)
    assert r.source == hybrid_cpp.Source.FAILSAFE
    assert r.steer == 0.0


def test_arbiter_nn_dominant():
    """NN clearly dominant → source = NN."""
    arb = hybrid_cpp.Arbiter()
    r = arb.arbitrate(0.2, 0.9, -0.1, 0.1)
    assert r.source == hybrid_cpp.Source.NN


def test_arbiter_mpc_dominant():
    """MPC clearly dominant → source = MPC."""
    arb = hybrid_cpp.Arbiter()
    r = arb.arbitrate(0.2, 0.1, -0.1, 0.9)
    assert r.source == hybrid_cpp.Source.MPC


def test_arbiter_blend():
    """Equal confidences → BLEND, output is weighted average."""
    arb = hybrid_cpp.Arbiter()
    r = arb.arbitrate(0.4, 0.5, -0.4, 0.5)
    assert r.source == hybrid_cpp.Source.BLEND
    assert abs(r.steer) < 0.01, "Equal weights → steer ≈ 0"


def test_arbiter_steer_clamped():
    """Output steer is always within [-0.6, 0.6]."""
    arb = hybrid_cpp.Arbiter()
    r = arb.arbitrate(5.0, 0.9, 5.0, 0.1)
    assert -0.6 <= r.steer <= 0.6
