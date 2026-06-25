"""Hybrid arbiter — confidence-weighted fusion of NN (E2E) and MPC steering.

This module wraps hybrid_cpp and adds a stateful HybridArbiter that computes
confidence scores from live stack signals and delegates arbitration to C++.

Typical usage
-------------
    arbiter = HybridArbiter()
    steer, source = arbiter.step(
        steer_nn=0.05,   epistemic_std=0.02, ood_distance=0.3,
        steer_mpc=-0.03, tracking_residual=0.1,
    )
"""
import sys, os

for _d in [
    os.path.join(os.path.dirname(__file__), "..", "build", "Release"),
    os.path.join(os.path.dirname(__file__), "..", "build"),
]:
    _d = os.path.normpath(_d)
    if os.path.isdir(_d):
        sys.path.insert(0, _d)

import hybrid_cpp


class HybridArbiter:
    """Stateless wrapper; holds tunable thresholds as Python attributes."""

    def __init__(self, thresh_failsafe: float = 0.10, thresh_dominant: float = 0.60,
                 max_steer: float = 0.6):
        self._arb = hybrid_cpp.Arbiter()
        self._arb.thresh_failsafe = thresh_failsafe
        self._arb.thresh_dominant = thresh_dominant
        self.max_steer = max_steer

    def step(self, *,
             steer_nn: float, epistemic_std: float, ood_distance: float,
             steer_mpc: float, tracking_residual: float) -> tuple[float, str]:
        """Compute confidences and arbitrate.

        Returns (steer_out, source_name) where source_name is one of
        'NN', 'MPC', 'BLEND', 'FAILSAFE'.
        """
        conf_nn  = hybrid_cpp.confidence_e2e(epistemic_std, ood_distance)
        conf_mpc = hybrid_cpp.confidence_modular(tracking_residual, steer_mpc,
                                                  self.max_steer)
        result = self._arb.arbitrate(steer_nn, conf_nn, steer_mpc, conf_mpc)
        return result.steer, result.source.name
