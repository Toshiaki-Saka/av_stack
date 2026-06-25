#pragma once
#include <cmath>
#include <algorithm>

namespace hybrid {

// Both functions return a value in [0, 1].
//   1.0 = fully trusted  /  0.0 = completely untrustworthy

// Modular (MPC/LQR) channel confidence
// Decays when tracking residual or steering demand is large.
//   tracking_residual : lateral error from trajectory tracker, metres
//   steer             : commanded steering angle, radians
//   max_steer         : saturation threshold (default 0.6 rad ≈ 34°)
inline double confidence_modular(double tracking_residual,
                                  double steer,
                                  double max_steer = 0.6)
{
    double r_penalty = 1.0 / (1.0 + std::exp(3.0 * (std::abs(tracking_residual) - 1.0)));
    double s_ratio   = std::abs(steer) / max_steer;
    double s_penalty = std::max(0.0, 1.0 - s_ratio * s_ratio);
    return r_penalty * s_penalty;
}

// E2E (neural) channel confidence
// Decays when epistemic uncertainty or OOD distance is large.
//   epistemic_std : MC-dropout standard deviation of steer prediction
//   ood_distance  : normalised feature-space distance (0=in-distribution, >1=OOD)
inline double confidence_e2e(double epistemic_std, double ood_distance)
{
    double u_penalty   = std::exp(-4.0 * epistemic_std);
    double ood_penalty = 1.0 / (1.0 + std::exp(5.0 * (ood_distance - 1.0)));
    return u_penalty * ood_penalty;
}

}  // namespace hybrid
