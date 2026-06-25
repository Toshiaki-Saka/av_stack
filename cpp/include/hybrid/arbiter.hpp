#pragma once
#include <algorithm>
#include <cmath>

namespace hybrid {

enum class Source { NN, MPC, BLEND, FAILSAFE };

struct ArbiterResult {
    double steer  = 0.0;
    Source source = Source::FAILSAFE;
};

// Confidence-weighted arbiter: merges a neural (E2E) steer and a modular (MPC)
// steer according to their real-time confidence scores.
//
// Priority:
//   1. Both channels below thresh_failsafe  → FAILSAFE (steer = 0)
//   2. One channel clearly dominates        → winner-takes-all (NN or MPC)
//   3. Otherwise                            → confidence-weighted BLEND
class Arbiter {
public:
    double thresh_failsafe = 0.10;  // below this → channel considered dead
    double thresh_dominant = 0.60;  // one channel must exceed this to be sole winner

    ArbiterResult arbitrate(double steer_nn,  double conf_nn,
                            double steer_mpc, double conf_mpc) const
    {
        if (conf_nn < thresh_failsafe && conf_mpc < thresh_failsafe)
            return {0.0, Source::FAILSAFE};

        double total = conf_nn + conf_mpc;
        if (total < 1e-12) return {0.0, Source::FAILSAFE};

        if (conf_nn  >= thresh_dominant && conf_nn  > conf_mpc * 1.5)
            return {std::max(-0.6, std::min(0.6, steer_nn)),  Source::NN};
        if (conf_mpc >= thresh_dominant && conf_mpc > conf_nn  * 1.5)
            return {std::max(-0.6, std::min(0.6, steer_mpc)), Source::MPC};

        double w_nn  = conf_nn  / total;
        double w_mpc = conf_mpc / total;
        double blended = w_nn * steer_nn + w_mpc * steer_mpc;
        return {std::max(-0.6, std::min(0.6, blended)), Source::BLEND};
    }
};

}  // namespace hybrid
