// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

#include "ad/common/types.hpp"

namespace ad::safety {

/// Constant-velocity prediction of obstacle positions `t` seconds ahead.
[[nodiscard]] std::vector<common::Obstacle> predict_constant_velocity(
    const std::vector<common::Obstacle>& obstacles, double t);

/// Predictive longitudinal safety layer. Given the planned path and the tracked
/// dynamic obstacles, it looks along the path within a reaction window, asks
/// "when would the ego reach this point, and where will each obstacle be then",
/// and caps the target speed so the ego can brake to a stop before the nearest
/// predicted conflict. A real stack would also reason laterally and about
/// intent; this is a transparent worst-case governor.
class SpeedGovernor {
 public:
  struct Params {
    double horizon_time{3.0};    ///< Prediction window [s].
    double react_distance{18.0}; ///< Path look-ahead for conflicts [m].
    double vehicle_radius{1.5};  ///< [m]
    double safety_margin{1.0};   ///< Extra clearance [m].
    double brake_margin{2.0};    ///< Stop this far short of a conflict [m].
    double max_decel{4.0};       ///< Comfort/again braking limit [m/s^2].
    double min_speed_eps{1.0};   ///< Floor used to estimate arrival time [m/s].
  };

  explicit SpeedGovernor(Params params) : params_{params} {}

  /// Returns a (possibly reduced) target speed given the requested one.
  [[nodiscard]] double cap_speed(
      const common::VehicleState& ego, const common::Plan& plan,
      const std::vector<common::Obstacle>& dynamic_obstacles,
      double requested_speed) const;

 private:
  Params params_;
};

}  // namespace ad::safety
