// SPDX-License-Identifier: Apache-2.0
#include "ad/safety/speed_governor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ad::safety {

std::vector<common::Obstacle> predict_constant_velocity(
    const std::vector<common::Obstacle>& obstacles, double t) {
  std::vector<common::Obstacle> predicted = obstacles;
  for (auto& obs : predicted) {
    obs.position.x += obs.velocity.x * t;
    obs.position.y += obs.velocity.y * t;
  }
  return predicted;
}

double SpeedGovernor::cap_speed(
    [[maybe_unused]] const common::VehicleState& ego, const common::Plan& plan,
    const std::vector<common::Obstacle>& dynamic_obstacles,
    double requested_speed) const {
  if (plan.path.empty() || dynamic_obstacles.empty()) {
    return requested_speed;
  }

  const double speed_for_eta = std::max(requested_speed, params_.min_speed_eps);

  // Walk the path ahead, accumulating arc length from the ego.
  double arc = 0.0;
  double conflict_arc = std::numeric_limits<double>::max();

  for (std::size_t i = 1; i < plan.path.size(); ++i) {
    const common::Point2D a{plan.path[i - 1].x, plan.path[i - 1].y};
    const common::Point2D b{plan.path[i].x, plan.path[i].y};
    arc += common::distance(a, b);
    if (arc > params_.react_distance) {
      break;
    }

    // Time at which the ego would reach this point.
    const double eta = std::min(arc / speed_for_eta, params_.horizon_time);
    const std::vector<common::Obstacle> predicted =
        predict_constant_velocity(dynamic_obstacles, eta);

    for (const auto& obs : predicted) {
      const double clearance =
          obs.radius + params_.vehicle_radius + params_.safety_margin;
      if (common::distance(b, obs.position) < clearance) {
        conflict_arc = std::min(conflict_arc, arc);
        break;
      }
    }
  }

  if (conflict_arc == std::numeric_limits<double>::max()) {
    return requested_speed;  // No predicted conflict.
  }

  // Speed from which we can still stop `brake_margin` short of the conflict.
  const double stop_dist = std::max(conflict_arc - params_.brake_margin, 0.0);
  const double safe_speed = std::sqrt(2.0 * params_.max_decel * stop_dist);
  return std::min(requested_speed, safe_speed);
}

}  // namespace ad::safety
