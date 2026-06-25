// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace ad::common {

/// Mathematical constant pi (avoids relying on non-portable M_PI).
inline constexpr double kPi = 3.14159265358979323846;

/// A point in the 2D map frame. Units: metres.
struct Point2D {
  double x{0.0};
  double y{0.0};
};

/// A planar pose: position plus heading. yaw is in radians, CCW from +x.
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Full kinematic state of the ego vehicle.
struct VehicleState {
  Pose2D pose{};
  double velocity{0.0};  ///< Longitudinal speed [m/s].
};

/// Actuation command sent to the vehicle (or simulated plant).
struct ControlCommand {
  double acceleration{0.0};    ///< Longitudinal acceleration [m/s^2].
  double steering_angle{0.0};  ///< Front-wheel steering angle [rad].
};

/// A circular obstacle approximation in the map frame. `velocity` defaults to
/// zero, so existing static-obstacle initialisation is unaffected.
struct Obstacle {
  Point2D position{};
  double radius{0.0};     ///< [m]
  Point2D velocity{};     ///< [m/s], map frame (zero => static).
};

/// An ordered sequence of reference poses.
using Trajectory = std::vector<Pose2D>;

/// Output of the planning stage: a reference path plus a desired speed.
struct Plan {
  Trajectory path{};
  double target_speed{0.0};  ///< [m/s]
};

/// Euclidean distance between two points.
[[nodiscard]] inline double distance(const Point2D& a, const Point2D& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

/// Wrap an angle into [-pi, pi].
[[nodiscard]] inline double normalize_angle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

/// Clamp helper that works under -Wconversion without surprises.
[[nodiscard]] inline double clamp(double value, double lo, double hi) {
  if (value < lo) {
    return lo;
  }
  if (value > hi) {
    return hi;
  }
  return value;
}

}  // namespace ad::common
