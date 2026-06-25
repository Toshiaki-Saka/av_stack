// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cmath>

#include "ad/common/types.hpp"
#include "ad/localization/localizer.hpp"

namespace ad::localization {

/// Extended Kalman Filter over the 3-state pose [x, y, yaw]. The motion model
/// is the kinematic bicycle driven by the last control (v, steering); the
/// measurement is a noisy absolute position (e.g. GNSS) of (x, y).
///
/// Drive it from the simulation loop:
///   ekf.predict(cmd, dt);          // after computing the control
///   ekf.update(measured_position); // when a position fix arrives
/// `localize()` then returns the current fused estimate.
class EkfLocalizer final : public ILocalizer {
 public:
  struct Params {
    double wheelbase{2.7};
    double process_pos{0.05};   ///< Process noise std on x, y [m].
    double process_yaw{0.02};   ///< Process noise std on yaw [rad].
    double meas_pos{0.30};      ///< Measurement noise std on x, y [m].
  };

  EkfLocalizer(const common::VehicleState& initial, Params params)
      : params_{params} {
    state_ = initial;
    // Small initial covariance (we trust the start pose).
    P_ = {0.10, 0.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0, 0.05};
  }

  /// Propagate the estimate forward under the control input.
  void predict(const common::ControlCommand& cmd, double dt) {
    const double yaw = state_.pose.yaw;
    const double v = state_.velocity;

    // State propagation (kinematic bicycle).
    state_.pose.x += v * std::cos(yaw) * dt;
    state_.pose.y += v * std::sin(yaw) * dt;
    state_.pose.yaw = common::normalize_angle(
        yaw + (v / params_.wheelbase) * std::tan(cmd.steering_angle) * dt);
    double next_v = v + cmd.acceleration * dt;
    if (next_v < 0.0) {
      next_v = 0.0;
    }
    state_.velocity = next_v;

    // Jacobian F = d f / d [x,y,yaw] (v treated as a known input here).
    const double dfx_dyaw = -v * std::sin(yaw) * dt;
    const double dfy_dyaw = v * std::cos(yaw) * dt;
    const std::array<double, 9> F{1.0, 0.0, dfx_dyaw,
                                  0.0, 1.0, dfy_dyaw,
                                  0.0, 0.0, 1.0};

    // P = F P F^T + Q
    const std::array<double, 9> FP = mul(F, P_);
    P_ = add(mul_t(FP, F), process_q());
  }

  /// Fuse an absolute (x, y) position measurement.
  void update(const common::Point2D& measured) {
    // Innovation y = z - H x, with H selecting (x, y).
    const double yx = measured.x - state_.pose.x;
    const double yy = measured.y - state_.pose.y;

    const double r = params_.meas_pos * params_.meas_pos;
    // S = H P H^T + R  (2x2, top-left block of P plus R on the diagonal).
    const double s00 = P_[0] + r;
    const double s01 = P_[1];
    const double s10 = P_[3];
    const double s11 = P_[4] + r;
    const double det = s00 * s11 - s01 * s10;
    if (std::abs(det) < 1e-12) {
      return;
    }
    const double inv00 = s11 / det;
    const double inv01 = -s01 / det;
    const double inv10 = -s10 / det;
    const double inv11 = s00 / det;

    // Kalman gain K = P H^T S^-1  (3x2). H^T picks columns 0 and 1 of P.
    std::array<double, 6> K{};
    for (int row = 0; row < 3; ++row) {
      const double p0 = P_[static_cast<std::size_t>(row) * 3 + 0];
      const double p1 = P_[static_cast<std::size_t>(row) * 3 + 1];
      K[static_cast<std::size_t>(row) * 2 + 0] = p0 * inv00 + p1 * inv10;
      K[static_cast<std::size_t>(row) * 2 + 1] = p0 * inv01 + p1 * inv11;
    }

    // State correction.
    state_.pose.x += K[0] * yx + K[1] * yy;
    state_.pose.y += K[2] * yx + K[3] * yy;
    state_.pose.yaw =
        common::normalize_angle(state_.pose.yaw + K[4] * yx + K[5] * yy);

    // Covariance update: P = (I - K H) P.
    std::array<double, 9> KH{};  // 3x3, H maps to first two state columns.
    for (int row = 0; row < 3; ++row) {
      KH[static_cast<std::size_t>(row) * 3 + 0] =
          K[static_cast<std::size_t>(row) * 2 + 0];
      KH[static_cast<std::size_t>(row) * 3 + 1] =
          K[static_cast<std::size_t>(row) * 2 + 1];
    }
    std::array<double, 9> ImKH{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    for (std::size_t i = 0; i < 9; ++i) {
      ImKH[i] -= KH[i];
    }
    P_ = mul(ImKH, P_);
  }

  [[nodiscard]] common::VehicleState localize() const override {
    return state_;
  }

 private:
  // Row-major 3x3 helpers.
  static std::array<double, 9> mul(const std::array<double, 9>& a,
                                   const std::array<double, 9>& b) {
    std::array<double, 9> c{};
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        double sum = 0.0;
        for (std::size_t k = 0; k < 3; ++k) {
          sum += a[i * 3 + k] * b[k * 3 + j];
        }
        c[i * 3 + j] = sum;
      }
    }
    return c;
  }

  // a * b^T for 3x3 matrices.
  static std::array<double, 9> mul_t(const std::array<double, 9>& a,
                                     const std::array<double, 9>& b) {
    std::array<double, 9> c{};
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        double sum = 0.0;
        for (std::size_t k = 0; k < 3; ++k) {
          sum += a[i * 3 + k] * b[j * 3 + k];
        }
        c[i * 3 + j] = sum;
      }
    }
    return c;
  }

  static std::array<double, 9> add(const std::array<double, 9>& a,
                                   const std::array<double, 9>& b) {
    std::array<double, 9> c{};
    for (std::size_t i = 0; i < 9; ++i) {
      c[i] = a[i] + b[i];
    }
    return c;
  }

  [[nodiscard]] std::array<double, 9> process_q() const {
    const double qp = params_.process_pos * params_.process_pos;
    const double qy = params_.process_yaw * params_.process_yaw;
    return {qp, 0.0, 0.0, 0.0, qp, 0.0, 0.0, 0.0, qy};
  }

  Params params_;
  common::VehicleState state_{};
  std::array<double, 9> P_{};  ///< Row-major 3x3 covariance.
};

}  // namespace ad::localization
