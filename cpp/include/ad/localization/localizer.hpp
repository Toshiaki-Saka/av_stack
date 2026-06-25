// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "ad/common/types.hpp"

namespace ad::localization {

/// Abstract localization stage: estimates the ego vehicle state.
class ILocalizer {
 public:
  virtual ~ILocalizer() = default;

  [[nodiscard]] virtual common::VehicleState localize() const = 0;
};

/// In simulation we can observe the true state directly. A real localizer
/// (EKF/UKF over GNSS + IMU + wheel odometry) would replace this. The class
/// holds a reference to the simulator's authoritative state, which must
/// outlive the localizer.
class GroundTruthLocalizer final : public ILocalizer {
 public:
  explicit GroundTruthLocalizer(const common::VehicleState& truth)
      : truth_{truth} {}

  [[nodiscard]] common::VehicleState localize() const override {
    return truth_;
  }

 private:
  const common::VehicleState& truth_;
};

}  // namespace ad::localization
