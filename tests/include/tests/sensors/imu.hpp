/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/imu.hpp"

#include "tests/random.hpp"

namespace hyper::tests {

template <>
class Mock<IMU> {
 public:
  /// Deleted default constructor.
  Mock() = delete;

  /// Creates an IMU.
  /// \return IMU.
  static auto Create() -> std::unique_ptr<IMU> {
    auto imu = std::make_unique<IMU>();
    imu->transformation() = Mock<Traits<IMU>::Transformation>::Random();
    imu->gyroscopeIntrinsics().setIdentity();
    imu->accelerometerIntrinsics().setIdentity();
    return imu;
  }
};

} // namespace hyper::tests
