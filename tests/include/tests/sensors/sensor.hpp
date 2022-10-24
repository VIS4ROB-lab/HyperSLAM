/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/sensor.hpp"

#include "tests/random.hpp"

namespace hyper::tests {

template <>
class Mock<Sensor> {
 public:
  /// Deleted default constructor.
  Mock() = delete;

  /// Creates a sensor.
  /// \return Sensor.
  static auto Create() -> std::unique_ptr<Sensor> {
    auto sensor = std::make_unique<Sensor>();
    sensor->transformation() = Mock<Traits<Sensor>::Transformation>::Random();
    return sensor;
  }
};

} // namespace hyper::tests
