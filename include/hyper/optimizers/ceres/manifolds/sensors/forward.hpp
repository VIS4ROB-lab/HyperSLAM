/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/optimizers/forward.hpp"
#include "hyper/sensors/forward.hpp"

namespace hyper {

template <>
struct Traits<Manifold<Sensor, OptimizerSuite::CERES>> {
  static constexpr auto kNumManifolds = Traits<Sensor>::kNumParameters;
};

template <>
struct Traits<Manifold<Camera, OptimizerSuite::CERES>> {
  static constexpr auto kNumManifolds = Traits<Camera>::kNumParameters;
};

template <>
struct Traits<Manifold<IMU, OptimizerSuite::CERES>> {
  static constexpr auto kGyroscopeBiasManifoldOffset = Traits<IMU>::kNumParameters;
  static constexpr auto kAccelerometerBiasManifoldOffset = kGyroscopeBiasManifoldOffset + 1;
  static constexpr auto kNumManifolds = kAccelerometerBiasManifoldOffset + 1;
};

} // namespace hyper
