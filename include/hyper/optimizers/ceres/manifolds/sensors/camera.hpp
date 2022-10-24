/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/optimizers/ceres/manifolds/sensors/sensor.hpp"
#include "hyper/sensors/camera.hpp"

namespace hyper {

template <>
class Manifold<Camera, OptimizerSuite::CERES> final
    : public Manifold<Sensor, OptimizerSuite::CERES> {
 public:
  /// Default constructor.
  /// \param camera Camera to parametrize.
  /// \param constant Constancy flag.
  explicit Manifold(const Camera& camera, bool constant = true);

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] auto sensor() const -> const Camera& final;

  /// Intrinsics manifold accessor.
  [[nodiscard]] auto intrinsicsManifold() const -> ceres::Manifold*;

  /// Sets the intrinsics manifold.
  /// \param manifold Input manifold.
  auto setIntrinsicsManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void;

  /// Sets the intrinsics constant.
  /// \param constant Constancy flag.
  auto setIntrinsicsConstant(bool constant) -> void;

  /// Distortion manifold accessor.
  [[nodiscard]] auto distortionManifold() const -> ceres::Manifold*;

  /// Sets the distortion manifold.
  /// \param manifold Input manifold.
  auto setDistortionManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void;

  /// Sets the distortion constant.
  /// \param constant Constancy flag.
  auto setDistortionConstant(bool constant) -> void;
};

} // namespace hyper