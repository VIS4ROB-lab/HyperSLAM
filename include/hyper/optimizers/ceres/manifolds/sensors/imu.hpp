/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/optimizers/ceres/manifolds/sensors/sensor.hpp"
#include "hyper/sensors/imu.hpp"

namespace hyper {

template <>
class Manifold<IMU, OptimizerSuite::CERES> final
    : public Manifold<Sensor, OptimizerSuite::CERES> {
 public:
  /// Default constructor.
  /// \param imu IMU to parametrize.
  /// \param constant Constancy flag.
  explicit Manifold(const IMU& imu, bool constant = true);

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] auto sensor() const -> const IMU& final;

  /// Gyroscope intrinsics manifold accessor.
  [[nodiscard]] auto gyroscopeIntrinsicsManifold() const -> ceres::Manifold*;

  /// Sets the gyroscope intrinsics manifold.
  /// \param manifold Input manifold.
  auto setGyroscopeIntrinsicsManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void;

  /// Sets the sensor's gyroscope intrinsics constant or variable.
  /// \param constant Constancy flag.
  auto setGyroscopeIntrinsicsConstant(bool constant) -> void;

  /// Accelerometer intrinsics manifold accessor.
  [[nodiscard]] auto accelerometerIntrinsicsManifold() const -> ceres::Manifold*;

  /// Sets the accelerometer intrinsics manifold.
  /// \param manifold Input manifold.
  auto setAccelerometerIntrinsicsManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void;

  /// Sets the sensor's accelerometer intrinsics constant or variable.
  /// \param constant Constancy flag.
  auto setAccelerometerIntrinsicsConstant(bool constant) -> void;

  /// Gyroscope bias manifold accessor.
  [[nodiscard]] auto gyroscopeBiasManifold() const -> ceres::Manifold*;

  /// Sets the gyroscope bias manifold.
  /// \param manifold Input manifold.
  auto setGyroscopeBiasManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void;

  /// Sets the sensor's gyroscope bias constant or variable.
  /// \param constant Constancy flag.
  auto setGyroscopeBiasConstant(bool constant) -> void;

  /// Accelerometer bias manifold accessor.
  [[nodiscard]] auto accelerometerBiasManifold() const -> ceres::Manifold*;

  /// Sets the accelerometer bias manifold.
  /// \param manifold Input manifold.
  auto setAccelerometerBiasManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void;

  /// Sets the sensor's accelerometer bias constant or variable.
  /// \param constant Constancy flag.
  auto setAccelerometerBiasConstant(bool constant) -> void;

  /// Retrieves all manifolds.
  /// \return Manifolds.
  [[nodiscard]] auto manifolds() const -> Pointers<ceres::Manifold> final;

  /// Retrieves all manifold (stamp-based).
  /// \param stamp Query stamp.
  /// \return Manifolds.
  [[nodiscard]] auto manifolds(const Stamp& stamp) const -> Pointers<ceres::Manifold> final;

 private:
  /// Assembles multiple manifolds.
  /// \param num_gyroscope_bias_manifolds Number of gyroscope bias manifolds.
  /// \param num_accelerometer_bias_manifolds Number of accelerometer bias manifolds.
  /// \return Collection of manifolds.
  [[nodiscard]] auto assembleManifolds(const Size& num_gyroscope_bias_manifolds, const Size& num_accelerometer_bias_manifolds) const -> Pointers<ceres::Manifold>;
};

} // namespace hyper
