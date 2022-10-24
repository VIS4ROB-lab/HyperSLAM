/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/optimizers/ceres/manifolds/sensors/imu.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/euclidean.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/stamped.hpp"
#include "hyper/state/abstract.hpp"
#include "hyper/variables/stamped.hpp"

namespace hyper {

Manifold<IMU, OptimizerSuite::CERES>::Manifold(const IMU& imu, const bool constant)
    : Manifold<Sensor, OptimizerSuite::CERES>{imu, Traits<Manifold>::kNumManifolds, constant} {
  setGyroscopeIntrinsicsConstant(constant);
  setAccelerometerIntrinsicsConstant(constant);
  setGyroscopeBiasConstant(constant);
  setAccelerometerBiasConstant(constant);
}

auto Manifold<IMU, OptimizerSuite::CERES>::sensor() const -> const IMU& {
  DCHECK(sensor_ != nullptr);
  return static_cast<const IMU&>(*sensor_); // NOLINT
}

auto Manifold<IMU, OptimizerSuite::CERES>::gyroscopeIntrinsicsManifold() const -> ceres::Manifold* {
  return manifold(Traits<IMU>::kGyroscopeIntrinsicsOffset);
}

auto Manifold<IMU, OptimizerSuite::CERES>::setGyroscopeIntrinsicsManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void {
  DCHECK_EQ(manifold->AmbientSize(), Traits<Traits<IMU>::GyroscopeIntrinsics>::kNumParameters);
  setManifold(Traits<IMU>::kGyroscopeIntrinsicsOffset, std::move(manifold));
}

auto Manifold<IMU, OptimizerSuite::CERES>::setGyroscopeIntrinsicsConstant(const bool constant) -> void {
  constexpr auto kNumParameters = Traits<Traits<IMU>::GyroscopeIntrinsics>::kNumParameters;
  auto manifold = std::make_unique<Manifold<Cartesian<Scalar, kNumParameters>, OptimizerSuite::CERES>>(constant);
  setGyroscopeIntrinsicsManifold(std::move(manifold));
}

auto Manifold<IMU, OptimizerSuite::CERES>::accelerometerIntrinsicsManifold() const -> ceres::Manifold* {
  return manifold(Traits<IMU>::kAccelerometerIntrinsicsOffset);
}

auto Manifold<IMU, OptimizerSuite::CERES>::setAccelerometerIntrinsicsManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void {
  DCHECK_EQ(manifold->AmbientSize(), Traits<Traits<IMU>::AccelerometerIntrinsics>::kNumParameters);
  setManifold(Traits<IMU>::kAccelerometerIntrinsicsOffset, std::move(manifold));
}

auto Manifold<IMU, OptimizerSuite::CERES>::setAccelerometerIntrinsicsConstant(const bool constant) -> void {
  constexpr auto kNumParameters = Traits<Traits<IMU>::AccelerometerIntrinsics>::kNumParameters;
  auto manifold = std::make_unique<Manifold<Cartesian<Scalar, kNumParameters>, OptimizerSuite::CERES>>(constant);
  setAccelerometerIntrinsicsManifold(std::move(manifold));
}

auto Manifold<IMU, OptimizerSuite::CERES>::gyroscopeBiasManifold() const -> ceres::Manifold* {
  return manifold(Traits<Manifold>::kGyroscopeBiasManifoldOffset);
}

auto Manifold<IMU, OptimizerSuite::CERES>::setGyroscopeBiasManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void {
  DCHECK_EQ(manifold->AmbientSize(), Traits<Traits<IMU>::GyroscopeBias>::kNumParameters);
  setManifold(Traits<Manifold>::kGyroscopeBiasManifoldOffset, std::move(manifold));
}

auto Manifold<IMU, OptimizerSuite::CERES>::setGyroscopeBiasConstant(const bool constant) -> void {
  auto manifold = std::make_unique<Manifold<Traits<IMU>::GyroscopeBias, OptimizerSuite::CERES>>(true, constant);
  setGyroscopeBiasManifold(std::move(manifold));
}

auto Manifold<IMU, OptimizerSuite::CERES>::accelerometerBiasManifold() const -> ceres::Manifold* {
  return manifold(Traits<Manifold>::kAccelerometerBiasManifoldOffset);
}

auto Manifold<IMU, OptimizerSuite::CERES>::setAccelerometerBiasManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void {
  DCHECK_EQ(manifold->AmbientSize(), Traits<Traits<IMU>::AccelerometerBias>::kNumParameters);
  setManifold(Traits<Manifold>::kAccelerometerBiasManifoldOffset, std::move(manifold));
}

auto Manifold<IMU, OptimizerSuite::CERES>::setAccelerometerBiasConstant(const bool constant) -> void {
  auto manifold = std::make_unique<Manifold<Traits<IMU>::AccelerometerBias, OptimizerSuite::CERES>>(true, constant);
  setAccelerometerBiasManifold(std::move(manifold));
}

auto Manifold<IMU, OptimizerSuite::CERES>::manifolds() const -> Pointers<ceres::Manifold> {
  const auto num_gyroscope_bias_manifolds = sensor().gyroscopeBias().elements().size();
  const auto num_accelerometer_bias_manifolds = sensor().accelerometerBias().elements().size();
  return assembleManifolds(num_gyroscope_bias_manifolds, num_accelerometer_bias_manifolds);
}

auto Manifold<IMU, OptimizerSuite::CERES>::manifolds(const hyper::Stamp& /* stamp */) const -> Pointers<ceres::Manifold> {
  const auto num_gyroscope_bias_manifolds = sensor().gyroscopeBias().interpolator()->layout().outer.size;
  const auto num_accelerometer_bias_manifolds = sensor().accelerometerBias().interpolator()->layout().outer.size;
  return assembleManifolds(num_gyroscope_bias_manifolds, num_accelerometer_bias_manifolds);
}

auto Manifold<IMU, OptimizerSuite::CERES>::assembleManifolds(const Size& num_gyroscope_bias_manifolds, const Size& num_accelerometer_bias_manifolds) const -> Pointers<ceres::Manifold> {
  Pointers<ceres::Manifold> pointers;
  pointers.reserve(Traits<IMU>::kNumParameters + num_gyroscope_bias_manifolds + num_accelerometer_bias_manifolds);
  const auto start = manifolds_.begin();
  const auto end = std::next(start, Traits<IMU>::kNumParameters);
  std::transform(start, end, std::back_inserter(pointers), [](const auto& arg) -> ceres::Manifold* { return arg.get(); });
  std::fill_n(std::back_inserter(pointers), num_gyroscope_bias_manifolds, gyroscopeBiasManifold());
  std::fill_n(std::back_inserter(pointers), num_accelerometer_bias_manifolds, accelerometerBiasManifold());
  return pointers;
}

} // namespace hyper
