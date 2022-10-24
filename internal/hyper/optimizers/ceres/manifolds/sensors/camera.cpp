/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/optimizers/ceres/manifolds/sensors/camera.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/euclidean.hpp"
#include "hyper/variables/intrinsics.hpp"

namespace hyper {

Manifold<Camera, OptimizerSuite::CERES>::Manifold(const Camera& camera, const bool constant)
    : Manifold<Sensor, OptimizerSuite::CERES>{camera, Traits<Manifold>::kNumManifolds, constant} {
  setIntrinsicsConstant(constant);
  setDistortionConstant(constant);
}

auto Manifold<Camera, OptimizerSuite::CERES>::sensor() const -> const Camera& {
  DCHECK(sensor_ != nullptr);
  return static_cast<const Camera&>(*sensor_); // NOLINT
}

auto Manifold<Camera, OptimizerSuite::CERES>::intrinsicsManifold() const -> ceres::Manifold* {
  return manifold(Traits<Camera>::kIntrinsicsOffset);
}

auto Manifold<Camera, OptimizerSuite::CERES>::setIntrinsicsManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void {
  DCHECK_EQ(manifold->AmbientSize(), Traits<Intrinsics<Scalar>>::kNumParameters);
  setManifold(Traits<Camera>::kIntrinsicsOffset, std::move(manifold));
}

auto Manifold<Camera, OptimizerSuite::CERES>::setIntrinsicsConstant(const bool constant) -> void {
  auto manifold = std::make_unique<Manifold<Cartesian<Scalar, Traits<Intrinsics<Scalar>>::kNumParameters>, OptimizerSuite::CERES>>(constant);
  setIntrinsicsManifold(std::move(manifold));
}

auto Manifold<Camera, OptimizerSuite::CERES>::distortionManifold() const -> ceres::Manifold* {
  return manifold(Traits<Camera>::kDistortionOffset);
}

auto Manifold<Camera, OptimizerSuite::CERES>::setDistortionManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void {
  DCHECK_EQ(manifold->AmbientSize(), sensor().distortion().asVector().size());
  setManifold(Traits<Camera>::kDistortionOffset, std::move(manifold));
}

auto Manifold<Camera, OptimizerSuite::CERES>::setDistortionConstant(const bool constant) -> void {
  const auto num_parameters = sensor().distortion().asVector().size();
  auto manifold = std::make_unique<Manifold<Cartesian<Scalar, ceres::DYNAMIC>, OptimizerSuite::CERES>>(num_parameters, constant);
  setDistortionManifold(std::move(manifold));
}

} // namespace hyper
