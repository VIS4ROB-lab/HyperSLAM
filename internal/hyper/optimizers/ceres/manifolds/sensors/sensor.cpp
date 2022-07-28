/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/optimizers/ceres/manifolds/sensors/sensor.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/se3.hpp"

namespace hyper {

Manifold<Sensor, OptimizerSuite::CERES>::Manifold(const Sensor& sensor, const bool constant)
    : Manifold{sensor, Traits<Manifold>::kNumManifolds, constant} {}

auto Manifold<Sensor, OptimizerSuite::CERES>::sensor() const -> const Sensor& {
  DCHECK(sensor_ != nullptr);
  return *sensor_;
}

auto Manifold<Sensor, OptimizerSuite::CERES>::transformationManifold() const -> ceres::Manifold* {
  return manifold(Traits<Sensor>::kTransformationOffset);
}

auto Manifold<Sensor, OptimizerSuite::CERES>::setTransformationManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void {
  DCHECK_EQ(manifold->AmbientSize(), Traits<Traits<Sensor>::Transformation>::kNumParameters);
  setManifold(Traits<Sensor>::kTransformationOffset, std::move(manifold));
}

auto Manifold<Sensor, OptimizerSuite::CERES>::setTransformationConstant(bool constant) -> void {
  auto manifold = std::make_unique<Manifold<Traits<Sensor>::Transformation, OptimizerSuite::CERES>>(constant, constant);
  setTransformationManifold(std::move(manifold));
}

auto Manifold<Sensor, OptimizerSuite::CERES>::manifolds() const -> Pointers<ceres::Manifold> {
  Pointers<ceres::Manifold> pointers;
  pointers.reserve(manifolds_.size());
  std::transform(manifolds_.begin(), manifolds_.end(), std::back_inserter(pointers), [](const auto& arg) -> ceres::Manifold* { return arg.get(); });
  return pointers;
}

auto Manifold<Sensor, OptimizerSuite::CERES>::manifolds(const Stamp& /* stamp */) const -> Pointers<ceres::Manifold> {
  return manifolds();
}

Manifold<Sensor, OptimizerSuite::CERES>::Manifold(const Sensor& sensor, const Size& num_manifolds, const bool constant)
    : sensor_{&sensor},
      manifolds_{num_manifolds} {
  setTransformationConstant(constant);
}

auto Manifold<Sensor, OptimizerSuite::CERES>::manifold(const Size& index) const -> ceres::Manifold* {
  DCHECK_LT(index, manifolds_.size());
  return manifolds_[index].get();
}

auto Manifold<Sensor, OptimizerSuite::CERES>::setManifold(const Size& index, std::unique_ptr<ceres::Manifold>&& manifold) -> void {
  DCHECK_LT(index, manifolds_.size());
  manifolds_[index] = std::move(manifold);
}

} // namespace hyper
