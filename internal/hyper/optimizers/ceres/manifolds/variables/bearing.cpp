/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <ceres/sphere_manifold.h>

#include "hyper/optimizers/ceres/manifolds/variables/bearing.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/euclidean.hpp"

namespace hyper {

auto Manifold<Bearing<double>, OptimizerSuite::CERES>::CreateManifold(const bool constant) -> std::unique_ptr<ceres::Manifold> {
  if (constant) {
    return std::make_unique<Manifold<Cartesian<Scalar, Traits<Bearing<Scalar>>::kNumParameters>, OptimizerSuite::CERES>>(true);
  } else {
    return std::make_unique<ceres::SphereManifold<Traits<Bearing<Scalar>>::kNumParameters>>();
  }
}

} // namespace hyper
