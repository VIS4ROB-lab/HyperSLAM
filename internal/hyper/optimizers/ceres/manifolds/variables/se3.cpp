/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <ceres/product_manifold.h>

#include "hyper/optimizers/ceres/manifolds/variables/euclidean.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/se3.hpp"
#include "hyper/variables/adapters.hpp"
#include "hyper/variables/groups/se3.hpp"

namespace hyper {

using SE3Manifold = Manifold<SE3<double>, OptimizerSuite::CERES>;

auto SE3Manifold::AdapterJacobian(const Scalar* raw_se3) -> Jacobian<Tangent<SE3<Scalar>>, SE3<Scalar>> {
  return SE3JacobianAdapter(raw_se3);
}

auto SE3Manifold::CreateManifold(const bool rotation_constant, const bool translation_constant) -> std::unique_ptr<ceres::Manifold> {
  using RotationManifold = Manifold<SE3<Scalar>::Rotation, OptimizerSuite::CERES>;
  using TranslationManifold = Manifold<SE3<Scalar>::Translation, OptimizerSuite::CERES>;
  using ProductManifold = ceres::ProductManifold<RotationManifold, TranslationManifold>;
  return std::make_unique<ProductManifold>(RotationManifold{rotation_constant}, TranslationManifold{translation_constant});
}

} // namespace hyper
