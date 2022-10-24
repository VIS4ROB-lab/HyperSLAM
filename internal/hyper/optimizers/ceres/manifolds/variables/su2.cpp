/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/optimizers/ceres/manifolds/variables/su2.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/euclidean.hpp"
#include "hyper/variables/adapters.hpp"
#include "hyper/variables/groups/su2.hpp"

namespace hyper {

using SU2Manifold = Manifold<SU2<double>, OptimizerSuite::CERES>;

auto SU2Manifold::AdapterJacobian(const Scalar* raw_su2) -> Jacobian<Tangent<SU2<Scalar>>, SU2<Scalar>> {
  return SU2JacobianAdapter(raw_su2);
}

auto SU2Manifold::CreateManifold(const bool constant) -> std::unique_ptr<ceres::Manifold> {
  if (constant) {
    return std::make_unique<Manifold<Cartesian<Scalar, Traits<SU2<Scalar>>::kNumParameters>, OptimizerSuite::CERES>>(true);
  } else {
    return std::make_unique<ceres::EigenQuaternionManifold>();
  }
}

} // namespace hyper
