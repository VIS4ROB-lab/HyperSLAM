/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/variables/groups/forward.hpp"

#include "hyper/optimizers/ceres/manifolds/variables/wrapper.hpp"
#include "hyper/variables/jacobian.hpp"

namespace hyper {

template <>
class Manifold<SU2<double>, OptimizerSuite::CERES> final
    : public ManifoldWrapper {
 public:
  /// Adapter Jacobian (i.e. adapter to Ceres Jacobians).
  /// \param raw_su2 Raw SU2 input.
  /// \return Adapter Jacobian.
  static auto AdapterJacobian(const Scalar* raw_su2) -> Jacobian<Tangent<SU2<Scalar>>, SU2<Scalar>>;

  /// Constructor from constancy flag.
  /// \param constant Constancy flag.
  explicit Manifold(const bool constant = false)
      : ManifoldWrapper{CreateManifold(constant)} {}

 private:
  /// Creates a constant or non-constant manifold.
  /// \param constant Constancy flag.
  /// \return Manifold.
  static auto CreateManifold(bool constant) -> std::unique_ptr<ceres::Manifold>;
};

} // namespace hyper
