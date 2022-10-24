/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/variables/groups/forward.hpp"

#include "hyper/optimizers/ceres/manifolds/variables/su2.hpp"

namespace hyper {

template <>
class Manifold<SE3<double>, OptimizerSuite::CERES> final
    : public ManifoldWrapper {
 public:
  /// Adapter Jacobian (i.e. adapter to Ceres Jacobians).
  /// \param raw_se3 Raw SE3 input.
  /// \return Adapter Jacobian.
  static auto AdapterJacobian(const Scalar* raw_se3) -> Jacobian<Tangent<SE3<Scalar>>, SE3<Scalar>>;

  /// Constructor from constancy flags.
  /// \param rotation_constant Rotation constancy flag.
  /// \param translation_constant Translation constancy flag.
  explicit Manifold(const bool rotation_constant = false, const bool translation_constant = false)
      : ManifoldWrapper{CreateManifold(rotation_constant, translation_constant)} {}

 private:
  /// Creates a (partially) constant or non-constant manifold.
  /// \param rotation_constant Rotation constancy flag.
  /// \param translation_constant Translation constancy flag.
  /// \return Manifold.
  static auto CreateManifold(bool rotation_constant, bool translation_constant) -> std::unique_ptr<ceres::Manifold>;
};

} // namespace hyper
