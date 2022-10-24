/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <ceres/manifold.h>

#include "hyper/optimizers/forward.hpp"

namespace hyper {

/// @class Manifold wrapper for Ceres.
/// This wrapper is required due to some
/// manifolds being marked as final within
/// Ceres, rendering them non-inheritable.
/// We refer to the Ceres documentation
/// for all inherited and overloaded
/// functions in this class.
class ManifoldWrapper
    : public ceres::Manifold {
 public:
  using Scalar = double; // Ceres only supports double type.

  [[nodiscard]] auto AmbientSize() const -> int final {
    return manifold_->AmbientSize();
  }

  [[nodiscard]] auto TangentSize() const -> int final {
    return manifold_->TangentSize();
  }

  auto Plus(const Scalar* x_ptr, const Scalar* delta_ptr, Scalar* x_plus_delta_ptr) const -> bool final {
    return manifold_->Plus(x_ptr, delta_ptr, x_plus_delta_ptr);
  }

  auto PlusJacobian(const Scalar* x_ptr, Scalar* jacobian_ptr) const -> bool final {
    return manifold_->PlusJacobian(x_ptr, jacobian_ptr);
  }

  auto RightMultiplyByPlusJacobian(const Scalar* x, const int num_rows, const Scalar* ambient_matrix, Scalar* tangent_matrix) const -> bool final {
    return manifold_->RightMultiplyByPlusJacobian(x, num_rows, ambient_matrix, tangent_matrix);
  }

  auto Minus(const Scalar* y_ptr, const Scalar* x_ptr, Scalar* y_minus_x_ptr) const -> bool final {
    return manifold_->Minus(y_ptr, x_ptr, y_minus_x_ptr);
  }

  auto MinusJacobian(const Scalar* x_ptr, Scalar* jacobian_ptr) const -> bool final {
    return manifold_->MinusJacobian(x_ptr, jacobian_ptr);
  }

 protected:
  /// Protected constructor from input manifold.
  /// \param manifold Input manifold.
  explicit ManifoldWrapper(std::unique_ptr<ceres::Manifold>&& manifold)
      : manifold_{std::move(manifold)} {}

 private:
  std::unique_ptr<ceres::Manifold> manifold_; ///< Manifold.
};

} // namespace hyper
