/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <ceres/product_manifold.h>

#include "hyper/optimizers/ceres/manifolds/variables/euclidean.hpp"

namespace hyper {

template <typename TVariable>
class Manifold<Stamped<TVariable>, OptimizerSuite::CERES> final
    : public ManifoldWrapper {
 public:
  /// Constructor from constancy flags.
  /// \tparam TArgs_ Variadic argument types.
  /// \param time_constant Time constancy flag.
  /// \param args Variadic arguments (i.e. further constancy flags).
  template <typename... TArgs_>
  explicit Manifold(const bool time_constant = true, TArgs_&&... args)
      : ManifoldWrapper{CreateManifold(time_constant, std::forward<TArgs_>(args)...)} {}

 private:
  /// Creates a (partially) constant or non-constant manifold.
  /// \tparam TArgs_ Variadic argument types.
  /// \param time_constant Time constancy flag.
  /// \param args Variadic arguments (i.e. further constancy flags).
  /// \return Manifold
  template <typename... TArgs_>
  static auto CreateManifold(const bool time_constant, TArgs_&&... args) -> std::unique_ptr<ceres::Manifold> {
    using Stamp = typename Traits<Stamped<TVariable>>::Stamp;
    using StampManifold = Manifold<Stamp, OptimizerSuite::CERES>;
    using VariableManifold = Manifold<TVariable, OptimizerSuite::CERES>;
    using ProductManifold = ceres::ProductManifold<VariableManifold, StampManifold>;
    return std::make_unique<ProductManifold>(VariableManifold{std::forward<TArgs_>(args)...}, StampManifold{time_constant});
  }
};

} // namespace hyper
