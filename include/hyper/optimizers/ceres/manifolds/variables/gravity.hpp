/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/optimizers/ceres/manifolds/variables/bearing.hpp"

namespace hyper {

template <>
class Manifold<Gravity<double>, OptimizerSuite::CERES> final
    : public Manifold<Bearing<double>, OptimizerSuite::CERES> {
 public:
  /// Constructor from constancy flag.
  /// \param constant Constancy flag.
  explicit Manifold(const bool constant = false)
      : Manifold<Bearing<Scalar>, OptimizerSuite::CERES>{constant} {}
};

} // namespace hyper
