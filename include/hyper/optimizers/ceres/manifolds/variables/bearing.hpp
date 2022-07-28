/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/variables/forward.hpp"

#include "hyper/optimizers/ceres/manifolds/variables/wrapper.hpp"

namespace hyper {

template <>
class Manifold<Bearing<double>, OptimizerSuite::CERES>
    : public ManifoldWrapper {
 public:
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
