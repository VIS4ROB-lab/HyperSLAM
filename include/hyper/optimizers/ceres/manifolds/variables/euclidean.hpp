/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <numeric>

#include "hyper/variables/forward.hpp"

#include "hyper/optimizers/ceres/manifolds/variables/wrapper.hpp"

namespace hyper {

template <int TNumParameters>
class Manifold<Cartesian<double, TNumParameters>, OptimizerSuite::CERES> final
    : public ManifoldWrapper {
 public:
  /// Constructor from constancy flag.
  /// \param constant Constancy flag.
  explicit Manifold(const bool constant = false)
      : Manifold(TNumParameters, constant) {}

  /// Constructor from number of parameters and constancy flag.
  /// \param num_parameters Number of parameters.
  /// \param constant Constancy flag.
  Manifold(const int num_parameters, const bool constant)
      : ManifoldWrapper{CreateManifold(num_parameters, constant)} {}

 private:
  /// Creates a constant or non-constant manifold.
  /// \param num_parameters Number of parameters.
  /// \param constant Constancy flag.
  /// \return Manifold.
  static auto CreateManifold(const int num_parameters, const bool constant) -> std::unique_ptr<ceres::Manifold> {
    if (constant) {
      return std::make_unique<ceres::SubsetManifold>(num_parameters, CreateConstancyMask(num_parameters));
    } else {
      return std::make_unique<ceres::EuclideanManifold<TNumParameters>>(num_parameters);
    }
  }

  /// Creates a constancy mask (i.e. all parameters are held constant).
  /// \param num_parameters Number of parameters.
  /// \return Constancy mask.
  static auto CreateConstancyMask(const int num_parameters) -> std::vector<int> {
    std::vector<int> mask;
    mask.resize(num_parameters);
    std::iota(mask.begin(), mask.end(), 0);
    return mask;
  }
};

} // namespace hyper
