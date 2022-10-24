/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <ceres/gradient_checker.h>
#include <ceres/numeric_diff_options.h>
#include <gtest/gtest.h>

#include "hyper/composite.hpp"

#include "tests/optimizers/evaluators/forward.hpp"
#include "tests/state/abstract.hpp"

namespace hyper::tests {

template <>
class EvaluatorTests<OptimizerSuite::CERES>
    : public testing::Test {
 public:
  // Definitions.
  static constexpr auto kDefaultNumericTolerance = 1e-5;

  /// Creates numeric differentiation options.
  /// \return Numeric differentiation options.
  static auto NumericDifferentiationOptions() -> ceres::NumericDiffOptions {
    ceres::NumericDiffOptions options;
    options.relative_step_size = 1e-6;
    options.ridders_relative_initial_step_size = 1e-6;
    options.max_num_ridders_extrapolations = 1;
    return options;
  }

  /// Probes the gradients with relative and absolute checks.
  /// \param cost Input cost.
  /// \param manifolds Input manifolds.
  /// \return True if either the relative or absolute check passes.
  static auto Probe(ExteroceptiveCost<OptimizerSuite::CERES>& cost, const Pointers<const ceres::Manifold>& manifolds, const Scalar& tolerance = kDefaultNumericTolerance) -> bool {
    // Collect parameters.
    const auto parameters = cost.update();
    CHECK_EQ(parameters.size(), manifolds.size());

    // Create gradient checker.
    auto gradient_checker = ceres::GradientChecker{&cost, &manifolds, NumericDifferentiationOptions()};

    // Relative check.
    ceres::GradientChecker::ProbeResults results;
    const auto relative_check = gradient_checker.Probe(parameters.data(), tolerance, &results);

    // Absolute check.
    bool absolute_check = true;
    for (const auto& [J_l_a, J_l_n] : makeComposite(results.local_jacobians, results.local_numeric_jacobians)) {
      const auto n_J_l_a = J_l_a.normalized().eval();
      const auto n_J_l_n = J_l_n.normalized().eval();
      if (!(n_J_l_n - n_J_l_a).isZero(tolerance)) {
        absolute_check = false;
        break;
      }
    }

    // Log error if both checks fail.
    const auto passed = relative_check || absolute_check;
    LOG_IF(INFO, !passed) << results.error_log;
    return passed;
  }
};

} // namespace hyper::tests
