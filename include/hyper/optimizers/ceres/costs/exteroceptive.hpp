/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <ceres/dynamic_cost_function.h>

#include "hyper/optimizers/ceres/costs/forward.hpp"
#include "hyper/optimizers/evaluators/forward.hpp"

namespace hyper {

template <>
class ExteroceptiveCost<OptimizerSuite::CERES>
    : public ceres::DynamicCostFunction {
 public:
  /// Constructor from context and options.
  /// \param configuration Input configuration.
  /// \param context Input context.
  ExteroceptiveCost(const CostConfiguration<double>& configuration, const CostContext& context);

  /// Updates this cost.
  /// \return Pointers to parameters.
  virtual auto update() -> Pointers<double>;

  /// Evaluates this.
  /// \param parameters Input parameters.
  /// \param residuals Output residuals.
  /// \param jacobians Output Jacobians.
  /// \return True on success.
  auto Evaluate(const double* const* parameters, double* residuals, double** jacobians) const -> bool override;

 private:
  /// Checks the configuration.
  /// \return True if passed.
  [[nodiscard]] auto checkConfiguration() const -> bool;

  CostConfiguration<double> configuration_; ///< Configuration.
  CostContext context_;                     ///< Context.
  EvaluatorLayout<Index> layout_;           ///< Layout.
};

} // namespace hyper
