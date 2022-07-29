/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <ceres/dynamic_cost_function.h>

#include "hyper/environment/observations/forward.hpp"
#include "hyper/optimizers/ceres/costs/forward.hpp"
#include "hyper/optimizers/evaluators/forward.hpp"
#include "hyper/state/forward.hpp"
#include "hyper/variables/metrics/forward.hpp"

#include "hyper/optimizers/evaluators/query.hpp"
#include "hyper/variables/memory.hpp"

namespace hyper {

template <>
class ExteroceptiveCost<OptimizerSuite::CERES>
    : public ceres::DynamicCostFunction {
 public:
  // Definitions.
  using Scalar = double; // Ceres only supports double type.
  using Size = std::int32_t;

  /// Constructor from parameters.
  /// \param state Input state.
  /// \param observation Input observation.
  /// \param evaluator Input evaluator.
  /// \param metric Input metric.
  ExteroceptiveCost(
      AbstractState& state,
      AbstractObservation& observation,
      const AbstractEvaluator& evaluator,
      const AbstractMetric<Scalar>& metric);

  /// State accessor.
  /// \return State.
  [[nodiscard]] auto state() const -> AbstractState&;

  /// Observation accessor.
  /// \return Observation.
  [[nodiscard]] auto observation() const -> AbstractObservation&;

  /// Evaluator accessor.
  /// \return Evaluator.
  [[nodiscard]] auto evaluator() const -> const AbstractEvaluator&;

  /// Retrieves the metric.
  /// \return Metric.
  [[nodiscard]] auto metric() const -> const AbstractMetric<Scalar>&;

  /// Updates the parameters for this cost.
  /// \return Updated parameters.
  virtual auto updateParameters() -> Pointers<Scalar>;

  /// Evaluates the cost associated with this observation.
  /// \param parameters Input parameters.
  /// \param residuals Output residuals.
  /// \param jacobians Output Jacobians.
  /// \return True on success.
  auto Evaluate(const Scalar* const* parameters, Scalar* residuals, Scalar** jacobians) const -> bool override;

 protected:
  // Definitions.
  using Layout = EvaluatorQuery<Scalar, Size>::Layout;
  using Sizes = std::vector<Size>;
  using Offsets = std::vector<Size>;

  /// Updates the zero-based cumulative offsets and number of parameters.
  /// \param sizes Parameter block sizes.
  auto updateCost(const Sizes& sizes) -> void;

  Layout layout_;       ///< Layout.
  Offsets offsets_;     ///< Offsets.
  Size num_parameters_; ///< Number of parameters.

 private:
  AbstractState* state_;                 ///< State.
  AbstractObservation* observation_;     ///< Observation.
  const AbstractEvaluator* evaluator_;   ///< Evaluator.
  const AbstractMetric<Scalar>* metric_; ///< Metric.
};

} // namespace hyper
