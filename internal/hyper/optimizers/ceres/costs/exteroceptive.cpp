/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <numeric>

#include <glog/logging.h>

#include "hyper/environment/observations/abstract.hpp"
#include "hyper/messages/measurements/abstract.hpp"
#include "hyper/optimizers/ceres/costs/exteroceptive.hpp"
#include "hyper/optimizers/evaluators/abstract.hpp"
#include "hyper/sensors/sensor.hpp"
#include "hyper/state/abstract.hpp"
#include "hyper/variables/metrics/abstract.hpp"

namespace hyper {

ExteroceptiveCost<OptimizerSuite::CERES>::ExteroceptiveCost(const CostConfiguration<double>& configuration, const CostContext& context)
    : configuration_{configuration},
      context_{context},
      layout_{} {
  layout_.sizes = mutable_parameter_block_sizes();
}

auto ExteroceptiveCost<OptimizerSuite::CERES>::update() -> Pointers<double> {
  // Sanity check.
  DCHECK(checkConfiguration());

  // Fetch stamp and sensor.
  const auto& measurement = context_.observation->measurement();
  const auto& stamp = measurement.stamp();
  const auto& sensor = measurement.sensor();

  // Collect variables.
  const auto state_variables = context_.state->parameters(stamp);
  const auto sensor_variables = sensor.parameters(stamp);
  const auto observation_variables = context_.observation->variables(stamp);

  // Evaluate number of variables.
  const auto num_state_variables = state_variables.size();
  const auto num_sensor_variables = sensor_variables.size();
  const auto num_observation_variables = observation_variables.size();
  const auto num_variables = num_state_variables + num_sensor_variables + num_observation_variables;

  // Update layout.
  const auto state_idx = Index{0};
  const auto sensor_static_idx = state_idx + static_cast<Index>(num_state_variables);
  const auto sensor_dynamic_idx = sensor_static_idx + static_cast<Index>(sensor.variables().size());
  const auto observation_idx = sensor_static_idx + static_cast<Index>(num_sensor_variables);
  layout_.indices = {state_idx, sensor_static_idx, sensor_dynamic_idx, observation_idx};

  // Allocate parameter block pointers.
  Pointers<double> pointers;
  pointers.reserve(num_variables);

  // Allocate parameter block sizes.
  auto& parameter_block_sizes = *mutable_parameter_block_sizes();
  parameter_block_sizes.clear();
  parameter_block_sizes.reserve(num_variables);

  for (const auto& variable : state_variables) {
    auto vector = variable->asVector();
    pointers.emplace_back(vector.data());
    parameter_block_sizes.push_back(vector.size());
  }

  for (const auto& variable : sensor_variables) {
    auto vector = variable->asVector();
    pointers.emplace_back(vector.data());
    parameter_block_sizes.push_back(vector.size());
  }

  for (const auto& variable : observation_variables) {
    auto vector = variable->asVector();
    pointers.emplace_back(vector.data());
    parameter_block_sizes.push_back(vector.size());
  }

  // Compute number of parameters.
  const auto begin = parameter_block_sizes.cbegin();
  const auto end = parameter_block_sizes.cend();
  layout_.num_parameters = std::accumulate(begin, end, Index{0});

  // Compute cumulative offsets.
  if (!parameter_block_sizes.empty()) {
    layout_.offsets.resize(parameter_block_sizes.size());
    std::partial_sum(begin, end - 1, layout_.offsets.begin() + 1);
  }

  // Update number of residuals.
  if (configuration_.metric) {
    SetNumResiduals(configuration_.metric->outputSize());
  } else {
    SetNumResiduals(context_.observation->measurement().variable().asVector().size());
  }

  // Return pointers.
  return pointers;
}

auto ExteroceptiveCost<OptimizerSuite::CERES>::Evaluate(const double* const* parameters, double* residuals, double** jacobians) const -> bool {
  auto output = Eigen::Map<DynamicVector<double>>{residuals, num_residuals()};

  if (!jacobians) {
    const auto pointers = EvaluatorPointers<double>{parameters, nullptr, jacobians};
    const auto query = EvaluatorQuery<double, Index>{pointers, layout_, context_};
    const auto prediction = configuration_.evaluator->evaluate(query);

    if (configuration_.metric) {
      if (configuration_.weights) {
        output = *configuration_.weights * configuration_.metric->distance(prediction, context_.observation->measurement().variable().asVector(), nullptr, nullptr);
      } else {
        output = configuration_.metric->distance(prediction, context_.observation->measurement().variable().asVector(), nullptr, nullptr);
      }
    } else {
      if (configuration_.weights) {
        output = *configuration_.weights * (prediction - context_.observation->measurement().variable().asVector());
      } else {
        output = prediction - context_.observation->measurement().variable().asVector();
      }
    }
  } else {
    DynamicJacobian<double> J_e;
    const auto pointers = EvaluatorPointers<double>{parameters, &J_e, jacobians};
    const auto query = EvaluatorQuery<double, Index>{pointers, layout_, context_};
    const auto prediction = configuration_.evaluator->evaluate(query);

    DynamicJacobian<double> J_w;
    if (configuration_.metric) {
      if (configuration_.weights) {
        DynamicJacobian<double> J_m;
        output = *configuration_.weights * configuration_.metric->distance(prediction, context_.observation->measurement().variable().asVector(), &J_m, nullptr);
        J_w = *configuration_.weights * J_m * J_e;
      } else {
        DynamicJacobian<double> J_m;
        output = configuration_.metric->distance(prediction, context_.observation->measurement().variable().asVector(), &J_m, nullptr);
        J_w = J_m * J_e;
      }
    } else {
      if (configuration_.weights) {
        output = *configuration_.weights * (prediction - context_.observation->measurement().variable().asVector());
        J_w = *configuration_.weights * J_e;
      } else {
        output = prediction - context_.observation->measurement().variable().asVector();
        J_w = J_e;
      }
    }

    // Assign parameter block Jacobians.
    auto num_parameter_blocks = parameter_block_sizes().size();
    for (decltype(num_parameter_blocks) i = 0; i < num_parameter_blocks; ++i) {
      if (jacobians[i]) {
        const auto size = parameter_block_sizes()[i];
        Eigen::Map<DynamicJacobian<double, Eigen::RowMajor>>{jacobians[i], num_residuals(), size}.noalias() = J_w.middleCols(layout_.offsets[i], size);
      }
    }
  }

  return true;
}

auto ExteroceptiveCost<OptimizerSuite::CERES>::checkConfiguration() const -> bool {
  CHECK(configuration_.evaluator != nullptr);
  CHECK(context_.state != nullptr);
  CHECK(context_.observation != nullptr);
  if (configuration_.weights || configuration_.metric) {
    if (configuration_.weights && configuration_.metric) {
      CHECK_EQ(configuration_.weights->rows(), configuration_.weights->cols());
      CHECK_EQ(configuration_.weights->cols(), configuration_.metric->outputSize());
    } else if (configuration_.weights) {
      CHECK_EQ(configuration_.weights->rows(), configuration_.weights->cols());
      CHECK_EQ(configuration_.weights->cols(), context_.observation->measurement().variable().asVector().size());
    } else {
      CHECK_EQ(configuration_.metric->inputSize(), context_.observation->measurement().variable().asVector().size());
    }
  }
  return true;
}

} // namespace hyper
