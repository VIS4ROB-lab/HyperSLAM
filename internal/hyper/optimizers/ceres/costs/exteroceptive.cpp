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

ExteroceptiveCost<OptimizerSuite::CERES>::ExteroceptiveCost(
    AbstractState& state,
    AbstractObservation& observation,
    const AbstractEvaluator& evaluator,
    const AbstractMetric<Scalar>& metric)
    : layout_{},
      offsets_{},
      num_parameters_{},
      state_{&state},
      observation_{&observation},
      evaluator_{&evaluator},
      metric_{&metric} {}

auto ExteroceptiveCost<OptimizerSuite::CERES>::state() const -> AbstractState& {
  DCHECK(state_ != nullptr);
  return *state_;
}

auto ExteroceptiveCost<OptimizerSuite::CERES>::observation() const -> AbstractObservation& {
  DCHECK(observation_ != nullptr);
  return *observation_;
}

auto ExteroceptiveCost<OptimizerSuite::CERES>::evaluator() const -> const AbstractEvaluator& {
  DCHECK(evaluator_ != nullptr);
  return *evaluator_;
}

auto ExteroceptiveCost<OptimizerSuite::CERES>::metric() const -> const AbstractMetric<Scalar>& {
  DCHECK(metric_ != nullptr);
  return *metric_;
}

auto ExteroceptiveCost<OptimizerSuite::CERES>::updateParameters() -> Pointers<Scalar> {
  // Fetch parameters.
  auto& state = this->state();
  auto& observation = this->observation();
  auto& measurement = observation.measurement();

  // Fetch stamp and sensor.
  const auto& stamp = measurement.stamp();
  const auto& sensor = measurement.sensor();

  // Collect parameters.
  const auto state_parameters = state.parameters(stamp);
  const auto sensor_parameters = sensor.parameters(stamp);
  const auto observation_parameters = observation.parameters();

  // Evaluate number of parameters.
  const auto num_state_parameters = state_parameters.size();
  const auto num_sensor_parameters = sensor_parameters.size();
  const auto num_observation_parameters = observation_parameters.size();

  // Update layout.
  const auto state_idx = Size{0};
  const auto sensor_static_idx = state_idx + static_cast<Size>(num_state_parameters);
  const auto sensor_dynamic_idx = sensor_static_idx + static_cast<Size>(sensor.variables().size());
  const auto observation_idx = sensor_static_idx + static_cast<Size>(num_sensor_parameters);
  layout_ = {state_idx, sensor_static_idx, sensor_dynamic_idx, observation_idx};

  // Collect parameter information.
  Pointers<Scalar> pointers;
  Sizes sizes;

  const auto num_parameters = num_state_parameters + num_sensor_parameters + num_observation_parameters;
  pointers.reserve(num_parameters);
  sizes.reserve(num_parameters);

  for (const auto& parameter : state_parameters) {
    auto [address, size] = parameter->memory();
    pointers.emplace_back(address);
    sizes.emplace_back(size);
  }

  for (const auto& parameter : sensor_parameters) {
    auto [address, size] = parameter->memory();
    pointers.emplace_back(address);
    sizes.emplace_back(size);
  }

  for (const auto& parameter : observation_parameters) {
    auto [address, size] = parameter->memory();
    pointers.emplace_back(address);
    sizes.emplace_back(size);
  }

  // Update cost.
  updateCost(sizes);
  return pointers;
}

auto ExteroceptiveCost<OptimizerSuite::CERES>::Evaluate(const Scalar* const* parameters, Scalar* residuals, Scalar** jacobians) const -> bool {
  // Fetch references.
  const auto& sizes = this->parameter_block_sizes();
  const auto& state = this->state();
  const auto& measurement = this->observation().measurement();
  const auto& metric = this->metric();

  // Create query.
  using Query = EvaluatorQuery<Scalar, Size>;
  const auto pointers = Query::Pointers{parameters, jacobians};
  const auto query = Query{pointers, layout_, offsets_, sizes, state, measurement};

  // Retrieve measurement address.
  const auto p_measurement = measurement.memoryBlock().address;

  if (!jacobians) {
    // Evaluate residuals.
    const auto prediction = evaluator().evaluate(query, nullptr);
    metric.distance(residuals, prediction.data(), p_measurement, nullptr, nullptr);

  } else {
    // Allocate metric and evaluator Jacobians.
    const auto shape = metric.jacobianShape();
    auto J_m = DynamicJacobian<Scalar>{shape.num_outputs, shape.num_inputs};
    auto J_e = DynamicJacobian<Scalar>{J_m.cols(), num_parameters_};
    J_e.setZero();

    // Evaluate residuals.
    const auto prediction = evaluator().evaluate(query, &J_e);
    metric.distance(residuals, prediction.data(), p_measurement, J_m.data(), nullptr);
    const auto J = (J_m * J_e).eval();

    // Assign Jacobians.
    auto i = 0;
    const auto num_residuals = this->num_residuals();
    for (const auto& size : sizes) {
      if (jacobians[i]) {
        Eigen::Map<DynamicJacobian<Scalar, Eigen::RowMajor>>{jacobians[i], num_residuals, size}.noalias() = J.middleCols(offsets_[i], size);
      }
      ++i;
    }
  }

  return true;
}

auto ExteroceptiveCost<OptimizerSuite::CERES>::updateCost(const Sizes& sizes) -> void {
  // Retrieves parameter block sizes.
  DCHECK(!sizes.empty());
  *mutable_parameter_block_sizes() = sizes;

  // Fetch iterators.
  const auto begin = sizes.cbegin();
  const auto end = sizes.cend();

  // Compute offsets and number of parameters.
  num_parameters_ = std::accumulate(begin, end, Size{0});
  offsets_.resize(sizes.size());
  std::partial_sum(begin, end - 1, offsets_.begin() + 1);

  // Update number of residuals.
  SetNumResiduals(metric().shape().num_outputs);
}

} // namespace hyper
