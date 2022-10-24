/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <typeindex>

#include "hyper/composite.hpp"
#include "hyper/environment/environment.hpp"
#include "hyper/environment/observations/inertial.hpp"
#include "hyper/environment/observations/manifold.hpp"
#include "hyper/environment/observations/visual.hpp"
#include "hyper/optimizers/ceres/costs/exteroceptive.hpp"
#include "hyper/optimizers/ceres/manifolds/sensors/camera.hpp"
#include "hyper/optimizers/ceres/manifolds/sensors/imu.hpp"
#include "hyper/optimizers/ceres/optimizer.hpp"
#include "hyper/optimizers/evaluators/evaluator.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/variables/bearing.hpp"
#include "hyper/variables/gravity.hpp"
#include "hyper/variables/metrics/angular.hpp"
#include "hyper/variables/metrics/cartesian.hpp"
#include "hyper/variables/metrics/manifold.hpp"
#include "hyper/yaml/yaml.hpp"

namespace hyper {

namespace {

const auto kDefaultProblemOptions = ceres::Problem::Options{
    .cost_function_ownership = ceres::TAKE_OWNERSHIP,
    .loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP,
    .manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP,
    .enable_fast_removal = true,
    .disable_all_safety_checks = false,
    .context = nullptr,
    .evaluation_callback = nullptr,
};

const auto kDefaultSolverOptions = ceres::Solver::Options{
    //.use_nonmonotonic_steps = true,
    .max_num_iterations = 5,
    .num_threads = 1,
    //.function_tolerance = 1e-3,
    //.gradient_tolerance = 1e-7,
    //.parameter_tolerance = 1e-2,
    //.max_solver_time_in_seconds = 0.02,
    .linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY,
    .residual_blocks_for_subset_preconditioner = {},
    .sparse_linear_algebra_library_type = ceres::SUITE_SPARSE,
    .linear_solver_ordering = {},
    //.use_explicit_schur_complement = true,
    .inner_iteration_ordering = {},
    .trust_region_minimizer_iterations_to_dump = {},
    .callbacks = {},
};

auto createSensorManifold(const YAML::Node& /* yaml_node */, Sensor& sensor) -> std::unique_ptr<Manifold<Sensor, OptimizerSuite::CERES>> {
  const auto sensor_type_index = std::type_index{typeid(sensor)};
  if (sensor_type_index == std::type_index{typeid(Camera)}) {
    return std::make_unique<Manifold<Camera, OptimizerSuite::CERES>>(sensor.as<Camera>());
  } else if (sensor_type_index == std::type_index{typeid(IMU)}) {
    auto imu_manifold = std::make_unique<Manifold<IMU, OptimizerSuite::CERES>>(sensor.as<IMU>());
    imu_manifold->setGyroscopeBiasConstant(false);
    imu_manifold->setAccelerometerBiasConstant(false);
    return imu_manifold;
  } else {
    LOG(FATAL) << "Requested instantiation of unknown sensor manifold.";
    return nullptr;
  }
}

} // namespace

CeresOptimizer::Optimizer(const YAML::Node& yaml_node, const std::vector<Sensor*>& sensors)
    : AbstractOptimizer{yaml_node},
      problem_{kDefaultProblemOptions},
      sensor_manifolds_{},
      gravity_manifold_{nullptr},
      state_manifold_{nullptr} {
  if (!yaml_node.IsNull()) {
    for (auto& sensor : sensors) {
      DCHECK(sensor != nullptr);
      setSensorManifold(createSensorManifold(yaml_node, *sensor));
    }
  }
}

auto CeresOptimizer::swapEnvironment(std::unique_ptr<Environment<Manifold>>& environment) -> void {
  // Remove environment.
  if (environment_) {
    const auto address = mutableEnvironment().gravity().data();
    DCHECK(problem_.HasParameterBlock(address));
    problem_.RemoveParameterBlock(address);
    DLOG(INFO) << "Removed gravity parameter block " << address << ".";
  }

  // Swap environment.
  DLOG(INFO) << "Swapping environment " << environment_.get() << " and " << environment.get() << ".";
  std::swap(environment_, environment);

  // Add environment.
  if (environment_) {
    const auto address = mutableEnvironment().gravity().data();
    DCHECK(!problem_.HasParameterBlock(address));
    problem_.AddParameterBlock(address, Traits<Gravity<Scalar>>::kNumParameters, gravity_manifold_.get());
    DLOG(INFO) << "Added gravity parameter block " << address << ".";
  }
}

auto CeresOptimizer::swapState(std::unique_ptr<AbstractState>& state) -> void {
  // Remove state.
  if (state_) {
    for (const auto& parameter : state_->elements()) {
      auto vector = parameter->asVector();
      auto data = vector.data();
      if (problem_.HasParameterBlock(data)) {
        problem_.RemoveParameterBlock(data);
        DLOG(INFO) << "Removed state parameter block " << data;
      }
    }
  }

  // Swap state.
  DLOG(INFO) << "Swapping state " << state_.get() << " and " << state.get() << ".";
  std::swap(state_, state);

  // Update state.
  updateState(window_);
}

auto CeresOptimizer::setGravityConstant(const bool set_constant) -> void {
  auto address = mutableEnvironment().gravity().data();
  DCHECK(problem_.HasParameterBlock(address));
  const auto is_constant = problem_.IsParameterBlockConstant(address);
  if (set_constant && !is_constant) {
    DLOG(INFO) << "Setting gravity parameter block " << address << " constant.";
    problem_.SetParameterBlockConstant(address);
  } else if (!set_constant && is_constant) {
    DLOG(INFO) << "Setting gravity parameter block " << address << " variable.";
    problem_.SetParameterBlockVariable(address);
  }
}

auto CeresOptimizer::setSensorManifold(std::unique_ptr<SensorManifold>&& sensor_manifold) -> void {
  const auto& sensor = sensor_manifold->sensor();
  const auto parameters = sensor.parameters();
  const auto manifolds = sensor_manifold->manifolds();
  DCHECK_EQ(parameters.size(), manifolds.size());

  for (const auto& [parameter, manifold] : makeComposite(parameters, manifolds)) {
    auto vector = parameter->asVector();
    problem_.AddParameterBlock(vector.data(), vector.size(), manifold);
  }

  sensor_manifolds_.insert(std::make_pair(&sensor, std::move(sensor_manifold)));
}

auto CeresOptimizer::hasSensor(const Sensor& sensor) const -> bool {
  return sensor_manifolds_.contains(&sensor);
}

auto CeresOptimizer::setGravityManifold(std::unique_ptr<GravityManifold>&& gravity_manifold) -> void {
  // Set manifold.
  DCHECK(gravity_manifold == nullptr || gravity_manifold->AmbientSize() == Traits<Gravity<Scalar>>::kNumParameters);
  gravity_manifold_ = std::move(gravity_manifold);

  // Update manifold.
  auto address = mutableEnvironment().gravity().data();
  DCHECK(problem_.HasParameterBlock(address));
  problem_.SetManifold(address, gravity_manifold_.get());
}

auto CeresOptimizer::setStateManifold(std::unique_ptr<StateManifold>&& state_manifold) -> void {
  // Set manifold.
  DCHECK(state_manifold == nullptr || state_manifold->AmbientSize() == Traits<StampedManifold>::kNumParameters);
  state_manifold_ = std::move(state_manifold);

  // Update manifold.
  if (state_) {
    for (const auto& element : state_->elements()) {
      auto vector = element->asVector();
      auto data = vector.data();
      if (problem_.HasParameterBlock(data)) {
        problem_.SetManifold(data, state_manifold_.get());
      }
    }
  }
}

auto CeresOptimizer::add(VisualBearingObservation& observation) -> void {
  // Create options.
  static auto weights = nullptr;
  static auto metric = AngularMetric<Bearing<Scalar>>{};
  static auto evaluator = VisualBearingEvaluator<Manifold>{};
  static auto configuration = CostConfiguration<Scalar>{weights, &metric, &evaluator};

  // Create context.
  const auto context = CostContext{state_.get(), &observation};

  // Create cost.
  auto cost = new ExteroceptiveCost<OptimizerSuite::CERES>{configuration, context};

  // Create loss.
  // Note: Constant is equivalent to a one pixel error (4.37234291e-3 for two pixel).
  static auto loss = std::make_unique<ceres::HuberLoss>(1.6e-3);

  // Add residual block.
  const auto parameter_blocks = cost->update();
  DCHECK(hasParameterBlocks(parameter_blocks));
  problem_.AddResidualBlock(cost, loss.get(), parameter_blocks);
}

auto CeresOptimizer::add(VisualPixelObservation& observation) -> void {
  // Create options.
  static auto weights = nullptr;
  static auto metric = CartesianMetric<Pixel<Scalar>>{};
  static auto evaluator = VisualPixelEvaluator<Manifold>{};
  static auto configuration = CostConfiguration<Scalar>{weights, &metric, &evaluator};

  // Create context.
  const auto context = CostContext{state_.get(), &observation};

  // Create cost.
  auto cost = new ExteroceptiveCost<OptimizerSuite::CERES>{configuration, context};

  // Create loss.
  static auto loss = std::make_unique<ceres::HuberLoss>(0.5);

  // Add residual block.
  const auto parameter_blocks = cost->update();
  DCHECK(hasParameterBlocks(parameter_blocks));
  problem_.AddResidualBlock(cost, loss.get(), parameter_blocks);
}

auto CeresOptimizer::add(ManifoldObservation<Manifold>& observation) -> void {
  // Create options.
  static auto weights = nullptr;
  static auto metric = ManifoldMetric<Manifold>{};
  static auto evaluator = ManifoldEvaluator<Manifold>{};
  static auto configuration = CostConfiguration<Scalar>{weights, &metric, &evaluator};

  // Create context.
  const auto context = CostContext{state_.get(), &observation};

  // Create cost.
  auto cost = new ExteroceptiveCost<OptimizerSuite::CERES>{configuration, context};

  // Add residual block.
  const auto parameter_blocks = cost->update();
  DCHECK(hasParameterBlocks(parameter_blocks));
  problem_.AddResidualBlock(cost, nullptr, parameter_blocks);
}

auto CeresOptimizer::add(InertialObservation<Manifold>& observation) -> void {
  // Create options.
  static auto weights = nullptr;
  static auto metric = CartesianMetric<Cartesian<Scalar, 6>>{};
  static auto evaluator = InertialEvaluator<Manifold>{};
  static auto configuration = CostConfiguration<Scalar>{weights, &metric, &evaluator};

  // Create context.
  const auto context = CostContext{state_.get(), &observation};

  // Create cost.
  auto cost = new ExteroceptiveCost<OptimizerSuite::CERES>{configuration, context};

  // Create loss.
  constexpr auto kLossScale = 1.6e-5;
  static auto loss = std::make_unique<ceres::ScaledLoss>(nullptr, kLossScale, ceres::TAKE_OWNERSHIP);

  // Add residual block.
  const auto parameter_blocks = cost->update();
  DCHECK(hasParameterBlocks(parameter_blocks));
  problem_.AddResidualBlock(cost, loss.get(), parameter_blocks);
}

auto CeresOptimizer::optimize() -> void {
  ceres::Solver::Summary summary;
  ceres::Solve(kDefaultSolverOptions, &problem_, &summary);
  LOG(INFO) << summary.BriefReport();
}

auto CeresOptimizer::hasParameterBlocks(const Pointers<Scalar>& parameter_blocks) -> bool {
  return std::all_of(parameter_blocks.cbegin(), parameter_blocks.cend(), [this](const auto& parameter_block) -> bool { return problem_.HasParameterBlock(parameter_block); });
}

auto CeresOptimizer::updateState(const Range& range) -> void {
  const auto& elements = state().elements();
  const auto [left_padding, right_padding] = state().interpolator()->layout().outerPadding();
  const auto begin = std::prev(elements.upper_bound(range.lowerBound()), left_padding);
  const auto end = std::next(elements.upper_bound(range.upperBound()), right_padding);

  const auto stamp_0 = (*begin)->stamp();
  const auto stamp_n = (*std::prev(end))->stamp();
  const auto lower_bound = range.lowerBound();

  // Add new parameter blocks.
  for (auto itr = begin; itr != end; ++itr) {
    const auto& variable = *(itr);
    auto vector = variable->asVector();
    auto data = vector.data();
    const auto size = vector.size();
    if (!problem_.HasParameterBlock(data)) {
      DLOG(INFO) << "Adding state variable at " << variable->stamp() << ".";
      variables_.insert(variable.get());
      problem_.AddParameterBlock(data, size, state_manifold_.get());
    }
  }

  // Update existing parameter blocks.
  auto itr = variables_.begin();
  while (itr != variables_.cend()) {
    // Retrieve variable.
    const auto itr_i = itr++;
    const auto& variable_i = *itr_i;

    // Retrieve stamp and address.
    const auto stamp = variable_i->stamp();
    const auto vector = variable_i->asVector();
    const auto data = vector.data();
    DCHECK(problem_.HasParameterBlock(data));

    // Set parameter blocks constant.
    if (stamp <= lower_bound || stamp_n < stamp) {
      if (!problem_.IsParameterBlockConstant(data)) {
        DLOG(INFO) << "Setting state variable at " << variable_i->stamp() << " constant.";
        problem_.SetParameterBlockConstant(data);
      }
    }

    // Remove parameter blocks without residuals.
    if (stamp < stamp_0 || stamp_n < stamp) {
      // Retrieve residual block ids.
      std::vector<ceres::ResidualBlockId> residual_block_ids;
      problem_.GetResidualBlocksForParameterBlock(data, &residual_block_ids);

      if (residual_block_ids.empty()) {
        DLOG(INFO) << "Removing state variable at " << variable_i->stamp() << ".";
        problem_.RemoveParameterBlock(data);
        variables_.erase(itr_i);
      }
    }
  }

  DLOG(INFO) << "Variables: " << variables_.size() << ".";
}

auto CeresOptimizer::addLandmark(Landmark<Position<Scalar>>& landmark) -> void {
  // Add landmark.
  DCHECK(!landmarks_.contains(&landmark));
  landmarks_.insert(&landmark);
  // DLOG(INFO) << landmarks_.size();

  // Add landmark parameter block.
  auto address = landmark.variable().data();
  DCHECK(!problem_.HasParameterBlock(address));
  problem_.AddParameterBlock(address, Traits<Position<Scalar>>::kNumParameters);
  // DLOG(INFO) << problem_.NumParameterBlocks();
}

auto CeresOptimizer::updateLandmarks(const Range& range) -> void {
  // DLOG(INFO) << "Landmarks: " << landmarks_.size() << ".";
  auto itr = landmarks_.cbegin();
  while (itr != landmarks_.cend()) {
    auto itr_i = itr++;
    const auto p_landmark = *itr_i;
    if (!p_landmark->range().intersects(range)) {
      // Remove parameter block.
      for (const auto& variable : p_landmark->variables()) {
        auto vector = variable->asVector();
        auto data = vector.data();
        DCHECK(problem_.HasParameterBlock(data));
        problem_.RemoveParameterBlock(data);
      }
      // DLOG(INFO) << problem_.NumParameterBlocks();

      // Remove landmark.
      landmarks_.erase(itr_i);
      // DLOG(INFO) << landmarks_.size();
    }
  }
  DLOG(INFO) << "Landmarks: " << landmarks_.size() << ".";
}

auto CeresOptimizer::updateSensor(IMU& /* imu */, const Range& /* range */) -> void {
  CHECK(false);
}

} // namespace hyper
