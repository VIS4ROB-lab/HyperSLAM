/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/environment/observations/manifold.hpp"
#include "hyper/messages/measurements/variable.hpp"
#include "hyper/optimizers/ceres/costs/exteroceptive.hpp"
#include "hyper/optimizers/ceres/manifolds/sensors/sensor.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/se3.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/stamped.hpp"
#include "hyper/optimizers/evaluators/evaluator.hpp"
#include "hyper/state/abstract.hpp"
#include "hyper/state/interpolators/basis.hpp"
#include "hyper/state/policies/se3.hpp"
#include "hyper/variables/groups/se3.hpp"
#include "hyper/variables/metrics/manifold.hpp"
#include "hyper/variables/stamped.hpp"

#include "tests/optimizers/evaluators/evaluator.hpp"
#include "tests/sensors/sensor.hpp"

namespace hyper::tests {

using ManifoldEvaluatorTestsTypes = ::testing::Types<SE3<Scalar>>;

TYPED_TEST_SUITE_P(ManifoldEvaluatorTests);

template <typename TAmbientSpace>
class ManifoldEvaluatorTests
    : public EvaluatorTests<OptimizerSuite::CERES> {
 public:
  // Definitions.
  using AmbientSpace = TAmbientSpace;
  using StampedAmbientSpace = Stamped<AmbientSpace>;
  using Measurement = ManifoldMeasurement<AmbientSpace>;
  using Observation = ManifoldObservation<AmbientSpace>;
  using Metric = ManifoldMetric<AmbientSpace>;

  using Cost = ExteroceptiveCost<OptimizerSuite::CERES>;
  using StateManifold = Manifold<StampedAmbientSpace, OptimizerSuite::CERES>;
  using SensorManifold = Manifold<Sensor, OptimizerSuite::CERES>;
  using Manifolds = Pointers<const ceres::Manifold>;

  /// Checks the gradients.
  /// \return True if correct.
  static auto checkGradients() -> bool {
    // Create state.
    auto state = Mock<AbstractState>::Random<AmbientSpace>();
    state->interpolator() = std::make_unique<BasisInterpolator>(3, true);
    state->policy() = std::make_unique<ManifoldPolicy<StampedAmbientSpace>>();
    const auto state_manifold = StateManifold{true, false, false};
    const auto state_range = state->range();

    // Create sensor.
    const auto sensor = Mock<Sensor>::Create();
    const auto sensor_manifold = SensorManifold{*sensor, false};

    // Create measurement.
    const auto stamp = state_range.sample();
    const auto element = Mock<AmbientSpace>::Random();
    auto measurement = Measurement{stamp, *sensor, element};

    // Create observation.
    auto observation = Observation{measurement};

    // Create cost.
    const auto metric = Metric{};
    const auto evaluator = Evaluator<Observation, AmbientSpace>{};
    const auto configuration = CostConfiguration<Scalar>{nullptr, &metric, &evaluator};
    const auto context = CostContext{state.get(), &observation};
    auto cost = Cost{configuration, context};

    // Collect manifolds.
    const auto num_state_manifolds = state->interpolator()->layout().outer.size;
    const auto state_manifolds = Manifolds(num_state_manifolds, &state_manifold);
    const auto sensor_manifolds = sensor_manifold.manifolds(stamp);

    Manifolds manifolds;
    manifolds.reserve(state_manifolds.size() + sensor_manifolds.size());
    manifolds.insert(manifolds.end(), state_manifolds.begin(), state_manifolds.end());
    manifolds.insert(manifolds.end(), sensor_manifolds.begin(), sensor_manifolds.end());

    // Gradient check.
    return Probe(cost, manifolds);
  }
};

TYPED_TEST_P(ManifoldEvaluatorTests, Gradients) {
  EXPECT_TRUE(this->checkGradients());
}

REGISTER_TYPED_TEST_SUITE_P(ManifoldEvaluatorTests, Gradients);

INSTANTIATE_TYPED_TEST_SUITE_P(HyperSystemTests, ManifoldEvaluatorTests, ManifoldEvaluatorTestsTypes);

} // namespace hyper::tests
