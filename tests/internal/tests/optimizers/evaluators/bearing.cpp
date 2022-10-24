/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/environment/observations/visual.hpp"
#include "hyper/messages/measurements/visual.hpp"
#include "hyper/optimizers/ceres/costs/exteroceptive.hpp"
#include "hyper/optimizers/ceres/manifolds/sensors/camera.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/se3.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/stamped.hpp"
#include "hyper/optimizers/evaluators/evaluator.hpp"
#include "hyper/state/abstract.hpp"
#include "hyper/state/interpolators/basis.hpp"
#include "hyper/state/policies/se3.hpp"
#include "hyper/variables/groups/se3.hpp"
#include "hyper/variables/metrics/angular.hpp"
#include "hyper/variables/stamped.hpp"

#include "tests/optimizers/evaluators/evaluator.hpp"
#include "tests/sensors/camera.hpp"

namespace hyper::tests {

using BearingEvaluatorTestsTypes = ::testing::Types<SE3<Scalar>>;

TYPED_TEST_SUITE_P(BearingEvaluatorTests);

template <typename TAmbientSpace>
class BearingEvaluatorTests
    : public EvaluatorTests<OptimizerSuite::CERES> {
 public:
  // Definitions.
  using AmbientSpace = TAmbientSpace;
  using StampedAmbientSpace = Stamped<AmbientSpace>;
  using Measurement = BearingMeasurement;
  using Observation = VisualBearingObservation;
  using Landmark = Observation::Landmark;
  using Metric = AngularMetric<Bearing<Scalar>>;

  using Cost = ExteroceptiveCost<OptimizerSuite::CERES>;
  using StateManifold = Manifold<StampedAmbientSpace, OptimizerSuite::CERES>;
  using CameraManifold = Manifold<Camera, OptimizerSuite::CERES>;
  using LandmarkManifold = Manifold<Position<Scalar>, OptimizerSuite::CERES>;
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

    // Create camera.
    const auto camera = Mock<Camera>::Create();
    const auto camera_manifold = CameraManifold{*camera, false};

    // Create measurement.
    const auto stamp = state_range.sample();
    const auto bearing = Mock<Bearing<Scalar>>::Random();
    auto measurement = Measurement{stamp, *camera, bearing};

    // Create observation.
    auto landmark = Mock<Landmark>::InSphere();
    auto landmark_manifold = LandmarkManifold{false};
    auto observation = Observation{measurement, landmark};

    // Create cost.
    const auto metric = Metric{};
    const auto evaluator = Evaluator<Observation, AmbientSpace>{};
    const auto configuration = CostConfiguration<Scalar>{nullptr, &metric, &evaluator};
    const auto context = CostContext{state.get(), &observation};
    auto cost = Cost{configuration, context};

    // Collect manifolds.
    const auto num_state_manifolds = state->interpolator()->layout().outer.size;
    const auto state_manifolds = Manifolds(num_state_manifolds, &state_manifold);
    const auto camera_manifolds = camera_manifold.manifolds(stamp);

    Manifolds manifolds;
    manifolds.reserve(state_manifolds.size() + camera_manifolds.size() + 1);
    manifolds.insert(manifolds.end(), state_manifolds.begin(), state_manifolds.end());
    manifolds.insert(manifolds.end(), camera_manifolds.begin(), camera_manifolds.end());
    manifolds.emplace_back(&landmark_manifold);

    // Gradient check.
    return Probe(cost, manifolds);
  }
};

TYPED_TEST_P(BearingEvaluatorTests, Gradients) {
  EXPECT_TRUE(this->checkGradients());
}

REGISTER_TYPED_TEST_SUITE_P(BearingEvaluatorTests, Gradients);

INSTANTIATE_TYPED_TEST_SUITE_P(HyperSystemTests, BearingEvaluatorTests, BearingEvaluatorTestsTypes);

} // namespace hyper::tests
