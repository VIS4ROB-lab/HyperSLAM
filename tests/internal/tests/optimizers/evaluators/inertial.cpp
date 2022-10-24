/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/environment/observations/inertial.hpp"
#include "hyper/environment/observations/manifold.hpp"
#include "hyper/messages/measurements/inertial.hpp"
#include "hyper/optimizers/ceres/costs/exteroceptive.hpp"
#include "hyper/optimizers/ceres/manifolds/sensors/imu.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/gravity.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/se3.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/stamped.hpp"
#include "hyper/optimizers/evaluators/evaluator.hpp"
#include "hyper/state/abstract.hpp"
#include "hyper/state/interpolators/basis.hpp"
#include "hyper/state/policies/se3.hpp"
#include "hyper/variables/gravity.hpp"
#include "hyper/variables/groups/se3.hpp"
#include "hyper/variables/metrics/cartesian.hpp"
#include "hyper/variables/stamped.hpp"

#include "tests/optimizers/evaluators/evaluator.hpp"
#include "tests/sensors/imu.hpp"

namespace hyper::tests {

using InertialEvaluatorTestsTypes = ::testing::Types<SE3<Scalar>>;

TYPED_TEST_SUITE_P(InertialEvaluatorTests);

template <typename TAmbientSpace>
class InertialEvaluatorTests
    : public EvaluatorTests<OptimizerSuite::CERES> {
 public:
  // Definitions.
  using AmbientSpace = TAmbientSpace;
  using TangentSpace = Tangent<AmbientSpace>;
  using StampedAmbientSpace = Stamped<AmbientSpace>;
  using Measurement = InertialMeasurement<AmbientSpace>;
  using Observation = InertialObservation<AmbientSpace>;
  using Metric = CartesianMetric<Cartesian<Scalar, Traits<TangentSpace>::kNumParameters>>;

  using Cost = ExteroceptiveCost<OptimizerSuite::CERES>;
  using StateManifold = Manifold<StampedAmbientSpace, OptimizerSuite::CERES>;
  using IMUManifold = Manifold<IMU, OptimizerSuite::CERES>;
  using GravityManifold = Manifold<Gravity<Scalar>, OptimizerSuite::CERES>;
  using Manifolds = Pointers<const ceres::Manifold>;

  static constexpr auto kDefaultGyroscopeBiasSeparation = 10;
  static constexpr auto kDefaultAccelerometerBiasSeparation = 10;

  /// Checks the gradients.
  /// \return True if correct.
  static auto checkGradients() -> bool {
    // Create state.
    auto state = Mock<AbstractState>::Random<AmbientSpace>();
    state->interpolator() = std::make_unique<BasisInterpolator>(3, true);
    state->policy() = std::make_unique<ManifoldPolicy<StampedAmbientSpace>>();
    const auto state_manifold = StateManifold{true, false, false};
    const auto state_range = state->range();

    // Create query time and mock sensor.
    const auto imu = Mock<IMU>::Create();
    const auto imu_manifold = IMUManifold{*imu, false};

    // Initialize gyroscope bias.
    const auto outer_gyroscope_bias_padding = imu->gyroscopeBias().interpolator()->layout().outerPadding();
    const auto gyroscope_bias_lower = state_range.lower - outer_gyroscope_bias_padding.left * kDefaultGyroscopeBiasSeparation;
    const auto gyroscope_bias_upper = state_range.upper + outer_gyroscope_bias_padding.right * kDefaultGyroscopeBiasSeparation;
    const auto num_gyroscope_bias_parameters = std::ceil((gyroscope_bias_upper - gyroscope_bias_lower) / kDefaultGyroscopeBiasSeparation);

    for (auto i = 0; i < num_gyroscope_bias_parameters; ++i) {
      auto timed_element = std::make_unique<Traits<IMU>::GyroscopeBias>();
      timed_element->stamp() = gyroscope_bias_lower + i * kDefaultGyroscopeBiasSeparation;
      timed_element->variable().setRandom();
      imu->gyroscopeBias().elements().insert(std::move(timed_element));
    }

    // Initialize accelerometer bias.
    const auto outer_accelerometer_bias_padding = imu->accelerometerBias().interpolator()->layout().outerPadding();
    const auto accelerometer_bias_lower = state_range.lower - outer_accelerometer_bias_padding.left * kDefaultAccelerometerBiasSeparation;
    const auto accelerometer_bias_upper = state_range.upper + outer_accelerometer_bias_padding.right * kDefaultAccelerometerBiasSeparation;
    const auto num_accelerometer_bias_parameters = std::ceil((accelerometer_bias_upper - accelerometer_bias_lower) / kDefaultAccelerometerBiasSeparation);

    for (auto i = 0; i < num_accelerometer_bias_parameters; ++i) {
      auto timed_element = std::make_unique<Traits<IMU>::GyroscopeBias>();
      timed_element->stamp() = accelerometer_bias_lower + i * kDefaultAccelerometerBiasSeparation;
      timed_element->variable().setRandom();
      imu->accelerometerBias().elements().insert(std::move(timed_element));
    }

    // Create measurement.
    const auto stamp = state_range.sample();
    const auto element = TangentSpace::Random().eval();
    auto measurement = Measurement{stamp, *imu, element};

    // Create observation.
    auto gravity = Mock<Gravity<Scalar>>::Random();
    auto gravity_manifold = GravityManifold{false};
    auto observation = Observation{measurement, gravity};

    // Create cost.
    const auto metric = Metric{};
    const auto evaluator = Evaluator<Observation, AmbientSpace>{};
    const auto configuration = CostConfiguration<Scalar>{nullptr, &metric, &evaluator};
    const auto context = CostContext{state.get(), &observation};
    auto cost = Cost{configuration, context};

    // Collect manifolds.
    const auto num_state_manifolds = state->interpolator()->layout().outer.size;
    const auto state_manifolds = Manifolds(num_state_manifolds, &state_manifold);
    const auto sensor_manifolds = imu_manifold.manifolds(stamp);

    Manifolds manifolds;
    manifolds.reserve(state_manifolds.size() + sensor_manifolds.size() + 1);
    manifolds.insert(manifolds.end(), state_manifolds.begin(), state_manifolds.end());
    manifolds.insert(manifolds.end(), sensor_manifolds.begin(), sensor_manifolds.end());
    manifolds.emplace_back(&gravity_manifold);

    // Gradient check.
    return Probe(cost, manifolds);
  }
};

TYPED_TEST_P(InertialEvaluatorTests, Gradients) {
  EXPECT_TRUE(this->checkGradients());
}

REGISTER_TYPED_TEST_SUITE_P(InertialEvaluatorTests, Gradients);

INSTANTIATE_TYPED_TEST_SUITE_P(HyperSystemTests, InertialEvaluatorTests, InertialEvaluatorTestsTypes);

} // namespace hyper::tests
