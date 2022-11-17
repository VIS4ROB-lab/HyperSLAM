/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <typeindex>

#include "hyper/messages/measurements/inertial.hpp"
#include "hyper/messages/measurements/variable.hpp"
#include "hyper/messages/visual.hpp"
#include "hyper/optimizers/abstract.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/sensors/imu.hpp"
#include "hyper/state/interpolators/basis.hpp"
#include "hyper/state/policies/se3.hpp"
#include "hyper/system/components/frontends/visual/MonocularUtilizer.h"
#include "hyper/variables/intrinsics.hpp"
#include "hyper/yaml/yaml.hpp"

namespace hyper {

namespace {

// Parameter names.
constexpr auto kSeparationName = "separation";
constexpr auto kMaxWindowName = "max_window";

// Default parameters.
constexpr Stamp kDefaultRootStamp = 0;
constexpr Stamp kDefaultSeparation = 0.1;
constexpr Stamp kDefaultMaxWindow = 3.0;

} // namespace

auto AbstractOptimizer::root() const -> const Stamp& {
  return root_stamp_;
}

auto AbstractOptimizer::window() const -> const Window& {
  return window_;
}

auto AbstractOptimizer::setWindow(const Window& window) -> void {
  // Range checks.
  const auto range = state().range();
  DCHECK_LE(range.lowerBound(), window_.lowerBound());
  DCHECK_LE(window_.upperBound(), range.upperBound());

  // Update window.
  window_ = window;
  DLOG(INFO) << "Window: " << window_;

  // Update landmarks.
  updateLandmarks(window_);

  // Update state.
  updateState(window_);

  // Update gravity.
  if (window_.size() < range.size()) {
    setGravityConstant(true);
  } else {
    setGravityConstant(false);
  }
}

auto AbstractOptimizer::environment() const -> const Environment<Manifold>& {
  DCHECK(environment_ != nullptr);
  return *environment_;
}

auto AbstractOptimizer::state() const -> const AbstractState& {
  DCHECK(state_ != nullptr);
  return *state_;
}


auto AbstractOptimizer::submit(std::unique_ptr<AbstractMessage>&& message) -> void {
  // Initialize state.
  if (!state_) {
    root_stamp_ = message->stamp();
    const auto range = Range{0, separation_};
    auto interpolator = std::make_unique<BasisInterpolator>();
    auto policy = std::make_unique<ManifoldPolicy<StampedManifold>>();
    state_ = std::make_unique<AbstractState>(std::move(interpolator), std::move(policy));

    // Retrieve layout.
    const auto layout = state().interpolator()->layout();

    // Create variables.
    for (auto i = 0; i < layout.outer.size; ++i) {
      auto stamped_variable = std::make_unique<StampedManifold>();
      stamped_variable->stamp() = range.lowerBound() + (i - (layout.outer.size - 1) / 2) * separation_;
      stamped_variable->variable() = Manifold{};
      mutableState().elements().insert(std::move(stamped_variable));
    }

    // Set window.
    setWindow(range);
  }

  // Retrieve message stamp.
  auto& stamp = message->stamp();
  stamp -= root_stamp_;

  // Retrieve state range.
  const auto state_range = state().range();

  // Process message.
  if (state_range.contains(stamp)) {
    if (window_.contains(stamp)) {
      process(*message);
    } else if (window_.isSmaller(stamp)) {
      DLOG(FATAL) << "Not implemented.";
    } else {
      DLOG(FATAL) << "Not implemented.";
    }
  } else {
    if (state_range.isSmaller(stamp)) {
      DLOG(WARNING) << "Discarding out-of-scope message.";

    } else {
      optimize();

      const auto delta = stamp - window_.upperBound();
      DCHECK_LE(0, delta);

      const auto n = static_cast<int>(std::ceil(delta));
      DLOG_IF(WARNING, 1 < n) << "Window expanded by multiple states.";

      for (auto i = 1; i <= n; ++i) {
        const auto new_stamp = state().elements().rbegin()->get()->stamp() + i * separation_;
        DLOG(INFO) << "Extrapolating state variable at " << new_stamp << ".";
        auto itr_0 = std::next(state().elements().rbegin(), 0);
        auto itr_1 = std::next(state().elements().rbegin(), 1);
        auto new_element = std::make_unique<StampedManifold>();
        new_element->stamp() = new_stamp;
        static_cast<StampedManifold&>(**itr_0).variable() = static_cast<const StampedManifold&>(**itr_1).variable();
        new_element->variable() = static_cast<const StampedManifold&>(**itr_1).variable();
        mutableState().elements().insert(std::move(new_element));
      }

      const auto x = n * separation_;
      const auto upper_boundary = window_.upper + x;
      const auto window_size = window_.size();
      const auto window = (window_size + x <= max_window_) ? Window{window_.lower, upper_boundary} : Window{upper_boundary - window_size, upper_boundary};
      setWindow(window);
      process(*message);
    }
  }
}

Stamp stamp_;
Camera camera_;


AbstractOptimizer::AbstractOptimizer(const YAML::Node& yaml_node)
    : root_stamp_{kDefaultRootStamp},
      separation_{kDefaultSeparation},
      window_{},
      max_window_{kDefaultMaxWindow},
      environment_{nullptr},
      landmarks_{},
      state_{nullptr},
//      prev_message{nullptr},
      variables_{} {
  if (!yaml_node.IsNull()) {
    separation_ = yaml::ReadAs<Stamp>(yaml_node, kSeparationName);
    max_window_ = yaml::ReadAs<Stamp>(yaml_node, kMaxWindowName);
  }
}

auto AbstractOptimizer::mutableEnvironment() -> Environment<Manifold>& {
  return const_cast<Environment<Manifold>&>(std::as_const(*this).environment());
}

auto AbstractOptimizer::mutableState() -> AbstractState& {
  return const_cast<AbstractState&>(std::as_const(*this).state());
}

auto AbstractOptimizer::process(const AbstractMessage& message) -> void {
  DCHECK(hasSensor(message.sensor()));
  const auto index = std::type_index{typeid(message)};
  if (index == typeid(VisualTracks)) {
    process(static_cast<const VisualTracks&>(message)); // NOLINT
  }
//  else if (index == typeid(VisualTracksInit)) {
//    process(static_cast<const VisualTracksInit&>(message)); // NOLINT
//  }
  else if (index == typeid(InertialMeasurement<Manifold>)) {
    process(static_cast<const InertialMeasurement<Manifold>&>(message)); // NOLINT
  } else if (index == typeid(ManifoldMeasurement<Manifold>)) {
    process(static_cast<const ManifoldMeasurement<Manifold>&>(message)); // NOLINT
  } else {
    LOG(WARNING) << "Unknown message.";
  }
}

//auto AbstractOptimizer::process(const VisualTracksInit& message) -> void {
//  // C0, C1 need to be calculated by initialization
//
//  const auto& initial_track = message.initial_track;
//  const auto& current_track = message.current_track;
//  // Unpack cameras.
//  const auto& C0 = initial_track->sensor();
//  const auto& C1 = current_track->sensor();
//  const auto& [I0, P0] = initial_track->getTrack(C0);
//  const auto& [I1, P1] = current_track->getTrack(C1);
//  DCHECK(hasSensor(C0) && hasSensor(C1));
//
//  // Compute relative transformation.
//  const auto q_S_wb = StateQuery{initial_track->stamp()};
//  const auto S_wb = state().evaluate(q_S_wb);
//  const auto T_w0 = S_wb.derivativeAs<Manifold>(0).groupPlus(C0.transformation());
////  const auto T_01 = C0.transformation().groupInverse().groupPlus(C1.transformation());
//  const auto T_01 = message.initial_pose->groupPlus(message.current_pose->groupInverse());
//
//
//  // Convert points to pixels.
//  std::vector<Pixel<Scalar>> PX0;
//  PX0.reserve(P0.size());
//  std::transform(P0.cbegin(), P0.cend(), std::back_inserter(PX0), [](const auto& p) -> Pixel<Scalar> { return {p.x, p.y}; });
//  DCHECK_EQ(P0.size(), PX0.size());
//
//  std::vector<Pixel<Scalar>> PX1;
//  PX1.reserve(P1.size());
//  std::transform(P1.cbegin(), P1.cend(), std::back_inserter(PX1), [](const auto& p) -> Pixel<Scalar> { return {p.x, p.y}; });
//  DCHECK_EQ(P1.size(), PX1.size());
//
//  // Correct shutter stamps.
//  // const auto T0 = C0.correctShutterStamps(stamp, PX0);
//  // const auto T1 = C1.correctShutterStamps(stamp, PX1);
//
//  // Convert pixel to bearings.
//  const auto B0 = C0.convertPixelsToBearings(PX0);
//  const auto B1 = C1.convertPixelsToBearings(PX1);
//
//  /* for (auto i = std::size_t{0}; i < message.identifiers.size(); ++i) {
//    // Create observations.
//    auto [observation_0, inserted] = mutableEnvironment().addPixelMeasurement(message.identifiers[i], {stamp, C0, PX0[i]});
//    auto [observation_1, _] = mutableEnvironment().addPixelMeasurement(message.identifiers[i], {stamp, C1, PX1[i]});
//
//    // Triangulate if new landmark.
//    if (inserted) {
//      auto& landmark = observation_0.landmark();
//      const auto p_i = Camera::Triangulate(T_01, B0[i], B1[i]);
//      landmark.value() = T_w0.vectorPlus(p_i); // TODO: Apply threshold to determine validity of triangulation.
//      addLandmark(landmark);
//    }
//
//    // Add pixel observations.
//    add(observation_0);
//    add(observation_1);
//  } */
//
//  for (auto i = std::size_t{0}; i < current_track->identifiers.size(); ++i) {
//    // Create observations.
//    const auto identifier = current_track->identifiers[i];
//    auto [observation_0, inserted_0] = mutableEnvironment().addVisualMeasurement(identifier, BearingMeasurement{initial_track->stamp(), C0, B0[i]});
//    auto [observation_1, inserted_1] = mutableEnvironment().addVisualMeasurement(identifier, BearingMeasurement{current_track->stamp(), C1, B1[i]});
//
//    // Triangulate if new landmark.
//    if (inserted_0) {
//      auto& landmark = observation_0.landmark();
//      const auto p_i = Camera::Triangulate(T_01, B0[i], B1[i]);
//      landmark.variable() = T_w0.vectorPlus(p_i); // TODO: Apply threshold to determine validity of triangulation.
//      addLandmark(landmark);
//    }
//
//    // Add bearing observations.
//    add(observation_0);
//    add(observation_1);
//}
////      prev_message = std::make_unique<VisualTracks>(current_track->stamp(), current_track->sensor());
//      prev_message = std::make_unique<VisualTracks>(current_track->stamp(), current_track->sensor());
//      prev_message->identifiers = current_track->identifiers;
//      prev_message->tracks = current_track->tracks;
//      prev_message->lengths = current_track->lengths;
//      InitializationState_ = false;
//}

auto AbstractOptimizer::process(const VisualTracks& message) -> void {
  // Process individual views.
  DCHECK(environment_ != nullptr);
  const auto num_cameras = message.tracks.size();
  const auto& stamp = message.stamp();

  if (num_cameras == 2) {
    // Unpack cameras.
    const auto& C0 = message.sensor();
    const auto& C1 = (message.tracks.cbegin()->first == &C0) ? *message.tracks.crbegin()->first : *message.tracks.cbegin()->first;
    const auto& [I0, P0] = message.getTrack(C0);
    const auto& [I1, P1] = message.getTrack(C1);
    DCHECK(hasSensor(C0) && hasSensor(C1));

    // Compute relative transformation.
    const auto q_S_wb = StateQuery{stamp};
    const auto S_wb = state().evaluate(q_S_wb);
    const auto T_w0 = S_wb.derivativeAs<Manifold>(0).groupPlus(C0.transformation());
    const auto T_01 = C0.transformation().groupInverse().groupPlus(C1.transformation());

    // Convert points to pixels.
    std::vector<Pixel<Scalar>> PX0;
    PX0.reserve(P0.size());
    std::transform(P0.cbegin(), P0.cend(), std::back_inserter(PX0), [](const auto& p) -> Pixel<Scalar> { return {p.x, p.y}; });
    DCHECK_EQ(P0.size(), PX0.size());

    std::vector<Pixel<Scalar>> PX1;
    PX1.reserve(P1.size());
    std::transform(P1.cbegin(), P1.cend(), std::back_inserter(PX1), [](const auto& p) -> Pixel<Scalar> { return {p.x, p.y}; });
    DCHECK_EQ(P1.size(), PX1.size());

    // Correct shutter stamps.
    // const auto T0 = C0.correctShutterStamps(stamp, PX0);
    // const auto T1 = C1.correctShutterStamps(stamp, PX1);

    // Convert pixel to bearings.
    const auto B0 = C0.convertPixelsToBearings(PX0);
    const auto B1 = C1.convertPixelsToBearings(PX1);

    /* for (auto i = std::size_t{0}; i < message.identifiers.size(); ++i) {
      // Create observations.
      auto [observation_0, inserted] = mutableEnvironment().addPixelMeasurement(message.identifiers[i], {stamp, C0, PX0[i]});
      auto [observation_1, _] = mutableEnvironment().addPixelMeasurement(message.identifiers[i], {stamp, C1, PX1[i]});

      // Triangulate if new landmark.
      if (inserted) {
        auto& landmark = observation_0.landmark();
        const auto p_i = Camera::Triangulate(T_01, B0[i], B1[i]);
        landmark.value() = T_w0.vectorPlus(p_i); // TODO: Apply threshold to determine validity of triangulation.
        addLandmark(landmark);
      }

      // Add pixel observations.
      add(observation_0);
      add(observation_1);
    } */

    for (auto i = std::size_t{0}; i < message.identifiers.size(); ++i) {
      // Create observations.
      const auto identifier = message.identifiers[i];
      auto [observation_0, inserted_0] = mutableEnvironment().addVisualMeasurement(identifier, BearingMeasurement{stamp, C0, B0[i]});
      auto [observation_1, inserted_1] = mutableEnvironment().addVisualMeasurement(identifier, BearingMeasurement{stamp, C1, B1[i]});

      // Triangulate if new landmark.
      if (inserted_0) {
        auto& landmark = observation_0.landmark();
        const auto p_i = Camera::Triangulate(T_01, B0[i], B1[i]);
        landmark.variable() = T_w0.vectorPlus(p_i); // TODO: Apply threshold to determine validity of triangulation.
        addLandmark(landmark);
      }

      // Add bearing observations.
      add(observation_0);
      add(observation_1);
    }
  }
  else if (num_cameras == 1) {
    // Unpack cameras.
    const auto& C0 = message.sensor();
//    const auto& C1 = (message.tracks.cbegin()->first == &C0) ? *message.tracks.crbegin()->first : *message.tracks.cbegin()->first;
    const auto& [I0, P0] = message.getTrack(C0);
//    const auto& [I1, P1] = message.getTrack(C1);
    DCHECK(hasSensor(C0));

    // Compute relative transformation.
    const auto q_S_wb = StateQuery{stamp};
    const auto S_wb = state().evaluate(q_S_wb);
    const auto T_w0 = S_wb.derivativeAs<Manifold>(0).groupPlus(C0.transformation());
//    const auto T_01 = C0.transformation().groupInverse().groupPlus(C1.transformation());

    // Convert points to pixels.
    std::vector<Pixel<Scalar>> PX0;
    PX0.reserve(P0.size());
    std::transform(P0.cbegin(), P0.cend(), std::back_inserter(PX0), [](const auto& p) -> Pixel<Scalar> { return {p.x, p.y}; });
    DCHECK_EQ(P0.size(), PX0.size());

//    std::vector<Pixel<Scalar>> PX1;
//    PX1.reserve(P1.size());
//    std::transform(P1.cbegin(), P1.cend(), std::back_inserter(PX1), [](const auto& p) -> Pixel<Scalar> { return {p.x, p.y}; });
//    DCHECK_EQ(P1.size(), PX1.size());

    // Correct shutter stamps.
    // const auto T0 = C0.correctShutterStamps(stamp, PX0);
    // const auto T1 = C1.correctShutterStamps(stamp, PX1);

    // Convert pixel to bearings.
    const auto B0 = C0.convertPixelsToBearings(PX0);
//    const auto B1 = C1.convertPixelsToBearings(PX1);

    /* for (auto i = std::size_t{0}; i < message.identifiers.size(); ++i) {
      // Create observations.
      auto [observation_0, inserted] = mutableEnvironment().addPixelMeasurement(message.identifiers[i], {stamp, C0, PX0[i]});
      auto [observation_1, _] = mutableEnvironment().addPixelMeasurement(message.identifiers[i], {stamp, C1, PX1[i]});

      // Triangulate if new landmark.
      if (inserted) {
        auto& landmark = observation_0.landmark();
        const auto p_i = Camera::Triangulate(T_01, B0[i], B1[i]);
        landmark.value() = T_w0.vectorPlus(p_i); // TODO: Apply threshold to determine validity of triangulation.
        addLandmark(landmark);
      }

      // Add pixel observations.
      add(observation_0);
      add(observation_1);
    } */

    for (auto i = std::size_t{0}; i < message.identifiers.size(); ++i) {
      // Create observations.
      const auto identifier = message.identifiers[i];
      auto [observation_0, inserted_0] = mutableEnvironment().addVisualMeasurement(identifier, BearingMeasurement{stamp, C0, B0[i]});
//      auto [observation_1, inserted_1] = mutableEnvironment().addVisualMeasurement(identifier, BearingMeasurement{stamp, C1, B1[i]});

      // Triangulate if new landmark.
      Position<Scalar> p_i;
      if (inserted_0) {
        auto& landmark = observation_0.landmark();
        if (InitializationState_){
          p_i = message.positions[i];
//        }else{
//          const auto T_01 = prev_Tw0_.groupInverse().groupPlus(S_wb.derivativeAs<Manifold>(0));
//          p_i = Camera::Triangulate(T_01, prev_B0_[i], B0[i]);
//        }

        landmark.variable() = T_w0.vectorPlus(p_i); // TODO: Apply threshold to determine validity of triangulation.
        addLandmark(landmark);
      }

      // Add bearing observations.
      add(observation_0);
//      add(observation_1);
    }
    InitializationState_ = false;
//    prev_Tw0_ = S_wb.derivativeAs<Manifold>(0);
//    prev_B0_ = B0;
  }
  }
  else {
    LOG(FATAL) << "Unsupported camera configuration.";
  }
}

auto AbstractOptimizer::process(const ManifoldMeasurement<Manifold>& message) -> void {
  // Create observation and residual.
  auto& observation = mutableEnvironment().addManifoldMeasurement(message);
  add(observation);
}

auto AbstractOptimizer::process(const InertialMeasurement<Manifold>& message) -> void {
  // Fetch parameters.
  const auto& stamp = message.stamp();
  auto& imu = const_cast<IMU&>(message.sensor());

  // Update gyroscope bias.
  const auto& gyroscope_bias = imu.gyroscopeBias();
  const auto& accelerometer_bias = imu.accelerometerBias();
  if (gyroscope_bias.elements().empty() ||
      accelerometer_bias.elements().empty() ||
      !gyroscope_bias.range().contains(stamp) ||
      !accelerometer_bias.range().contains(stamp)) {
    updateSensor(imu, window_);
  }
  DCHECK(gyroscope_bias.range().contains(stamp));
  DCHECK(accelerometer_bias.range().contains(stamp));

  // Create observation and residual.
  auto& observation = mutableEnvironment().addInertialMeasurement(message);
  add(observation);
}

} // namespace hyper
