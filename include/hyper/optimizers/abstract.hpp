/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <set>
#include <unordered_set>

#include "hyper/environment/landmarks/forward.hpp"
#include "hyper/optimizers/forward.hpp"
#include "hyper/sensors/forward.hpp"
#include "hyper/yaml/forward.hpp"

#include "hyper/definitions.hpp"
#include "hyper/environment/environment.hpp"
#include "hyper/environment/observations/visual.hpp"
#include "hyper/state/abstract.hpp"
#include "hyper/state/policies/abstract.hpp"
#include "hyper/variables/groups/se3.hpp"
#include "hyper/variables/stamped.hpp"

namespace hyper {

class AbstractOptimizer {
 public:
  // Definitions.
  using Manifold = SE3<Scalar>;
  using StampedManifold = Stamped<Manifold>;
  using Range = hyper::Range<Stamp, BoundaryPolicy::LOWER_INCLUSIVE_ONLY>;
  using Window = hyper::Range<Stamp, BoundaryPolicy::LOWER_INCLUSIVE_ONLY>;

  /// Default destructor.
  virtual ~AbstractOptimizer() = default;

  /// Optimization window accessor.
  /// \return Optimization window.
  [[nodiscard]] auto window() const -> const Window&;

  /// Sets the optimization window.
  /// \param window Input optimization window.
  auto setWindow(const Window& window) -> void;

  /// Adds a sensor.
  /// \param sensor Input sensor.
  auto addSensor(Sensor& sensor) -> void;

  /// Checks whether a sensor has been added.
  /// \param sensor Query sensor.
  /// \return True if sensor has been added.
  [[nodiscard]] auto hasSensor(const Sensor& sensor) const -> bool;

  /// Removes a sensor.
  /// \param sensor Input sensor.
  auto removeSensor(const Sensor& sensor) -> void;

  /// Environment accessor.
  /// \return Environment.
  [[nodiscard]] auto environment() const -> const Environment<Manifold>&;

  /// Swaps the environment.
  /// \param environment New environment.
  virtual auto swapEnvironment(std::unique_ptr<Environment<Manifold>>& environment) -> void = 0;

  /// State accessor.
  /// \return State.
  [[nodiscard]] auto state() const -> const AbstractState&;

  /// Swaps the state.
  /// \param state New state.
  virtual auto swapState(std::unique_ptr<AbstractState>& state) -> void = 0;

  /// Sets the gravity constant.
  /// \param constant Constancy flag.
  virtual auto setGravityConstant(bool constant) -> void = 0;

  /// Submits a message.
  /// \param message Input message.
  auto submit(std::unique_ptr<AbstractMessage>&& message) -> void;

  /// Adds an observation.
  /// \param observation Input observation.
  virtual auto add(BearingObservation& observation) -> void = 0;

  /// Adds an observation.
  /// \param observation Input observation.
  virtual auto add(PixelObservation& observation) -> void = 0;

  /// Adds an observation.
  /// \param observation Input observation.
  virtual auto add(ManifoldObservation<Manifold>& observation) -> void = 0;

  /// Adds an observation.
  /// \param observation Input observation.
  virtual auto add(InertialObservation<Manifold>& observation) -> void = 0;

  /// Solves the optimization.
  virtual auto optimize() -> void = 0;

 protected:
  struct StampedManifoldCompare {
    using is_transparent = std::true_type;
    auto operator()(const AbstractStamped<Stamp>* lhs, const AbstractStamped<Stamp>* rhs) const -> bool {
      return lhs->stamp() < rhs->stamp();
    }
    auto operator()(const AbstractStamped<Stamp>* lhs, const Stamp& rhs) const -> bool {
      return lhs->stamp() < rhs;
    }
    auto operator()(const Stamp& lhs, const AbstractStamped<Stamp>* rhs) const -> bool {
      return lhs < rhs->stamp();
    }
  };

  /// Constructor from YAML node.
  /// \param yaml_node Input YAML node.
  explicit AbstractOptimizer(const YAML::Node& yaml_node);

  /// Environment modifier.
  /// \return Environment.
  [[nodiscard]] auto mutableEnvironment() -> Environment<Manifold>&;

  /// State modifier.
  /// \return State.
  [[nodiscard]] auto mutableState() -> AbstractState&;

  /// Rebases time stamps.
  /// \return Rebased time stamp.
  [[nodiscard]] auto rebase(const Stamp& stamp) const -> Stamp;

  /// Processes a message.
  /// \param message Input message.
  auto process(const AbstractMessage& message) -> void;

  /// Processes a message.
  /// \param message Input message.
  auto process(const VisualTracks& message) -> void;

  /// Processes a message.
  /// \param message Input message.
  auto process(const InertialMeasurement<Manifold>& message) -> void;

  /// Processes a message.
  /// \param message Input message.
  auto process(const ManifoldMeasurement<Manifold>& message) -> void;

  /// Initializes the state.
  /// \param range Input range.
  virtual auto initializeState(const Range& range) -> void = 0;

  /// Extrapolates the state.
  /// \param stamp Separation between stamped state parameters.
  /// \param n Number of extrapolated state parameters.
  virtual auto extrapolateState(const Stamp& stamp, int n) -> void = 0;

  /// Updates the state manifold.
  virtual auto updateStateManifold() -> void = 0;

  /// Updates the state.
  virtual auto updateState() -> void = 0;

  /// Updates the landmarks.
  virtual auto updateLandmarks() -> void = 0;

  /// Updates the gyroscope bias.
  /// \param imu Input IMU.
  /// \param range Input range.
  virtual auto updateGyroscopeBias(IMU& imu, const Range& range) -> void = 0;

  /// Updates the accelerometer bias.
  /// \param imu Input IMU.
  /// \param range Input range.
  virtual auto updateAccelerometerBias(IMU& imu, const Range& range) -> void = 0;

  Stamp root_stamp_;         ///< Root stamp.
  Stamp default_separation_; ///< Default separation.

  Window window_;    ///< Optimization window.
  Stamp max_window_; ///< Maximum optimization window.

  std::unordered_set<Sensor*> sensors_;                ///< Sensors.
  std::unique_ptr<Environment<Manifold>> environment_; ///< Environment.
  std::unique_ptr<AbstractState> state_;               ///< State.

  std::set<AbstractStamped<Stamp>*, StampedManifoldCompare> active_state_parameters_; ///< Active state parameters.
  std::unordered_set<AbstractLandmark*> active_landmarks_;                            ///< Active landmarks.
};

} // namespace hyper
