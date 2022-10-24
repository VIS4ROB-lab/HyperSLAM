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

  /// Root accessor.
  /// \return Root.
  [[nodiscard]] auto root() const -> const Stamp&;

  /// Optimization window accessor.
  /// \return Optimization window.
  [[nodiscard]] auto window() const -> const Window&;

  /// Sets the optimization window.
  /// \param window Input optimization window.
  auto setWindow(const Window& window) -> void;

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

  /// Submits a message.
  /// \param message Input message.
  auto submit(std::unique_ptr<AbstractMessage>&& message) -> void;

  /// Adds an observation.
  /// \param observation Input observation.
  virtual auto add(VisualBearingObservation& observation) -> void = 0;

  /// Adds an observation.
  /// \param observation Input observation.
  virtual auto add(VisualPixelObservation& observation) -> void = 0;

  /// Adds an observation.
  /// \param observation Input observation.
  virtual auto add(ManifoldObservation<Manifold>& observation) -> void = 0;

  /// Adds an observation.
  /// \param observation Input observation.
  virtual auto add(InertialObservation<Manifold>& observation) -> void = 0;

  /// Checks whether a sensor has been added.
  /// \param sensor Query sensor.
  /// \return True if sensor has been added.
  [[nodiscard]] virtual auto hasSensor(const Sensor& sensor) const -> bool = 0;

  /// Sets the gravity constant.
  /// \param set_constant Constancy flag.
  virtual auto setGravityConstant(bool set_constant) -> void = 0;

  /// Solves the optimization.
  virtual auto optimize() -> void = 0;

 protected:
  /// Constructor from YAML node.
  /// \param yaml_node Input YAML node.
  explicit AbstractOptimizer(const YAML::Node& yaml_node);

  /// Environment modifier.
  /// \return Environment.
  [[nodiscard]] auto mutableEnvironment() -> Environment<Manifold>&;

  /// State modifier.
  /// \return State.
  [[nodiscard]] auto mutableState() -> AbstractState&;

  /// Processes a message.
  /// \param message Input message.
  auto process(const AbstractMessage& message) -> void;

  /// Processes a message.
  /// \param message Input message.
  auto process(const VisualTracks& message) -> void;

  /// Processes a message.
  /// \param message Input message.
  auto process(const ManifoldMeasurement<Manifold>& message) -> void;

  /// Processes a message.
  /// \param message Input message.
  auto process(const InertialMeasurement<Manifold>& message) -> void;

  /// State update.
  /// \param range Input range.
  virtual auto updateState(const Range& range) -> void = 0;

  /// Adds a landmark.
  /// \param landmark Input landmark.
  virtual auto addLandmark(Landmark<Position<Scalar>>& landmark) -> void = 0;

  /// Updates the landmarks.
  /// \param range Input range.
  virtual auto updateLandmarks(const Range& range) -> void = 0;

  /// Sensor update.
  /// \param imu Input sensor.
  /// \param range Input range.
  virtual auto updateSensor(IMU& imu, const Range& range) -> void = 0;

  Stamp root_stamp_; ///< Root stamp.
  Stamp separation_; ///< Separation.

  Window window_;    ///< Optimization window.
  Stamp max_window_; ///< Maximum optimization window.

  std::unique_ptr<Environment<Manifold>> environment_; ///< Environment.
  std::set<AbstractLandmark*, std::less<>> landmarks_; ///< Active landmarks.

  std::unique_ptr<AbstractState> state_;                      ///< State.
  std::set<AbstractStamped<Scalar>*, std::less<>> variables_; ///< Active variables.

  std::set<AbstractLandmark*, std::less<>> active_landmarks_;              ///< Active landmarks.
  std::set<AbstractStamped<Scalar>*, std::less<>> active_state_variables_; ///< Active state variables.
};

} // namespace hyper
