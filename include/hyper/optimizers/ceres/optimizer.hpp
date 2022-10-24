/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

#include "hyper/variables/forward.hpp"

#include "hyper/optimizers/abstract.hpp"
#include "hyper/optimizers/ceres/manifolds/sensors/sensor.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/gravity.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/stamped.hpp"

namespace hyper {

template <>
class Optimizer<OptimizerSuite::CERES> final
    : public AbstractOptimizer {
 public:
  using StateManifold = hyper::Manifold<StampedManifold, OptimizerSuite::CERES>;
  using SensorManifold = hyper::Manifold<Sensor, OptimizerSuite::CERES>;
  using GravityManifold = hyper::Manifold<Gravity<Scalar>, OptimizerSuite::CERES>;

  /// Constructor from YAML node and set of sensors.
  /// \param yaml_node Input YAML node.
  /// \param sensors Vector containing sensors.
  explicit Optimizer(const YAML::Node& yaml_node = {}, const std::vector<Sensor*>& sensors = {});

  /// Swaps the environment.
  /// \param environment New environment.
  /// \return Previous environment.
  auto swapEnvironment(std::unique_ptr<Environment<Manifold>>& environment) -> void final;

  /// Swaps the state.
  /// \param state New state.
  auto swapState(std::unique_ptr<AbstractState>& state) -> void final;

  /// Adds an observation to the optimization.
  /// \param observation Observation to add.
  auto add(VisualBearingObservation& observation) -> void final;

  /// Adds an observation to the optimization.
  /// \param observation Observation to add.
  auto add(ManifoldObservation<Manifold>& observation) -> void final;

  /// Adds an observation to the optimization.
  /// \param observation Observation to add.
  auto add(InertialObservation<Manifold>& observation) -> void final;

  /// Adds an observation to the optimization.
  /// \param observation Observation to add.
  auto add(VisualPixelObservation& observation) -> void final;

  /// Checks whether a sensor has been added.
  /// \param sensor Query sensor.
  /// \return True if sensor has been added.
  [[nodiscard]] auto hasSensor(const Sensor& sensor) const -> bool final;

  /// Sets the gravity manifold.
  /// \param manifold Input manifold.
  auto setGravityManifold(std::unique_ptr<GravityManifold>&& manifold) -> void;

  /// Sets the state manifold.
  /// \param manifold Input manifold.
  auto setStateManifold(std::unique_ptr<StateManifold>&& manifold) -> void;

  /// Sets a sensor manifold.
  /// \param manifold Input manifold.
  auto setSensorManifold(std::unique_ptr<SensorManifold>&& manifold) -> void;

  /// Sets the gravity constant.
  /// \param set_constant Constancy flag.
  auto setGravityConstant(bool set_constant) -> void final;

  /// Solves the optimization.
  auto optimize() -> void final;

 private:
  /// Checks that the input parameter blocks
  /// are present in the optimization problem.
  /// \param parameter_blocks Input parameter blocks.
  /// \return True if all parameter blocks are present.
  auto hasParameterBlocks(const Pointers<Scalar>& parameter_blocks) -> bool;

  /// State update.
  /// \param range Input range.
  auto updateState(const Range& range) -> void final;

  /// Adds a landmark.
  /// \param landmark Input landmark.
  auto addLandmark(Landmark<Position<Scalar>>& landmark) -> void final;

  /// Updates the landmarks.
  /// \param range Input range.
  auto updateLandmarks(const Range& range) -> void final;

  /// Sensor update.
  /// \param imu Input sensor.
  /// \param range Input range.
  auto updateSensor(IMU& imu, const Range& range) -> void final;

  ceres::Problem problem_;                                                    ///< Ceres problem.
  std::map<const Sensor*, std::unique_ptr<SensorManifold>> sensor_manifolds_; ///< Sensor manifolds.
  std::unique_ptr<GravityManifold> gravity_manifold_;                         ///< Gravity manifold.
  std::unique_ptr<StateManifold> state_manifold_;                             ///< State manifold.
};

using CeresOptimizer = Optimizer<OptimizerSuite::CERES>;

} // namespace hyper
