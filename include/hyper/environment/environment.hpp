/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/abstract.hpp"
#include "hyper/environment/observations/inertial.hpp"
#include "hyper/environment/observations/manifold.hpp"
#include "hyper/variables/gravity.hpp"

namespace hyper {

template <typename TManifold>
class Environment final
    : public AbstractEnvironment {
 public:
  /// Default constructor.
  Environment() : AbstractEnvironment{Traits<Environment>::kNumParameters} {
    parameters().setVariable(Traits<Environment>::kTransformationOffset, std::make_unique<TManifold>());
  }

  /// Transformation accessor.
  /// \return Transformation.
  [[nodiscard]] auto transformation() const -> const TManifold& {
    return static_cast<const TManifold&>(parameters().variable(Traits<Environment>::kTransformationOffset)); // NOLINT
  }

  /// Transformation modifier.
  /// \return Transformation.
  auto transformation() -> TManifold& {
    return const_cast<TManifold&>(std::as_const(*this).transformation());
  }

  /// Adds a measurement.
  /// \param measurement Input measurement.
  auto addManifoldMeasurement(const ManifoldMeasurement<TManifold>& measurement) -> ManifoldObservation<TManifold>& {
    // Find relevant observations.
    const auto p_sensor = &measurement.sensor();
    auto& observations = observations_[typeid(ManifoldMeasurement<TManifold>)][p_sensor];

    // Create observation.
    const auto observation_itr = observations.insert(std::make_unique<ManifoldObservation<TManifold>>(measurement));
    auto& observation = static_cast<ManifoldObservation<TManifold>&>(**observation_itr); // NOLINT

    // Return observation.
    return observation;
  }

  /// Adds a measurement.
  /// \param measurement Input measurement.
  auto addInertialMeasurement(const InertialMeasurement<TManifold>& measurement) -> InertialObservation<TManifold>& {
    // Find relevant observations.
    const auto p_sensor = &measurement.sensor();
    auto& observations = observations_[typeid(InertialMeasurement<TManifold>)][p_sensor];

    // Create observation.
    const auto observation_itr = observations.insert(std::make_unique<InertialObservation<TManifold>>(measurement, gravity()));
    auto& observation = static_cast<InertialObservation<TManifold>&>(**observation_itr); // NOLINT

    // Return observation.
    return observation;
  }
};

} // namespace hyper
