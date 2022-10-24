/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <map>
#include <set>

#include "hyper/environment/landmarks/forward.hpp"
#include "hyper/environment/observations/forward.hpp"
#include "hyper/sensors/forward.hpp"

#include "hyper/definitions.hpp"
#include "hyper/range.hpp"
#include "hyper/variables/composite.hpp"

namespace hyper {

class AbstractLandmark {
 public:
  // Definitions.
  struct AbstractObservationCompare {
    using is_transparent = std::true_type;
    auto operator()(const Stamp& lhs, const AbstractObservation* rhs) const -> bool;
    auto operator()(const AbstractObservation* lhs, const Stamp& rhs) const -> bool;
    auto operator()(const AbstractObservation* lhs, const AbstractObservation* rhs) const -> bool;
  };

  using ObservationSet = std::set<const AbstractObservation*, AbstractObservationCompare>;
  using ObservationMap = std::map<const Sensor*, ObservationSet>;
  using Size = ObservationSet::size_type;

  /// Default destructor.
  virtual ~AbstractLandmark() = default;

  /// Variables accessor.
  /// \return Variables.
  [[nodiscard]] virtual auto variables() const -> Pointers<const AbstractVariable<Scalar>> = 0;

  /// Variables modifier.
  /// \return Variables.
  [[nodiscard]] virtual auto variables() -> Pointers<AbstractVariable<Scalar>> = 0;

  /// Variables accessor.
  /// \param stamp Input stamp.
  /// \return Variables.
  [[nodiscard]] virtual auto variables(const Stamp& stamp) const -> Pointers<const AbstractVariable<Scalar>> = 0;

  /// Variables modifier.
  /// \param stamp Input stamp.
  /// \return Variables.
  [[nodiscard]] virtual auto variables(const Stamp& stamp) -> Pointers<AbstractVariable<Scalar>> = 0;

  /// Observation accessor.
  /// \return Observations.
  auto observations() const -> ObservationMap&;

  /// Observation accessor.
  /// \return Observations.
  auto observations(const Sensor& sensor) const -> ObservationSet&;

  /// Retrieves the number of observations.
  /// \return Number of observations.
  auto numObservations() const -> Size;

  /// Adds an observation to this landmark.
  /// \param observation Observation to added.
  auto addObservation(const AbstractObservation& observation) const -> void;

  /// Removes an observation from this landmark.
  /// \param observation Observation to remove.
  auto removeObservation(const AbstractObservation& observation) const -> void;

  /// Retrieves the observation range.
  /// \return Observation range.
  auto range() const -> Range<Stamp, BoundaryPolicy::INCLUSIVE>;

  /// Retrieves the observation range.
  /// \param sensor Input sensor.
  /// \return Observation range.
  auto range(const Sensor& sensor) const -> Range<Stamp, BoundaryPolicy::INCLUSIVE>;

 protected:
  mutable ObservationMap observation_map_; ///< Observation map.
};

} // namespace hyper
