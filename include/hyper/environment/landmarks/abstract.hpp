/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

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
  struct ObservationCompare {
    using is_transparent = std::true_type;
    using Pivot = std::pair<Stamp, const Sensor*>;
    static auto ExtractPivot(const AbstractObservation* observation) -> Pivot;
    auto operator()(const Pivot& lhs, const AbstractObservation* rhs) const -> bool;
    auto operator()(const AbstractObservation* lhs, const Pivot& rhs) const -> bool;
    auto operator()(const AbstractObservation* lhs, const AbstractObservation* rhs) const -> bool;
  };

  using Parameters = std::vector<AbstractVariable<Scalar>*>;
  using ConstParameters = std::vector<AbstractVariable<const Scalar>*>;
  using Observations = std::set<const AbstractObservation*, ObservationCompare>;

  /// Default destructor.
  virtual ~AbstractLandmark() = default;

  /// Parameters accessor.
  /// \return Parameters.
  [[nodiscard]] virtual auto parameters() const -> ConstParameters = 0;

  /// Parameters modifier.
  /// \return Parameters.
  [[nodiscard]] virtual auto parameters() -> Parameters = 0;

  /// Accessor for the landmark observations.
  /// \return Constant reference to associated observations.
  auto observations() const -> const Observations&;

  /// Adds an observation to this landmark.
  /// \param observation Observation to added.
  auto addObservation(const AbstractObservation& observation) const -> void;

  /// Removes an observation from this landmark.
  /// \param observation Observation to remove.
  auto removeObservation(const AbstractObservation& observation) const -> void;

  /// Retrieves the observation range.
  /// \return Observation range.
  auto range() const -> Range<Stamp, BoundaryPolicy::INCLUSIVE>;

 protected:
  /// Default constructor.
  AbstractLandmark() = default;

  mutable Observations observations_; ///< Observations.
};

} // namespace hyper
