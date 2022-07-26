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
 private:
  struct ObservationCompare {
    using is_transparent = std::true_type;
    using Pivot = std::pair<Stamp, const Sensor*>;
    static auto ExtractPivot(const AbstractObservation* observation) -> Pivot;
    auto operator()(const Pivot& lhs, const AbstractObservation* rhs) const -> bool;
    auto operator()(const AbstractObservation* lhs, const Pivot& rhs) const -> bool;
    auto operator()(const AbstractObservation* lhs, const AbstractObservation* rhs) const -> bool;
  };

 public:
  using Size = std::size_t;
  using Parameters = CompositeVariable<Scalar>;
  using Observations = std::set<const AbstractObservation*, ObservationCompare>;

  /// Default destructor.
  virtual ~AbstractLandmark() = default;

  /// Parameters accessor.
  /// \return Parameters.
  [[nodiscard]] auto parameters() const -> const Parameters&;

  /// Accessor for the landmark observations.
  /// \return Constant reference to associated observations.
  auto observations() const -> const Observations&;

  /// Retrieves the observation range.
  /// \return Observation range.
  auto range() const -> Range<Stamp, BoundaryPolicy::INCLUSIVE>;

 protected:
  /// Constructor from number of parameters.
  /// \param num_variables Number of parameters.
  explicit AbstractLandmark(const Size& num_parameters);

  Parameters parameters_;             ///< Parameters.
  mutable Observations observations_; ///< Observations.
};

} // namespace hyper
