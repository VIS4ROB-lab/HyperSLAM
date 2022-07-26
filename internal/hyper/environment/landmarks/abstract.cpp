/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/environment/landmarks/abstract.hpp"
#include "hyper/environment/observations/abstract.hpp"
#include "hyper/messages/measurements/abstract.hpp"
#include "hyper/sensors/sensor.hpp"

namespace hyper {

auto AbstractLandmark::ObservationCompare::ExtractPivot(const AbstractObservation* const observation) -> Pivot {
  const auto& measurement = observation->measurement();
  return {measurement.stamp(), &measurement.sensor()};
}

auto AbstractLandmark::ObservationCompare::operator()(const Pivot& lhs, const AbstractObservation* const rhs) const -> bool {
  return lhs < ExtractPivot(rhs);
}

auto AbstractLandmark::ObservationCompare::operator()(const AbstractObservation* const lhs, const Pivot& rhs) const -> bool {
  return ExtractPivot(lhs) < rhs;
}

auto AbstractLandmark::ObservationCompare::operator()(const AbstractObservation* const lhs, const AbstractObservation* const rhs) const -> bool {
  return ExtractPivot(lhs) < ExtractPivot(rhs);
}

auto AbstractLandmark::parameters() const -> const Parameters& {
  return parameters_;
}

auto AbstractLandmark::observations() const -> const Observations& {
  return observations_;
}

auto AbstractLandmark::range() const -> Range<Stamp, BoundaryPolicy::INCLUSIVE> {
  return {(*observations_.begin())->measurement().stamp(), (*observations_.rbegin())->measurement().stamp()};
}

AbstractLandmark::AbstractLandmark(const Size& num_parameters)
    : parameters_{num_parameters},
      observations_{} {}

} // namespace hyper
