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

auto AbstractLandmark::observations() const -> const Observations& {
  return observations_;
}

auto AbstractLandmark::addObservation(const AbstractObservation& observation) const -> void {
  DCHECK(!observations_.contains(&observation));
  observations_.insert(&observation);
}

auto AbstractLandmark::removeObservation(const AbstractObservation& observation) const -> void {
  DCHECK(observations_.contains(&observation));
  observations_.erase(&observation);
}

auto AbstractLandmark::range() const -> Range<Stamp, BoundaryPolicy::INCLUSIVE> {
  const auto itr_begin = observations().begin();
  const auto itr_rbegin = observations().rbegin();
  return {(*itr_begin)->measurement().stamp(), (*itr_rbegin)->measurement().stamp()};
}

} // namespace hyper
