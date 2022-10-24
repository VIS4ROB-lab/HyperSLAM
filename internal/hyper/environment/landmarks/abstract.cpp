/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/environment/landmarks/abstract.hpp"
#include "hyper/environment/observations/abstract.hpp"
#include "hyper/messages/measurements/abstract.hpp"
#include "hyper/sensors/sensor.hpp"

namespace hyper {

auto AbstractLandmark::AbstractObservationCompare::operator()(const Stamp& lhs, const AbstractObservation* const rhs) const -> bool {
  return lhs < rhs->measurement().stamp();
}

auto AbstractLandmark::AbstractObservationCompare::operator()(const AbstractObservation* const lhs, const Stamp& rhs) const -> bool {
  return lhs->measurement().stamp() < rhs;
}

auto AbstractLandmark::AbstractObservationCompare::operator()(const AbstractObservation* const lhs, const AbstractObservation* const rhs) const -> bool {
  return lhs->measurement().stamp() < rhs->measurement().stamp();
}

auto AbstractLandmark::observations() const -> ObservationMap& {
  return observation_map_;
}

auto AbstractLandmark::observations(const Sensor& sensor) const -> ObservationSet& {
  const auto p_sensor = &sensor;
  DCHECK(observation_map_.contains(p_sensor));
  return observation_map_[p_sensor];
}

auto AbstractLandmark::numObservations() const -> Size {
  auto num_observations = Size{0};
  std::for_each(observation_map_.cbegin(), observation_map_.cend(), [&num_observations](const auto& itr) -> void { num_observations += itr.second.size(); });
  return num_observations;
}

auto AbstractLandmark::addObservation(const AbstractObservation& observation) const -> void {
  const auto p_sensor = &observation.measurement().sensor();
  const auto [_, inserted] = observation_map_[p_sensor].insert(&observation);
  DCHECK(inserted);
}

auto AbstractLandmark::removeObservation(const AbstractObservation& observation) const -> void {
  // Retrieve sensor.
  const auto& sensor = observation.measurement().sensor();

  // Erase observation.
  auto& set = observations(sensor);
  const auto erased_0 = set.erase(&observation);
  DCHECK(erased_0);

  // Erase set.
  if (set.empty()) {
    const auto p_sensor = &sensor;
    const auto erased_1 = observation_map_.erase(p_sensor);
    DCHECK(erased_1);
  }
}

auto AbstractLandmark::range() const -> Range<Stamp, BoundaryPolicy::INCLUSIVE> {
  // Sanity checks.
  DCHECK(!observation_map_.empty());

  // Retrieve entry.
  const auto itr_0 = observation_map_.cbegin();
  const auto& [sensor_0, observation_set_0] = *itr_0;

  // Initialize boundaries.
  DCHECK(!observation_set_0.empty());
  auto lower = (*observation_set_0.cbegin())->measurement().stamp();
  auto upper = (*observation_set_0.crbegin())->measurement().stamp();

  std::for_each(std::next(itr_0), observation_map_.cend(), [&lower, &upper](const auto& itr_i) -> void {
    // Retrieve entry.
    const auto& [sensor_i, observation_set_i] = itr_i;
    DCHECK(!observation_set_i.empty());

    // Update boundaries.
    const auto lower_i = (*observation_set_i.cbegin())->measurement().stamp();
    const auto upper_i = (*observation_set_i.crbegin())->measurement().stamp();
    lower = std::min(lower, lower_i);
    upper = std::max(upper, upper_i);
  });

  return {lower, upper};
}

auto AbstractLandmark::range(const Sensor& sensor) const -> Range<Stamp, BoundaryPolicy::INCLUSIVE> {
  // Retrieve observation set.
  const auto& observation_set = observations(sensor);
  DCHECK(!observation_set.empty());

  // Retrieve boundaries.
  const auto& lower = (*observation_set.cbegin())->measurement().stamp();
  const auto& upper = (*observation_set.crbegin())->measurement().stamp();
  return {lower, upper};
}

} // namespace hyper
