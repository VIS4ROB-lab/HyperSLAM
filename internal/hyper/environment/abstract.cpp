/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/environment/abstract.hpp"
#include "hyper/environment/landmarks/variable.hpp"
#include "hyper/environment/observations/abstract.hpp"
#include "hyper/environment/observations/visual.hpp"
#include "hyper/messages/measurements/abstract.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/variables/bearing.hpp"
#include "hyper/variables/gravity.hpp"

namespace hyper {

auto AbstractEnvironment::ObservationComparator::operator()(const Stamp& lhs, const std::unique_ptr<AbstractObservation>& rhs) const -> bool {
  return lhs < rhs->measurement().stamp();
}

auto AbstractEnvironment::ObservationComparator::operator()(const std::unique_ptr<AbstractObservation>& lhs, const Stamp& rhs) const -> bool {
  return lhs->measurement().stamp() < rhs;
}

auto AbstractEnvironment::ObservationComparator::operator()(const std::unique_ptr<AbstractObservation>& lhs, const std::unique_ptr<AbstractObservation>& rhs) const -> bool {
  return lhs->measurement().stamp() < rhs->measurement().stamp();
}

auto AbstractEnvironment::parameters() const -> const Parameters& {
  return parameters_;
}

auto AbstractEnvironment::gravity() const -> Eigen::Map<const Gravity<Scalar>> {
  auto address = parameters().variable(Traits<AbstractEnvironment>::kGravityOffset).memory().address;
  return Eigen::Map<const Gravity<Scalar>>{address};
}

auto AbstractEnvironment::gravity() -> Eigen::Map<Gravity<Scalar>> {
  auto address = parameters().variable(Traits<AbstractEnvironment>::kGravityOffset).memory().address;
  return Eigen::Map<Gravity<Scalar>>{address};
}

auto AbstractEnvironment::landmarks() const -> const Landmarks& {
  return landmarks_;
}

auto AbstractEnvironment::landmarks() -> Landmarks& {
  return const_cast<Landmarks&>(std::as_const(*this).landmarks());
}

auto AbstractEnvironment::observations() const -> const Observations& {
  return observations_;
}

auto AbstractEnvironment::addObservation(std::unique_ptr<AbstractObservation>&& observation) -> void {
  // Retrieve or create sensor entry.
  auto sensor_ptr = &observation->measurement().sensor();
  auto sensor_itr = observations_.find(sensor_ptr);
  if (sensor_itr == observations_.cend()) {
    sensor_itr = observations_.try_emplace(sensor_itr, sensor_ptr, ObservationSet{});
  }

  // Add new observation.
  sensor_itr->second.insert(std::move(observation));
}

auto AbstractEnvironment::addObservation(std::unique_ptr<BearingObservation>&& observation) -> void {
  // Link observation to landmark.
  observation->landmark().addObservation(*observation);

  // Retrieve or create sensor entry.
  auto sensor_ptr = &observation->measurement().sensor();
  auto sensor_itr = observations_.find(sensor_ptr);
  if (sensor_itr == observations_.cend()) {
    sensor_itr = observations_.try_emplace(sensor_itr, sensor_ptr, ObservationSet{});
  }

  // Add new observation.
  sensor_itr->second.insert(std::move(observation));
}

AbstractEnvironment::AbstractEnvironment(const Size& num_parameters)
    : parameters_{num_parameters},
      landmarks_{},
      observations_{} {
  DCHECK_LE(Traits<AbstractEnvironment>::kNumParameters, parameters().variables().size());
  parameters().setVariable(Traits<AbstractEnvironment>::kGravityOffset, std::make_unique<Gravity<Scalar>>(-Traits<Gravity<Scalar>>::kNorm, Scalar{0}, Scalar{0}));
}

auto AbstractEnvironment::parameters() -> Parameters& {
  return const_cast<Parameters&>(std::as_const(*this).parameters());
}

} // namespace hyper
