/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/environment/abstract.hpp"
#include "hyper/environment/landmarks/variable.hpp"
#include "hyper/environment/observations/abstract.hpp"
#include "hyper/environment/observations/visual.hpp"
#include "hyper/messages/measurements/abstract.hpp"
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

auto AbstractEnvironment::gravity() const -> const Gravity<Scalar>& {
  return static_cast<const Gravity<Scalar>&>(parameters().variable(Traits<AbstractEnvironment>::kGravityOffset)); // NOLINT
}

auto AbstractEnvironment::gravity() -> Gravity<Scalar>& {
  return const_cast<Gravity<Scalar>&>(std::as_const(*this).gravity());
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

auto AbstractEnvironment::addVisualMeasurement(const Identifier& identifier, const BearingMeasurement& measurement) -> std::tuple<VisualBearingObservation&, bool> {
  return addVisualMeasurement<VisualBearingObservation>(identifier, measurement);
}

auto AbstractEnvironment::addVisualMeasurement(const Identifier& identifier, const PixelMeasurement& measurement) -> std::tuple<VisualPixelObservation&, bool> {
  return addVisualMeasurement<VisualPixelObservation>(identifier, measurement);
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
