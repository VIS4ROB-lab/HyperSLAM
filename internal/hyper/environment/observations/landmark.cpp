/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/environment/observations/landmark.hpp"
#include "hyper/environment/landmarks/abstract.hpp"

namespace hyper {

auto LandmarkObservation::variables() const -> Pointers<const AbstractVariable<Scalar>> {
  return landmark().variables();
}

auto LandmarkObservation::variables() -> Pointers<AbstractVariable<Scalar>> {
  return landmark().variables();
}

auto LandmarkObservation::variables(const Stamp& stamp) const -> Pointers<const AbstractVariable<Scalar>> {
  return landmark().variables(stamp);
}

auto LandmarkObservation::variables(const Stamp& stamp) -> Pointers<AbstractVariable<Scalar>> {
  return landmark().variables(stamp);
}

auto LandmarkObservation::landmark() const -> const AbstractLandmark& {
  return *landmark_;
}

auto LandmarkObservation::landmark() -> AbstractLandmark& {
  return const_cast<AbstractLandmark&>(std::as_const(*this).landmark());
}

LandmarkObservation::LandmarkObservation(AbstractLandmark& landmark)
    : landmark_{&landmark} {}

auto LandmarkObservation::setLandmark(AbstractLandmark& landmark) -> void {
  landmark_ = &landmark;
}

} // namespace hyper
