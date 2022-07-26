/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/environment/observations/visual.hpp"
#include "hyper/environment/landmarks/absolute.hpp"
#include "hyper/messages/measurements/visual.hpp"
#include "hyper/variables/bearing.hpp"
#include "hyper/variables/cartesian.hpp"

namespace hyper {

PixelObservation::PixelObservation(std::unique_ptr<Measurement>&& measurement, const PositionLandmark& landmark)
    : LandmarkObservation{std::move(measurement), landmark} {
}

auto PixelObservation::measurement() const -> const Measurement& {
  return static_cast<const Measurement&>(AbstractObservation::measurement()); // NOLINT
}

auto PixelObservation::measurement() -> Measurement& {
  return const_cast<Measurement&>(std::as_const(*this).measurement());
}

auto PixelObservation::landmark() const -> const PositionLandmark& {
  return static_cast<const PositionLandmark&>(LandmarkObservation::landmark()); // NOLINT
}

BearingObservation::BearingObservation(std::unique_ptr<Measurement>&& measurement, const PositionLandmark& landmark)
    : LandmarkObservation{std::move(measurement), landmark} {
}

auto BearingObservation::measurement() const -> const Measurement& {
  return static_cast<const Measurement&>(AbstractObservation::measurement()); // NOLINT
}

auto BearingObservation::measurement() -> Measurement& {
  return const_cast<Measurement&>(std::as_const(*this).measurement());
}

auto BearingObservation::landmark() const -> const PositionLandmark& {
  return static_cast<const PositionLandmark&>(LandmarkObservation::landmark()); // NOLINT
}

} // namespace hyper
