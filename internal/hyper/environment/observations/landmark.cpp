/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/environment/landmarks/abstract.hpp"
#include "hyper/environment/observations/landmark.hpp"

namespace hyper {

LandmarkObservation::~LandmarkObservation() = default;

auto LandmarkObservation::landmark() const -> const AbstractLandmark& {
  DCHECK(landmark_ != nullptr);
  return *landmark_;
}

auto LandmarkObservation::memoryBlocks() const -> MemoryBlocks<Scalar> {
  return landmark().parameters().memoryBlocks();
}

LandmarkObservation::LandmarkObservation(std::unique_ptr<AbstractMeasurement>&& measurement, const AbstractLandmark& landmark)
    : AbstractObservation{std::move(measurement)},
      landmark_{&landmark} {
}

} // namespace hyper
