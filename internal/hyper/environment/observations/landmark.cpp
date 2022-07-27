/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/environment/landmarks/abstract.hpp"
#include "hyper/environment/observations/landmark.hpp"

namespace hyper {

auto LandmarkObservation::parameters() const -> ConstParameters {
  return landmark().parameters();
}

auto LandmarkObservation::parameters() -> Parameters {
  return landmark().parameters();
}

auto LandmarkObservation::landmark() const -> const AbstractLandmark& {
  DCHECK(landmark_ != nullptr);
  return *landmark_;
}

auto LandmarkObservation::landmark() -> AbstractLandmark& {
  return const_cast<AbstractLandmark&>(std::as_const(*this).landmark());
}

LandmarkObservation::LandmarkObservation(AbstractLandmark& landmark)
    : landmark_{&landmark} {}

} // namespace hyper
