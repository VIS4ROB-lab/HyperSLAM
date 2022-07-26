/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/environment/landmarks/absolute.hpp"
#include "hyper/environment/observations/visual.hpp"
#include "hyper/variables/cartesian.hpp"

namespace hyper {

PositionLandmark::PositionLandmark()
    : AbstractLandmark{Traits<PositionLandmark>::kNumParameterBlocks} {
  parameters_.setVariable(Traits<PositionLandmark>::kPositionOffset, std::make_unique<Position<Scalar>>());
}

PositionLandmark::~PositionLandmark() = default;

auto PositionLandmark::addObservation(const BearingObservation& observation) const -> void {
  observations_.emplace(&observation);
}

auto PositionLandmark::position() const -> Eigen::Map<const Position<Scalar>> {
  auto address = parameters().variable(Traits<PositionLandmark>::kPositionOffset).memory().address;
  return Eigen::Map<const Position<Scalar>>{address};
}

auto PositionLandmark::position() -> Eigen::Map<Position<Scalar>> {
  auto address = parameters().variable(Traits<PositionLandmark>::kPositionOffset).memory().address;
  return Eigen::Map<Position<Scalar>>{address};
}

} // namespace hyper
