/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/environment/observations/abstract.hpp"
#include "hyper/messages/measurements/abstract.hpp"

namespace hyper {

AbstractObservation::~AbstractObservation() = default;

auto AbstractObservation::measurement() const -> const AbstractMeasurement& {
  DCHECK(measurement_ != nullptr);
  return *measurement_;
}

auto AbstractObservation::measurement() -> AbstractMeasurement& {
  return const_cast<AbstractMeasurement&>(std::as_const(*this).measurement());
}

auto AbstractObservation::memoryBlocks() const -> MemoryBlocks<Scalar> {
  return {};
}

AbstractObservation::AbstractObservation(std::unique_ptr<AbstractMeasurement>&& measurement)
    : measurement_{std::move(measurement)} {
}

} // namespace hyper
