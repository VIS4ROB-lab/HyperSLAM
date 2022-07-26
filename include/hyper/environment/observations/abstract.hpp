/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <memory>
#include <vector>

#include "hyper/environment/observations/forward.hpp"
#include "hyper/messages/measurements/forward.hpp"
#include "hyper/sensors/forward.hpp"
#include "hyper/variables/forward.hpp"

#include "hyper/definitions.hpp"

namespace hyper {

class AbstractObservation {
 public:
  /// Deleted default constructor.
  AbstractObservation() = delete;

  /// Virtual default destructor.
  virtual ~AbstractObservation();

  /// Measurement accessor.
  /// \return Measurement.
  [[nodiscard]] auto measurement() const -> const AbstractMeasurement&;

  /// Measurement modifier.
  /// \return Measurement.
  [[nodiscard]] auto measurement() -> AbstractMeasurement&;

  /// Collects the memory blocks.
  /// \return Memory blocks.
  [[nodiscard]] virtual auto memoryBlocks() const -> MemoryBlocks<Scalar>;

 protected:
  /// Constructor from measurement.
  /// \param measurement Measurement to use.
  explicit AbstractObservation(std::unique_ptr<AbstractMeasurement>&& measurement);

  std::unique_ptr<AbstractMeasurement> measurement_; ///< Measurement.
};

} // namespace hyper
