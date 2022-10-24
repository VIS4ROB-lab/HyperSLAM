/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/observations/forward.hpp"
#include "hyper/messages/measurements/forward.hpp"
#include "hyper/variables/forward.hpp"

#include "hyper/definitions.hpp"

namespace hyper {

class AbstractObservation {
 public:
  /// Virtual default destructor.
  virtual ~AbstractObservation() = default;

  /// Measurement accessor.
  /// \return Measurement.
  [[nodiscard]] virtual auto measurement() const -> const AbstractMeasurement& = 0;

  /// Measurement modifier.
  /// \return Measurement.
  [[nodiscard]] virtual auto measurement() -> AbstractMeasurement& = 0;

  /// Variables accessor.
  /// \return Variables.
  [[nodiscard]] virtual auto variables() const -> Pointers<const AbstractVariable<Scalar>> = 0;

  /// Variables modifier.
  /// \return Variables.
  [[nodiscard]] virtual auto variables() -> Pointers<AbstractVariable<Scalar>> = 0;

  /// Variables accessor.
  /// \param stamp Input stamp.
  /// \return Variables.
  [[nodiscard]] virtual auto variables(const Stamp& stamp) const -> Pointers<const AbstractVariable<Scalar>> = 0;

  /// Variables modifier.
  /// \param stamp Input stamp.
  /// \return Variables.
  [[nodiscard]] virtual auto variables(const Stamp& stamp) -> Pointers<AbstractVariable<Scalar>> = 0;
};

} // namespace hyper
