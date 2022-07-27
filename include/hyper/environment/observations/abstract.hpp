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
  using Parameters = std::vector<AbstractVariable<Scalar>*>;
  using ConstParameters = std::vector<const AbstractVariable<Scalar>*>;

  /// Virtual default destructor.
  virtual ~AbstractObservation() = default;

  /// Measurement accessor.
  /// \return Measurement.
  [[nodiscard]] virtual auto measurement() const -> const AbstractMeasurement& = 0;

  /// Measurement modifier.
  /// \return Measurement.
  [[nodiscard]] virtual auto measurement() -> AbstractMeasurement& = 0;

  /// Parameters accessor.
  /// \return Parameters.
  [[nodiscard]] virtual auto parameters() const -> ConstParameters = 0;

  /// Parameters modifier.
  /// \return Parameters.
  [[nodiscard]] virtual auto parameters() -> Parameters = 0;
};

} // namespace hyper
