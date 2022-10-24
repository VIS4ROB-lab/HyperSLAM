/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/observations/abstract.hpp"
#include "hyper/messages/measurements/variable.hpp"

namespace hyper {

template <typename TManifold>
class ManifoldObservation final
    : public AbstractObservation {
 public:
  // Definitions.
  using Measurement = ManifoldMeasurement<TManifold>;

  /// Constructor from measurement.
  /// \param measurement Input measurement.
  explicit ManifoldObservation(const Measurement& measurement)
      : measurement_{measurement} {}

  /// Measurement accessor.
  /// \return Measurement.
  [[nodiscard]] auto measurement() const -> const Measurement& final {
    return measurement_;
  }

  /// Measurement modifier.
  /// \return Measurement.
  [[nodiscard]] auto measurement() -> Measurement& final {
    return const_cast<Measurement&>(std::as_const(*this).measurement());
  }

  /// Variables accessor.
  /// \return Variables.
  [[nodiscard]] auto variables() const -> Pointers<const AbstractVariable<Scalar>> final {
    return {};
  }

  /// Variables modifier.
  /// \return Variables.
  [[nodiscard]] auto variables() -> Pointers<AbstractVariable<Scalar>> final {
    return {};
  }

  /// Variables accessor.
  /// \param stamp Input stamp.
  /// \return Variables.
  [[nodiscard]] auto variables(const Stamp& /* stamp */) const -> Pointers<const AbstractVariable<Scalar>> final {
    return variables();
  }

  /// Variables modifier.
  /// \param stamp Input stamp.
  /// \return Variables.
  [[nodiscard]] auto variables(const Stamp& /* stamp */) -> Pointers<AbstractVariable<Scalar>> final {
    return variables();
  }

 private:
  Measurement measurement_; ///< Measurement.
};

} // namespace hyper
