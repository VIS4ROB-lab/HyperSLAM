/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/observations/abstract.hpp"
#include "hyper/messages/measurements/inertial.hpp"
#include "hyper/variables/gravity.hpp"

namespace hyper {

template <typename TManifold>
class InertialObservation final
    : public AbstractObservation {
 public:
  // Definitions.
  using Measurement = InertialMeasurement<TManifold>;

  /// Constructor from measurement and gravity.
  /// \param measurement Input measurement.
  /// \param gravity Input gravity.
  InertialObservation(const Measurement& measurement, Gravity<Scalar>& gravity)
      : measurement_{measurement},
        gravity_{&gravity} {}

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
    return {gravity_};
  }

  /// Variables modifier.
  /// \return Variables.
  [[nodiscard]] auto variables() -> Pointers<AbstractVariable<Scalar>> final {
    return {gravity_};
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

  /// Gravity accessor.
  /// \return Gravity.
  [[nodiscard]] auto gravity() const -> const Gravity<Scalar>& {
    return *gravity_;
  }

  /// Gravity modifier.
  /// \return Gravity.
  [[nodiscard]] auto gravity() -> Gravity<Scalar>& {
    return const_cast<Gravity<Scalar>&>(std::as_const(*this).gravity());
  }

  /// Sets the gravity.
  /// \param gravity Input gravity.
  auto setGravity(Gravity<Scalar>& gravity) -> void {
    gravity_ = &gravity;
  }

 private:
  Measurement measurement_;  ///< Measurement.
  Gravity<Scalar>* gravity_; ///< Gravity.
};

} // namespace hyper
