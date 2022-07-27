/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <glog/logging.h>

#include "hyper/environment/observations/abstract.hpp"
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
  InertialObservation(const Measurement& measurement, Scalar* gravity)
      : measurement_{measurement},
        gravity_{gravity} {
    DCHECK(gravity_ != nullptr);
  }

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

  /// Gravity accessor.
  /// \return Gravity.
  [[nodiscard]] auto gravity() const -> Eigen::Map<const Gravity<Scalar>> {
    return Eigen::Map<const Gravity<Scalar>>{gravity_};
  }

  /// Gravity modifier.
  /// \return Gravity.
  [[nodiscard]] auto gravity() -> Eigen::Map<Gravity<Scalar>> {
    return Eigen::Map<Gravity<Scalar>>{gravity_};
  }

 private:
  Measurement measurement_; ///< Measurement.
  Scalar* gravity_;         ///< Gravity.
};

} // namespace hyper
