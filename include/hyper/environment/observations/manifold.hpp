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

  /// Parameters accessor.
  /// \return Parameters.
  [[nodiscard]] auto parameters() const -> ConstParameters final {
    return {};
  }

  /// Parameters modifier.
  /// \return Parameters.
  [[nodiscard]] auto parameters() -> Parameters final {
    return {};
  }

 private:
  Measurement measurement_; ///< Measurement.
};

} // namespace hyper
