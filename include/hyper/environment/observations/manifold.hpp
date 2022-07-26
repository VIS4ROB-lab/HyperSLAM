/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/observations/abstract.hpp"

namespace hyper {

template <typename TManifold>
class ManifoldObservation final
    : public AbstractObservation {
 public:
  // Definitions.
  using Measurement = ManifoldMeasurement<TManifold>;

  /// Constructor from measurement.
  /// \param measurement Measurement to use.
  explicit ManifoldObservation(std::unique_ptr<Measurement>&& measurement)
      : AbstractObservation{std::move(measurement)} {}

  /// Default destructor.
  ~ManifoldObservation() final = default;

  /// Measurement accessor.
  /// \return Measurement.
  [[nodiscard]] auto measurement() const -> const Measurement& {
    return static_cast<const Measurement&>(AbstractObservation::measurement()); // NOLINT
  }

  /// Measurement modifier.
  /// \return Measurement.
  [[nodiscard]] auto measurement() -> Measurement& {
    return const_cast<Measurement&>(std::as_const(*this).measurement());
  }
};

} // namespace hyper
