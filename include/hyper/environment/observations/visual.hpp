/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/observations/landmark.hpp"
#include "hyper/messages/measurements/visual.hpp"

namespace hyper {

class PixelObservation final
    : public LandmarkObservation {
 public:
  // Definitions.
  using Measurement = PixelMeasurement;

  /// Constructor from measurement and landmark.
  /// \param measurement Measurement to use.
  /// \param landmark Landmark to use.
  PixelObservation(std::unique_ptr<Measurement>&& measurement, const PositionLandmark& landmark);

  /// Default destructor.
  ~PixelObservation() final = default;

  /// Measurement accessor.
  /// \return Measurement.
  [[nodiscard]] auto measurement() const -> const Measurement&;

  /// Measurement modifier.
  /// \return Measurement.
  [[nodiscard]] auto measurement() -> Measurement&;

  /// Retrieves the associated landmark.
  /// \return Landmark.
  [[nodiscard]] auto landmark() const -> const PositionLandmark&;
};

class BearingObservation final
    : public LandmarkObservation {
 public:
  // Definitions.
  using Measurement = BearingMeasurement;

  /// Constructor from measurement and landmark.
  /// \param measurement Measurement to use.
  /// \param landmark Landmark to use.
  BearingObservation(std::unique_ptr<Measurement>&& measurement, const PositionLandmark& landmark);

  /// Default destructor.
  ~BearingObservation() final = default;

  /// Measurement accessor.
  /// \return Measurement.
  [[nodiscard]] auto measurement() const -> const Measurement&;

  /// Measurement modifier.
  /// \return Measurement.
  [[nodiscard]] auto measurement() -> Measurement&;

  /// Retrieves the associated landmark.
  /// \return Landmark.
  [[nodiscard]] auto landmark() const -> const PositionLandmark&;
};

} // namespace hyper
