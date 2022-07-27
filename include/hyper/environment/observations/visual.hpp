/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/landmarks/variable.hpp"
#include "hyper/environment/observations/landmark.hpp"
#include "hyper/messages/measurements/visual.hpp"

namespace hyper {

template <typename TMeasurement>
class VisualObservation final
    : public LandmarkObservation {
 public:
  /// Constructor from measurement and landmark.
  /// \param measurement Measurement to use.
  /// \param landmark Landmark to use.
  VisualObservation(const TMeasurement& measurement, PositionLandmark& landmark)
      : LandmarkObservation{landmark},
        measurement_{measurement} {}

  /// Measurement accessor.
  /// \return Measurement.
  [[nodiscard]] auto measurement() const -> const TMeasurement& final {
    return measurement_;
  }

  /// Measurement modifier.
  /// \return Measurement.
  [[nodiscard]] auto measurement() -> TMeasurement& final {
    return const_cast<TMeasurement&>(std::as_const(*this).measurement());
  }

  /// Landmark accessor.
  /// \return Landmark.
  [[nodiscard]] auto landmark() const -> const PositionLandmark& final {
    DCHECK(landmark_ != nullptr);
    return static_cast<const PositionLandmark&>(*landmark_); // NOLINT
  }

  /// Landmark accessor.
  /// \return Landmark.
  [[nodiscard]] auto landmark() -> PositionLandmark& final {
    return const_cast<PositionLandmark&>(std::as_const(*this).landmark());
  }

 private:
  TMeasurement measurement_; ///< Measurement.
};

using PixelObservation = VisualObservation<PixelMeasurement>;
using BearingObservation = VisualObservation<BearingMeasurement>;

} // namespace hyper
