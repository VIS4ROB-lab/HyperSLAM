/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/landmarks/variable.hpp"
#include "hyper/environment/observations/landmark.hpp"
#include "hyper/messages/measurements/visual.hpp"

namespace hyper {

template <typename TMeasurement, typename TLandmark>
class VisualObservation final
    : public LandmarkObservation {
 public:
  // Definitions.
  using Measurement = TMeasurement;
  using Landmark = TLandmark;

  /// Constructor from measurement and landmark.
  /// \param measurement Input measurement.
  /// \param landmark Input landmark.
  VisualObservation(const Measurement& measurement, Landmark& landmark)
      : LandmarkObservation{landmark},
        measurement_{measurement} {}

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

  /// Landmark accessor.
  /// \return Landmark.
  [[nodiscard]] auto landmark() const -> const Landmark& final {
    DCHECK(landmark_ != nullptr);
    return static_cast<const Landmark&>(*landmark_); // NOLINT
  }

  /// Landmark accessor.
  /// \return Landmark.
  [[nodiscard]] auto landmark() -> Landmark& final {
    return const_cast<Landmark&>(std::as_const(*this).landmark());
  }

 private:
  Measurement measurement_; ///< Measurement.
};

using VisualPixelObservation = VisualObservation<PixelMeasurement, Landmark<Position<Scalar>>>;

using VisualBearingObservation = VisualObservation<BearingMeasurement, Landmark<Position<Scalar>>>;

} // namespace hyper
