/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/landmarks/forward.hpp"

#include "hyper/environment/observations/abstract.hpp"

namespace hyper {

class LandmarkObservation
    : public AbstractObservation {
 public:
  /// Variables accessor.
  /// \return Variables.
  [[nodiscard]] auto variables() const -> Pointers<const AbstractVariable<Scalar>> final;

  /// Variables modifier.
  /// \return Variables.
  [[nodiscard]] auto variables() -> Pointers<AbstractVariable<Scalar>> final;

  /// Variables accessor.
  /// \param stamp Input stamp.
  /// \return Variables.
  [[nodiscard]] auto variables(const Stamp& stamp) const -> Pointers<const AbstractVariable<Scalar>> final;

  /// Variables modifier.
  /// \param stamp Input stamp.
  /// \return Variables.
  [[nodiscard]] auto variables(const Stamp& stamp) -> Pointers<AbstractVariable<Scalar>> final;

  /// Landmark accessor.
  /// \return Landmark.
  [[nodiscard]] virtual auto landmark() const -> const AbstractLandmark&;

  /// Landmark modifier.
  /// \return Landmark.
  [[nodiscard]] virtual auto landmark() -> AbstractLandmark&;

 protected:
  /// Constructor from landmark.
  /// \param landmark Landmark to use.
  explicit LandmarkObservation(AbstractLandmark& landmark);

  /// Sets the landmark.
  /// \param landmark Input landmark.
  auto setLandmark(AbstractLandmark& landmark) -> void;

  AbstractLandmark* landmark_; ///< Landmark.
};

} // namespace hyper
