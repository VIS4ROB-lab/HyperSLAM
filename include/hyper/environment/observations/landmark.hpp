/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/landmarks/forward.hpp"

#include "hyper/environment/observations/abstract.hpp"

namespace hyper {

class LandmarkObservation
    : public AbstractObservation {
 public:
  /// Deleted default constructor.
  LandmarkObservation() = delete;

  /// Parameters accessor.
  /// \return Parameters.
  [[nodiscard]] auto parameters() const -> ConstParameters final;

  /// Parameters modifier.
  /// \return Parameters.
  [[nodiscard]] auto parameters() -> Parameters final;

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

  AbstractLandmark* landmark_; ///< Landmark.
};

} // namespace hyper
