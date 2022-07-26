/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.#pragma once

#pragma once

#include "hyper/variables/forward.hpp"

#include "hyper/environment/landmarks/abstract.hpp"

namespace hyper {

class PositionLandmark final
    : public AbstractLandmark {
 public:
  /// Default constructor.
  PositionLandmark();

  /// Constructor from position.
  /// \tparam TDerived Derived type.
  /// \param derived Input position.
  template <typename TDerived>
  PositionLandmark(const Eigen::MatrixBase<TDerived>& derived) // NOLINT
      : PositionLandmark{} {
    position() = derived;
  }

  /// Virtual default destructor.
  ~PositionLandmark() final;

  /// Adds an observation to this landmark.
  /// \param observation Observation to be added.
  auto addObservation(const BearingObservation& observation) const -> void;

  /// Position accessor.
  /// \return Landmark position.
  [[nodiscard]] auto position() const -> Eigen::Map<const Position<Scalar>>;

  /// Position modifier.
  /// \return Landmark position.
  auto position() -> Eigen::Map<Position<Scalar>>;
};

} // namespace hyper
