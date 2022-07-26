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

  /// Default destructor.
  ~LandmarkObservation() override;

  /// Landmark accessor.
  /// \return Landmark.
  [[nodiscard]] auto landmark() const -> const AbstractLandmark&;

  /// Collects the memory blocks.
  /// \return Memory blocks.
  [[nodiscard]] auto memoryBlocks() const -> MemoryBlocks<Scalar> final;

 protected:
  /// Constructor from measurement and landmark.
  /// \param measurement Measurement to use.
  /// \param landmark Landmark to use.
  LandmarkObservation(std::unique_ptr<AbstractMeasurement>&& measurement, const AbstractLandmark& landmark);

 private:
  const AbstractLandmark* landmark_; ///< Landmark.
};

} // namespace hyper
