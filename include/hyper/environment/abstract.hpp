/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <map>
#include <memory>
#include <set>
#include <unordered_map>

#include "hyper/environment/forward.hpp"
#include "hyper/sensors/forward.hpp"
#include "hyper/variables/forward.hpp"

#include "hyper/definitions.hpp"
#include "hyper/environment/landmarks/abstract.hpp"
#include "hyper/environment/observations/visual.hpp"
#include "hyper/variables/composite.hpp"

namespace hyper {

class AbstractEnvironment {
 private:
  struct ObservationComparator {
    using is_transparent = std::true_type;
    auto operator()(const Stamp& lhs, const std::unique_ptr<AbstractObservation>& rhs) const -> bool;
    auto operator()(const std::unique_ptr<AbstractObservation>& lhs, const Stamp& rhs) const -> bool;
    auto operator()(const std::unique_ptr<AbstractObservation>& lhs, const std::unique_ptr<AbstractObservation>& rhs) const -> bool;
  };

 public:
  using Size = std::size_t;
  using Parameters = CompositeVariable<Scalar>;
  using Landmarks = std::unordered_map<Identifier, std::unique_ptr<AbstractLandmark>>;
  using ObservationSet = std::multiset<std::unique_ptr<AbstractObservation>, ObservationComparator>;
  using Observations = std::map<const Sensor*, ObservationSet, std::less<>>;

  /// Default destructor.
  virtual ~AbstractEnvironment() = default;

  /// Parameters accessor.
  /// \return Parameters.
  [[nodiscard]] auto parameters() const -> const Parameters&;

  /// Gravity accessor.
  /// \return Gravity.
  [[nodiscard]] auto gravity() const -> const Gravity<Scalar>&;

  /// Gravity modifier.
  /// \return Gravity.
  auto gravity() -> Gravity<Scalar>&;

  /// Landmark accessor.
  /// \return Landmarks.
  [[nodiscard]] auto landmarks() const -> const Landmarks&;

  /// Landmark modifier.
  /// \return Landmarks.
  [[nodiscard]] auto landmarks() -> Landmarks&;

  /// Observation accessor.
  /// \return Observations.
  [[nodiscard]] auto observations() const -> const Observations&;

  /// Adds an observation (without side effects).
  /// \param observation Input observation.
  auto addObservation(std::unique_ptr<AbstractObservation>&& observation) -> void;

  /// Adds an observation (with specialized side effects).
  /// \param observation Input observation.
  auto addObservation(std::unique_ptr<BearingObservation>&& observation) -> void;

 protected:
  /// Constructor from number of parameters.
  /// \param num_variables Number of parameters.
  explicit AbstractEnvironment(const Size& num_parameters);

  /// Parameters accessor.
  /// \return Parameters.
  auto parameters() -> Parameters&;

 private:
  Parameters parameters_;     ///< Parameters.
  Landmarks landmarks_;       ///< Landmarks.
  Observations observations_; ///< Observations.
};

} // namespace hyper
