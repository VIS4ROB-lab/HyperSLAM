/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <map>
#include <memory>
#include <set>
#include <typeindex>
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
 public:
  struct ObservationComparator {
    using is_transparent = std::true_type;
    auto operator()(const Stamp& lhs, const std::unique_ptr<AbstractObservation>& rhs) const -> bool;
    auto operator()(const std::unique_ptr<AbstractObservation>& lhs, const Stamp& rhs) const -> bool;
    auto operator()(const std::unique_ptr<AbstractObservation>& lhs, const std::unique_ptr<AbstractObservation>& rhs) const -> bool;
  };

  // Definitions.
  using Size = std::size_t;
  using Parameters = CompositeVariable<Scalar>;
  using Landmarks = std::map<std::type_index, std::unordered_map<Identifier, std::unique_ptr<AbstractLandmark>>>;
  using Observations = std::map<std::type_index, std::map<const Sensor*, std::multiset<std::unique_ptr<AbstractObservation>, ObservationComparator>>>;

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

  /// Adds a measurement and creates a landmark (if non-existent).
  /// \param identifier Input identifier.
  /// \param measurement Input measurement.
  /// \return Observation and insertion flag.
  auto addVisualMeasurement(const Identifier& identifier, const BearingMeasurement& measurement) -> std::tuple<VisualBearingObservation&, bool>;

  /// Adds a measurement and creates a landmark (if non-existent).
  /// \param identifier Input identifier.
  /// \param measurement Input measurement.
  /// \return Observation and insertion flag.
  auto addVisualMeasurement(const Identifier& identifier, const PixelMeasurement& measurement) -> std::tuple<VisualPixelObservation&, bool>;

 protected:
  /// Constructor from number of parameters.
  /// \param num_variables Number of parameters.
  explicit AbstractEnvironment(const Size& num_parameters);

  /// Parameters accessor.
  /// \return Parameters.
  auto parameters() -> Parameters&;

  /// Adds (or creates) a landmark.
  /// \tparam TLandmark Landmark type.
  /// \param identifier Input identifier.
  /// \return Landmark and insertion flag.
  template <typename TLandmark>
  auto addLandmark(const Identifier& identifier) -> std::pair<TLandmark&, bool> {
    const auto [itr, inserted] = landmarks_[typeid(TLandmark)].try_emplace(identifier, nullptr);
    if (inserted) itr->second = std::make_unique<TLandmark>();
    return {static_cast<TLandmark&>(*itr->second), inserted};
  }

  /// Adds an observation.
  /// \tparam TObservation Observation type.
  /// \param observation Input observation.
  /// \return Observation.
  template <typename TObservation>
  auto addObservation(std::unique_ptr<TObservation>&& observation) -> TObservation& {
    const auto itr = observations_[typeid(TObservation)][&observation->measurement().sensor()].insert(std::move(observation));
    return static_cast<TObservation&>(**itr);
  }

  /// Adds a visual measurement.
  /// \tparam TMeasurement_ Measurement type.
  /// \param identifier Input identifier.
  /// \param measurement Input measurement.
  /// \return Observation and insertion flag.
  template <typename TObservation_>
  auto addVisualMeasurement(const Identifier& identifier, const typename TObservation_::Measurement& measurement) -> std::tuple<TObservation_&, bool> {
    const auto [landmark, inserted] = addLandmark<typename TObservation_::Landmark>(identifier);
    auto& observation = addObservation(std::make_unique<TObservation_>(measurement, landmark));
    landmark.addObservation(observation);
    return {observation, inserted};
  }

  Parameters parameters_;     ///< Parameters.
  Landmarks landmarks_;       ///< Landmarks.
  Observations observations_; ///< Observations.
};

} // namespace hyper
