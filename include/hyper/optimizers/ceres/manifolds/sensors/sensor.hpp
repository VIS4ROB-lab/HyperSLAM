/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <ceres/manifold.h>

#include "hyper/optimizers/ceres/manifolds/sensors/forward.hpp"

#include "hyper/definitions.hpp"
#include "hyper/sensors/sensor.hpp"

namespace hyper {

template <>
class Manifold<Sensor, OptimizerSuite::CERES> {
 public:
  /// Constructor from sensor and constancy flag.
  /// \param sensor Input sensor.
  /// \param constant Constancy flag.
  explicit Manifold(const Sensor& sensor, bool constant = true);

  /// Default destructor.
  virtual ~Manifold() = default;

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] virtual auto sensor() const -> const Sensor&;

  /// Transformation manifold accessor.
  /// \return Transformation manifold.
  [[nodiscard]] auto transformationManifold() const -> ceres::Manifold*;

  /// Sets the transformation manifold.
  /// \param manifold Input manifold.
  auto setTransformationManifold(std::unique_ptr<ceres::Manifold>&& manifold) -> void;

  /// Sets the transformation manifold constant.
  /// \param constant Constancy flag.
  auto setTransformationConstant(bool constant) -> void;

  /// Retrieves all manifolds.
  /// \return Manifolds.
  [[nodiscard]] virtual auto manifolds() const -> Pointers<ceres::Manifold>;

  /// Retrieves all manifold (stamp-based).
  /// \param stamp Query stamp.
  /// \return Manifolds.
  [[nodiscard]] virtual auto manifolds(const Stamp& stamp) const -> Pointers<ceres::Manifold>;

 protected:
  // Definitions.
  using Size = std::size_t;
  using Manifolds = std::vector<std::unique_ptr<ceres::Manifold>>;

  /// Constructor from sensor, number of manifolds and constancy flag.
  /// \param sensor Input sensor.
  /// \param num_manifolds Number of manifolds.
  /// \param constant Constancy flag.
  Manifold(const Sensor& sensor, const Size& num_manifolds, bool constant);

  /// Retrieves a manifold at index.
  /// \param index Query index.
  /// \return Manifold.
  [[nodiscard]] auto manifold(const Size& index) const -> ceres::Manifold*;

  /// Sets a manifold at index.
  /// \param index Input index.
  /// \param manifold Input manifold.
  auto setManifold(const Size& index, std::unique_ptr<ceres::Manifold>&& manifold) -> void;

  const Sensor* sensor_; ///< Sensor.
  Manifolds manifolds_;  ///< Manifolds.
};

} // namespace hyper
