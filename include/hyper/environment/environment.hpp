/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/abstract.hpp"

namespace hyper {

template <typename TManifold>
class Environment final
    : public AbstractEnvironment {
 public:
  /// Default constructor.
  Environment() : AbstractEnvironment{Traits<Environment>::kNumParameters} {
    parameters().setVariable(Traits<Environment>::kTransformationOffset, std::make_unique<TManifold>());
  }

  /// Default destructor.
  ~Environment() final = default;

  /// Transformation accessor.
  /// \return Transformation.
  [[nodiscard]] auto transformation() const -> Eigen::Map<const TManifold> {
    auto address = parameters().variable(Traits<Environment>::kTransformationOffset).memory().address;
    return Eigen::Map<const TManifold>{address};
  }

  /// Transformation modifier.
  /// \return Transformation.
  auto transformation() -> Eigen::Map<TManifold> {
    auto address = parameters().variable(Traits<Environment>::kTransformationOffset).memory().address;
    return Eigen::Map<TManifold>{address};
  }
};

} // namespace hyper
