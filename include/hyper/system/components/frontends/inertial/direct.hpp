/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/system/components/frontends/inertial/forward.hpp"

#include "hyper/system/components/frontends/abstract.hpp"

namespace hyper {

template <typename TManifold>
class InertialFrontend<TManifold, InertialFrontendType::DIRECT> final
    : public AbstractFrontend {
 public:
  using Manifold = TManifold;

  /// Constructor from YAML node.
  /// \param node Input YAML node.
  explicit InertialFrontend(const Node& node);

  /// Message callback.
  /// \param sensor Input sensor.
  /// \param message Input message.
  void callback(const Sensor& sensor, const Message& message) final;
};

} // namespace hyper
