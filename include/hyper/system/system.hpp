/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <ros/node_handle.h>

#include "hyper/sensors/sensor.hpp"
#include "hyper/system/components/backend.hpp"
#include "hyper/system/components/frontends/abstract.hpp"
#include "hyper/system/components/module.hpp"

namespace hyper {

class System {
 public:
  using Node = YAML::Node;
  using Handle = ros::NodeHandle;
  using Sensors = std::map<std::string, std::unique_ptr<Sensor>>;
  using Modules = std::map<std::string, std::unique_ptr<Module>>;
  using Frontends = std::map<std::string, std::unique_ptr<AbstractFrontend>>;
  using Backends = std::map<std::string, std::unique_ptr<Backend>>;

  /// Constructor from YAML node.
  /// \param yaml_node Input YAML node.
  explicit System(const Node& yaml_node);

  /// Handle accessor.
  /// \return Handle.
  [[nodiscard]] auto handle() const -> const Handle&;

  /// Sensors accessor.
  /// \return Sensors.
  [[nodiscard]] auto sensors() const -> const Sensors&;

  /// Modules accessor.
  /// \return Modules.
  [[nodiscard]] auto modules() const -> const Modules&;

  /// Frontends accessor.
  /// \return Frontends.
  [[nodiscard]] auto frontends() const -> const Frontends&;

  /// Backends accessor.
  /// \return Backends.
  [[nodiscard]] auto backends() const -> const Backends&;

  /// Starts the system.
  auto start() -> void;

  /// Shuts the system down.
  auto shutdown() -> void;

 private:
  Handle handle_;       ///< Handle.
  Sensors sensors_;     ///< Sensors.
  Modules modules_;     ///< Modules.
  Frontends frontends_; ///< Frontends.
  Backends backends_;   ///< Backends.
};

} // namespace hyper
