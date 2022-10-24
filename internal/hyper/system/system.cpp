/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/system/system.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/sensors/imu.hpp"
#include "hyper/system/components/frontends/inertial/direct.hpp"
#include "hyper/system/components/frontends/visual/klt.hpp"

namespace hyper {

namespace {

using Node = YAML::Node;
using Handle = ros::NodeHandle;

template <typename Type>
using Entry = std::pair<std::string, std::unique_ptr<Type>>;

/// Creates a sensor.
/// \param node Input YAML node.
/// \return Sensor entry.
auto createSensor(const Node& node) -> Entry<Sensor> {
  const auto type = yaml::ReadString(node, "type");
  const auto name = yaml::ReadString(node, "name");
  if (type == "camera") {
    return {name, std::make_unique<Camera>(node)};
  } else if (type == "imu") {
    return {name, std::make_unique<IMU>(node)};
  } else {
    LOG(FATAL) << "Unknown sensor type '" << type << "'.";
    return {};
  }
}

/// Creates a module.
/// \param node_handle Input node handle.
/// \param node Input YAML node.
/// \return Module entry.
auto createModule(Handle& handle, const Node& node) -> Entry<Module> {
  const auto name = yaml::ReadString(node, "name");
  const auto threads = yaml::ReadAs<std::uint32_t>(node, "threads");
  return {name, std::make_unique<Module>(handle, threads)};
}

/// Creates a frontend.
/// \param node Input YAML node.
/// \return Frontend entry.
auto createFrontend(const Node& node) -> Entry<AbstractFrontend> {
  const auto type = yaml::ReadString(node, "type");
  const auto name = yaml::ReadString(node, "name");
  if (type == "visual") {
    return {name, std::make_unique<VisualFrontend>(node)};
  } else if (type == "inertial") {
    return {name, std::make_unique<InertialFrontend<SE3<Scalar>, InertialFrontendType::DIRECT>>(node)};
  } else {
    LOG(FATAL) << "Unknown frontend type '" << type << "'.";
    return {};
  }
}

/// Creates a backend.
/// \param node Input YAML backend node.
/// \param sensors Associated sensors.
/// \return Backend entry.
auto createBackend(const Node& node, const std::vector<Sensor*>& sensors) -> Entry<Backend> {
  const auto name = yaml::ReadString(node, "name");
  return {name, std::make_unique<Backend>(node, sensors)};
}

auto linkSensor(const System& system, const Node& sensor_node) -> void {
  // Retrieve associated sensor.
  const auto sensor_name = yaml::ReadString(sensor_node, "name");
  const auto sensor_itr = system.sensors().find(sensor_name);
  CHECK(sensor_itr != system.sensors().cend());
  const auto& sensor = *sensor_itr->second;

  // Retrieve associated module.
  const auto module_name = yaml::ReadString(sensor_node, "module");
  const auto module_itr = system.modules().find(module_name);
  CHECK(module_itr != system.modules().cend());
  auto& module = *module_itr->second;

  // Retrieve associated frontend.
  const auto frontend_name = yaml::ReadString(sensor_node, "frontend");
  const auto frontend_itr = system.frontends().find(frontend_name);
  CHECK(frontend_itr != system.frontends().cend());
  auto& frontend = *frontend_itr->second;

  // Link and store sensor.
  const auto topic = yaml::ReadString(sensor_node, "topic");
  module.link(topic, sensor, frontend);
}

auto linkFrontend(const System& system, const Node& frontend_node) -> void {
  // Retrieve associated frontend.
  const auto frontend_name = yaml::ReadString(frontend_node, "name");
  const auto frontend_itr = system.frontends().find(frontend_name);
  CHECK(frontend_itr != system.frontends().cend());
  auto& frontend = *frontend_itr->second;

  // Retrieve associated backend.
  const auto backend_name = yaml::ReadString(frontend_node, "backend");
  const auto backend_itr = system.backends().find(backend_name);
  CHECK(backend_itr != system.backends().cend());
  const auto& backend = *backend_itr->second;

  // Link frontend to backend(s).
  frontend.setBackend(backend);
}

} // namespace

System::System(const Node& node)
    : handle_{"hyper"} {
  // Create and register sensors.
  for (const auto& sensor : yaml::Read(node, "sensors")) {
    sensors_.insert(createSensor(sensor));
  }

  // Create and register modules.
  for (const auto& module : yaml::Read(node, "modules")) {
    modules_.insert(createModule(handle_, module));
  }

  // Create and register frontends.
  for (const auto& frontend : yaml::Read(node, "frontends")) {
    frontends_.insert(createFrontend(frontend));
  }

  // Create and register backends.
  for (const auto& backend : yaml::Read(node, "backends")) {
    // TODO: Only provide (correct) subset of sensors for multiple backends.
    std::vector<Sensor*> ptrs;
    ptrs.reserve(sensors_.size());
    for (const auto& sensor : sensors_) {
      ptrs.emplace_back(sensor.second.get());
    }
    backends_.insert(createBackend(backend, ptrs));
  }

  // Link sensors to modules and frontends.
  for (const auto& sensor : yaml::Read(node, "sensors")) {
    linkSensor(*this, sensor);
  }

  // Link frontends to backends.
  for (const auto& frontend : yaml::Read(node, "frontends")) {
    linkFrontend(*this, frontend);
  }
}

auto System::handle() const -> const Handle& {
  return handle_;
}

auto System::sensors() const -> const Sensors& {
  return sensors_;
}

auto System::modules() const -> const Modules& {
  return modules_;
}

auto System::frontends() const -> const Frontends& {
  return frontends_;
}

auto System::backends() const -> const Backends& {
  return backends_;
}

auto System::start() -> void {
  // Status.
  LOG(INFO) << "HyperSLAM running on thread " << std::this_thread::get_id() << ".";

  // Start all modules.
  for (const auto& [_, module] : modules_) {
    module->spinner().start();
  }

  // Start all backends.
  for (const auto& [_, backend] : backends_) {
    backend->start();
  }
}

auto System::shutdown() -> void {
  // Stop all modules.
  for (const auto& [_, module] : modules_) {
    module->spinner().stop();
  }

  // Stop all backends.
  for (const auto& [_, backend] : backends_) {
    backend->shutdown();
  }

  // Status.
  LOG(INFO) << "HyperSLAM on thread " << std::this_thread::get_id() << " stopped.";
}

} // namespace hyper
