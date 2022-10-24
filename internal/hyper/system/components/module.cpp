/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <typeindex>

#include <glog/logging.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "hyper/sensors/camera.hpp"
#include "hyper/sensors/imu.hpp"
#include "hyper/system/components/frontends/abstract.hpp"
#include "hyper/system/components/module.hpp"

namespace hyper {

Module::Module(Handle& handle, const ThreadCount& thread_count)
    : handle_{&handle},
      queue_{},
      spinner_{thread_count, &queue_},
      subscribers_{} {}

auto Module::handle() -> Handle& {
  DCHECK(handle_ != nullptr);
  return *handle_;
}

auto Module::queue() const -> const Queue& {
  return queue_;
}

auto Module::queue() -> Queue& {
  return const_cast<Queue&>(std::as_const(*this).queue());
}

auto Module::spinner() const -> const Spinner& {
  return spinner_;
}

auto Module::spinner() -> Spinner& {
  return const_cast<Spinner&>(std::as_const(*this).spinner());
}

auto Module::link(const Topic& topic, const Sensor& sensor, AbstractFrontend& frontend) -> void {
  // Definitions.
  using Options = ros::SubscribeOptions;

  // Create callback.
  const auto max_queue_size = frontend.maxQueueSize();
  const auto callback = [&sensor, &frontend](auto&& arg) { frontend.callback(sensor, std::forward<decltype(arg)>(arg)); };

  // Create options.
  Options options;
  const auto type_index = std::type_index{typeid(sensor)};
  if (type_index == std::type_index{typeid(Camera)}) {
    options = Options::create<sensor_msgs::Image>(topic, max_queue_size, callback, ros::VoidPtr(), &queue_);
  } else if (type_index == std::type_index{typeid(IMU)}) {
    options = Options::create<sensor_msgs::Imu>(topic, max_queue_size, callback, ros::VoidPtr(), &queue_);
  } else {
    LOG(FATAL) << "Unknown sensor can not be linked.";
  }

  // Create subscriber.
  options.allow_concurrent_callbacks = false; // TODO: Potentially allow this.
  subscribers_.try_emplace(&sensor, handle_->subscribe(options));
}

auto Module::unlink(const Sensor& sensor) -> void {
  const auto itr = subscribers_.find(&sensor);
  DCHECK(itr != subscribers_.cend());
  subscribers_.erase(itr);
}

auto Module::clear() -> void {
  subscribers_.clear();
}

} // namespace hyper
