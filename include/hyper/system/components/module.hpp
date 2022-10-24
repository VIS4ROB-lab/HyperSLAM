/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <map>

#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <ros/subscribe_options.h>
#include <ros/subscriber.h>

#include "hyper/sensors/forward.hpp"

#include "hyper/system/components/abstract.hpp"

namespace hyper {

/// A module is a processing unit that manages a number of threads and has
/// associated sensors for which it must execute the requested callbacks.
class Module : public AbstractComponent {
 public:
  // Definitions.
  using Topic = std::string;
  using Handle = ros::NodeHandle;
  using ThreadCount = std::uint32_t;
  using Queue = ros::CallbackQueue;
  using Spinner = ros::AsyncSpinner;

  /// Constructor from handel and number of threads.
  /// \param handle Input handle.
  /// \param thread_count Number of threads.
  Module(Handle& handle, const ThreadCount& thread_count);

  /// Handle modifier.
  /// \return Handle.
  auto handle() -> Handle&;

  /// Queue accessor.
  /// \return Queue.
  [[nodiscard]] auto queue() const -> const Queue&;

  /// Queue modifier.
  /// \return Queue.
  auto queue() -> Queue&;

  /// Spinner accessor.
  /// \return Spinner.
  [[nodiscard]] auto spinner() const -> const Spinner&;

  /// Spinner modifier.
  /// \return Spinner.
  auto spinner() -> Spinner&;

  /// Links a sensor.
  /// \param topic Sensor topic.
  /// \param sensor Sensor to link.
  /// \param frontend Frontend to link against.
  auto link(const Topic& topic, const Sensor& sensor, AbstractFrontend& frontend) -> void;

  /// Unlinks a sensor.
  /// \param sensor Sensor to unlink.
  auto unlink(const Sensor& sensor) -> void;

  /// Unlinks all sensors.
  auto clear() -> void;

 private:
  using Subscriber = ros::Subscriber;
  using Subscribers = std::map<const Sensor*, Subscriber>;

  Handle* handle_;          ///< Handle.
  Queue queue_;             ///< Queue.
  Spinner spinner_;         ///< Spinner.
  Subscribers subscribers_; ///< Subscribers.
};

} // namespace hyper
