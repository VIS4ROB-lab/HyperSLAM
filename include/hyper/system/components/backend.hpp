/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <condition_variable>
#include <queue>
#include <thread>

#include "hyper/sensors/forward.hpp"
#include "hyper/yaml/forward.hpp"

#include "hyper/optimizers/abstract.hpp"
#include "hyper/system/components/abstract.hpp"

namespace hyper {

class Backend final : public AbstractComponent {
 public:
  // Definitions.
  using Thread = std::thread;
  using Condition = std::condition_variable;
  using Mutex = std::mutex;
  using Lock = std::unique_lock<Mutex>;

  /// Constructor from YAML node and set of sensors.
  /// \param node Input YAML backend node.
  /// \param sensors Associated sensors.
  explicit Backend(const Node& node, const std::vector<Sensor*>& sensors);

  /// Acquires exclusive access.
  auto acquire() const -> void;

  /// Releases exclusive access.
  auto release() const -> void;

  /// Optimizer accessor.
  /// \return Reference to optimizer.
  auto optimizer() const -> const std::unique_ptr<AbstractOptimizer>&;

  /// Optimizer modifier.
  /// \return Reference to optimizer.
  auto optimizer() -> std::unique_ptr<AbstractOptimizer>&;

  /// Starts a new backend thread
  /// and begins to process messages.
  auto start() -> void;

  /// Submits a message.
  /// \param measurement Input message.
  auto submit(std::unique_ptr<AbstractMessage>&& message) const -> void;

  /// Waits until the backend idles.
  /// \return Message queue lock.
  auto wait() const -> Lock;

  /// Request shutdown.
  auto shutdown() -> void;

 private:
  /// Periodically checks for new messages,
  /// removes them from the message queue
  /// and forwards them for further processing.
  /// \param backend Associated backend.
  /// \return EXIT_SUCCESS on successful termination.
  static auto spin(Backend* backend) -> int;

  Thread thread_; ///< Thread.

  mutable bool locked_;         ///< Lock flag.
  mutable Condition condition_; ///< Condition.
  mutable Mutex mutex_;         ///< Mutex.

  mutable bool idle_;                ///< Idle flag.
  mutable Condition idle_condition_; ///< Idle condition.

  mutable std::queue<std::unique_ptr<AbstractMessage>> queue_; ///< Queue.
  mutable Condition queue_condition_;                          ///< Queue condition.
  mutable Mutex queue_mutex_;                                  ///< Queue mutex.

  std::unique_ptr<AbstractOptimizer> optimizer_; ///< Optimizer.

  bool shutdown_; ///< Shutdown flag.
};

} // namespace hyper
