/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <boost/shared_ptr.hpp>

#include "hyper/sensors/forward.hpp"
#include "hyper/system/components/frontends/forward.hpp"

#include "hyper/system/components/abstract.hpp"

namespace hyper {

class AbstractFrontend
    : public AbstractComponent {
 public:
  using QueueSize = std::uint32_t;
  using Message = boost::shared_ptr<const void>;

  /// Maximum queue size accessor.
  /// \return Maximum queue size.
  [[nodiscard]] auto maxQueueSize() const -> const QueueSize&;

  /// Maximum queue size modifier.
  /// \return Maximum queue size.
  auto maxQueueSize() -> QueueSize&;

  /// Backend accessor.
  /// \return Backend.
  [[nodiscard]] auto backend() const -> const Backend&;

  /// Sets the backend.
  /// \param backend Input backend.
  auto setBackend(const Backend& backend) -> void;

  /// Message callback.
  /// \param sensor Submitting sensor.
  /// \param message Input message.
  virtual void callback(const Sensor& sensor, const Message& message) = 0;

 protected:
  /// Constructor from YAML node.
  /// \param node Input YAML node.
  explicit AbstractFrontend(const Node& node);

  /// Submits a message to the backend(s).
  /// \param message Input message.
  auto submit(std::unique_ptr<AbstractMessage>&& message) -> void;

 private:
  QueueSize max_queue_size_; ///< Maximum queue size.
  const Backend* backend_;   ///< Backend.
};

} // namespace hyper
