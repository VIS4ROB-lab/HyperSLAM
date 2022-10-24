/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/system/components/backend.hpp"
#include "hyper/system/components/frontends/abstract.hpp"
#include "hyper/yaml/yaml.hpp"

namespace hyper {

namespace {

// Parameter names.
constexpr auto kMaxQueueSizeName = "max_queue_size";

// Default parameters.
constexpr auto kDefaultMaxQueueSize = 20;

} // namespace

auto AbstractFrontend::maxQueueSize() const -> const QueueSize& {
  return max_queue_size_;
}

auto AbstractFrontend::maxQueueSize() -> QueueSize& {
  return const_cast<QueueSize&>(std::as_const(*this).maxQueueSize());
}

auto AbstractFrontend::backend() const -> const Backend& {
  DCHECK(backend_ != nullptr);
  return *backend_;
}

auto AbstractFrontend::setBackend(const Backend& backend) -> void {
  backend_ = &backend;
}

AbstractFrontend::AbstractFrontend(const Node& node)
    : max_queue_size_{kDefaultMaxQueueSize},
      backend_{nullptr} {
  max_queue_size_ = yaml::ReadAs<QueueSize>(node, kMaxQueueSizeName);
}

auto AbstractFrontend::submit(std::unique_ptr<AbstractMessage>&& message) -> void {
  if (backend_ != nullptr) {
    backend_->submit(std::move(message));
  }
}

} // namespace hyper
