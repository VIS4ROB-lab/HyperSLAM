/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <fstream>

#include <glog/logging.h>

#include "hyper/environment/environment.hpp"
#include "hyper/messages/measurements/abstract.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/gravity.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/se3.hpp"
#include "hyper/optimizers/ceres/manifolds/variables/stamped.hpp"
#include "hyper/optimizers/ceres/optimizer.hpp"
#include "hyper/system/components/backend.hpp"
#include "hyper/yaml/yaml.hpp"

namespace hyper {

Backend::Backend(const Node& node, const std::vector<Sensor*>& sensors)
    : thread_{},
      locked_{false},
      condition_{},
      mutex_{},
      idle_{true},
      idle_condition_{},
      queue_{},
      queue_condition_{},
      queue_mutex_{},
      optimizer_{nullptr},
      shutdown_{false} {
  // Read parameters from YAML node.
  const auto suite = yaml::ReadString(node, "suite");
  const auto manifold = yaml::ReadString(node, "manifold");
  const auto representation = yaml::ReadString(node, "representation");

  // Setup optimization.
  if (suite == "ceres") {
    if (manifold == "se3") {
      if (representation == "continuous") {
        // Definitions.
        using Manifold = SE3<Scalar>;
        using StampedManifold = Stamped<Manifold>;
        using Optimizer = Optimizer<OptimizerSuite::CERES>;

        // Create optimizer.
        auto optimizer = std::make_unique<Optimizer>(node, sensors);
        auto environment = std::make_unique<Environment<SE3<Scalar>>>();
        optimizer->swapEnvironment(environment);
        optimizer->setGravityManifold(std::make_unique<hyper::Manifold<Gravity<Scalar>, OptimizerSuite::CERES>>());

        // Create manifolds.
        const auto time_constant = yaml::ReadAs<bool>(node, "time_constant");
        const auto rotation_constant = yaml::ReadAs<bool>(node, "rotation_constant");
        const auto translation_constant = yaml::ReadAs<bool>(node, "translation_constant");
        optimizer->setStateManifold(std::make_unique<hyper::Manifold<StampedManifold, OptimizerSuite::CERES>>(time_constant, rotation_constant, translation_constant));
        optimizer_ = std::move(optimizer);

      } else {
        LOG(FATAL) << "Unknown representation.";
      }
    } else {
      LOG(FATAL) << "Unknown manifold.";
    }
  } else {
    LOG(FATAL) << "Unknown suite.";
  }
}

auto Backend::acquire() const -> void {
  Lock lock{mutex_};
  while (locked_) {
    condition_.wait(lock);
  }
  locked_ = true;
}

auto Backend::release() const -> void {
  Lock lock{mutex_};
  locked_ = false;
  condition_.notify_one();
}

auto Backend::optimizer() const -> const std::unique_ptr<AbstractOptimizer>& {
  return optimizer_;
}

auto Backend::optimizer() -> std::unique_ptr<AbstractOptimizer>& {
  return const_cast<std::unique_ptr<AbstractOptimizer>&>(std::as_const(*this).optimizer());
}

auto Backend::start() -> void {
  if (!thread_.joinable()) {
    shutdown_ = false;
    thread_ = Thread{&Backend::spin, this};
    DLOG(INFO) << "Started backend thread " << thread_.get_id() << ".";
  }
}

auto Backend::submit(std::unique_ptr<AbstractMessage>&& message) const -> void {
  Lock lock{queue_mutex_};
  queue_.push(std::move(message));
  idle_ = false;
  lock.unlock();
  queue_condition_.notify_one();
}

auto Backend::wait() const -> Lock {
  Lock lock{queue_mutex_};
  while (!idle_) {
    idle_condition_.wait(lock);
  }
  return lock;
}

auto Backend::shutdown() -> void {
  if (thread_.joinable()) {
    shutdown_ = true;
    queue_condition_.notify_one();
    DLOG(INFO) << "Joining backend thread " << thread_.get_id() << ".";
    thread_.join();
  }
}

auto Backend::spin(Backend* backend) -> int {
  // Repeat until shutdown is requested.
  while (!backend->shutdown_) {
    // Wait for new message.
    Lock lock{backend->queue_mutex_};
    while (!backend->shutdown_ && backend->queue_.empty()) {
      backend->queue_condition_.wait(lock);
    }

    // Break if shutdown is requested.
    if (backend->shutdown_) break;

    // Extract message and unlock queue.
    std::unique_ptr<AbstractMessage> message = std::move(backend->queue_.front());
    backend->queue_.pop();
    lock.unlock();

    // Submit message to optimizer.
    if (backend->optimizer_ != nullptr) {
      backend->acquire();
      backend->optimizer_->submit(std::move(message));
      backend->release();
    }

    // Update idle condition.
    lock.lock();
    if (backend->queue_.empty()) {
      backend->idle_ = true;
      backend->idle_condition_.notify_one();
    }
    lock.unlock();
  }

  return EXIT_SUCCESS;
}

} // namespace hyper
