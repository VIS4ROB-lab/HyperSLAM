/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <sensor_msgs/Imu.h>

#include "hyper/messages/measurements/inertial.hpp"
#include "hyper/sensors/imu.hpp"
#include "hyper/system/components/frontends/inertial/direct.hpp"
#include "hyper/variables/groups/se3.hpp"

namespace hyper {

template <>
InertialFrontend<SE3<Scalar>, InertialFrontendType::DIRECT>::InertialFrontend(const Node& node)
    : AbstractFrontend{node} {}

template <>
void InertialFrontend<SE3<Scalar>, InertialFrontendType::DIRECT>::callback(const Sensor& sensor, const Message& message) {
  // Definitions.
  using Tangent = Tangent<Manifold>;
  using Angular = Tangent::Angular;
  using Linear = Tangent::Linear;

  // Downcast message.
  const auto imu_message = boost::static_pointer_cast<const sensor_msgs::Imu>(message);

  // Convert to tangent.
  Tangent tangent;
  const auto& angular = imu_message->angular_velocity;
  const auto& linear = imu_message->linear_acceleration;
  tangent.angular() = Angular{angular.x, angular.y, angular.z};
  tangent.linear() = Linear{linear.x, linear.y, linear.z};

  // Create and submit measurement.
  const auto stamp = imu_message->header.stamp.toSec();
  auto measurement = std::make_unique<InertialMeasurement<Manifold>>(stamp, sensor.as<const IMU>(), tangent);
  submit(std::move(measurement));
}

} // namespace hyper
