/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/camera.hpp"
#include "hyper/variables/distortions/radial_tangential.hpp"
#include "hyper/variables/intrinsics.hpp"

#include "tests/random.hpp"

namespace hyper::tests {

template <>
class Mock<Camera> {
 public:
  /// Deleted default constructor.
  Mock() = delete;

  /// Creates a camera.
  /// \return Camera.
  static auto Create() -> std::unique_ptr<Camera> {
    auto camera = std::make_unique<Camera>();
    camera->sensorSize() = {752, 480};
    camera->transformation() = Mock<Traits<Camera>::Transformation>::Random();
    camera->intrinsics() = Intrinsics<Scalar>{367.215, 248.375, 458.654, 457.296};

    // Create distortion.
    constexpr auto kMaxPerturbation = 0.05;
    auto distortion = std::make_unique<RadialTangentialDistortion<Scalar, 2>>(-0.28340811, 0.07395907, 1.76187114e-05, 0.00019359);
    distortion->perturb(kMaxPerturbation);
    camera->setDistortion(std::move(distortion));

    return camera;
  }
};

} // namespace hyper::tests
