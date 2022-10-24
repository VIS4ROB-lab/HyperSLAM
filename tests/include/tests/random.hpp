/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/definitions.hpp"
#include "hyper/environment/landmarks/variable.hpp"
#include "hyper/variables/abstract.hpp"
#include "hyper/variables/bearing.hpp"
#include "hyper/variables/gravity.hpp"
#include "hyper/variables/groups/se3.hpp"

#include "tests/forward.hpp"

namespace hyper::tests {

template <>
class Mock<Bearing<Scalar>> {
 public:
  /// Deleted default constructor.
  Mock() = delete;

  /// Generates a random bearing.
  /// \return Random bearing.
  static auto Random() -> Bearing<Scalar> {
    return Traits<Bearing<Scalar>>::kNorm * (Eigen::Quaternion<Scalar>::UnitRandom() * Bearing<Scalar>::Unit(0));
  }
};

template <>
class Mock<Gravity<Scalar>> {
 public:
  /// Deleted default constructor.
  Mock() = delete;

  /// Generates a random gravity.
  /// \return Random gravity.
  static auto Random() -> Gravity<Scalar> {
    return Traits<Gravity<Scalar>>::kNorm * (Eigen::Quaternion<Scalar>::UnitRandom() * Bearing<Scalar>::Unit(0));
  }
};

template <>
class Mock<SU2<Scalar>> {
 public:
  /// Deleted default constructor.
  Mock() = delete;

  /// Generates a random SU2 element.
  /// \return Random SU2 element.
  static auto Random() -> SU2<Scalar> {
    return SU2<Scalar>::UnitRandom();
  }
};

template <>
class Mock<SE3<Scalar>> {
 public:
  /// Deleted default constructor.
  Mock() = delete;

  /// Generates a random SE3 element.
  /// \return Random SE3 element.
  static auto Random() -> SE3<Scalar> {
    return {Mock<SU2<Scalar>>::Random(), Translation<Scalar>::Random()};
  }
};

template <>
class Mock<Landmark<Position<Scalar>>> {
 public:
  // Constants.
  static constexpr auto kMaxRadius = 10;

  /// Deleted default constructor.
  Mock() = delete;

  /// Generates a random bearing.
  /// \return Random bearing.
  static auto InCube() -> Landmark<Position<Scalar>> {
    return Landmark<Position<Scalar>>{kMaxRadius * Translation<Scalar>::Random()};
  }

  /// Generates a random bearing.
  /// \return Random bearing.
  static auto InSphere() -> Landmark<Position<Scalar>> {
    return Landmark<Position<Scalar>>{kMaxRadius * (Eigen::Quaternion<Scalar>::UnitRandom() * Bearing<Scalar>::Unit(0))};
  }
};

} // namespace hyper::tests
