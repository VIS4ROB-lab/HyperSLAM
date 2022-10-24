/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

namespace hyper {

template <typename>
struct Traits;

class AbstractEnvironment;

template <>
struct Traits<AbstractEnvironment> {
  static constexpr auto kGravityOffset = 0;
  static constexpr auto kNumParameters = kGravityOffset + 1;
};

template <typename>
class Environment;

template <typename TManifold>
struct Traits<Environment<TManifold>>
    : Traits<AbstractEnvironment> {
  static constexpr auto kTransformationOffset = Traits<AbstractEnvironment>::kNumParameters;
  static constexpr auto kNumParameters = kTransformationOffset + 1;
};

} // namespace hyper
