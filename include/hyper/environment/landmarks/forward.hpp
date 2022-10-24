/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/forward.hpp"

namespace hyper {

class AbstractLandmark;

template <typename TVariable>
class Landmark;

template <typename TVariable>
struct Traits<Landmark<TVariable>> {
  // Constants.
  static constexpr auto kVariableOffset = 0;
  static constexpr auto kNumParameterBlocks = kVariableOffset + 1;
};

} // namespace hyper
