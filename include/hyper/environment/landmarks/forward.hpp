/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/forward.hpp"

namespace hyper {

class AbstractLandmark;

class PositionLandmark;

template <>
struct Traits<PositionLandmark> {
  // Constants.
  static constexpr auto kPositionOffset = 0;
  static constexpr auto kNumParameterBlocks = kPositionOffset + 1;
};

} // namespace hyper
