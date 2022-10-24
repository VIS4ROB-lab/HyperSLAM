/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/system/components/frontends/forward.hpp"

namespace hyper {

enum class InertialFrontendType {
  DIRECT,
  INTEGRATING,
  DEFAULT = DIRECT
};

template <typename, InertialFrontendType>
class InertialFrontend;

} // namespace hyper
