/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/forward.hpp"
#include "hyper/state/forward.hpp"

namespace hyper {

template <typename TValue, typename TIndex = std::int32_t>
struct EvaluatorQuery {
  // Definitions.
  using Sizes = std::vector<TIndex>;
  using Offsets = std::vector<TIndex>;

  // Evaluator pointers.
  struct Pointers {
    const TValue* const* parameters;
    TValue** jacobians;
  };

  // Evaluator layout.
  struct Layout {
    TIndex static_state_idx;
    TIndex static_sensor_idx;
    TIndex dynamic_sensor_idx;
    TIndex static_observation_idx;
  };

  // Members.
  const Pointers& pointers;
  const Layout& layout;
  const Offsets& offsets;
  const Sizes& sizes;
  const AbstractState& state;
  const AbstractMeasurement& measurement;
};

} // namespace hyper
