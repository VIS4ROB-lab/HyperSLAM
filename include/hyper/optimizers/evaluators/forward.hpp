/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <vector>

#include "hyper/optimizers/forward.hpp"

#include "hyper/variables/jacobian.hpp"

namespace hyper {

template <typename TScalar>
struct EvaluatorPointers {
  const TScalar* const* parameters;
  DynamicJacobian<TScalar>* jacobian;
  const TScalar* const* jacobians;
};

template <typename TIndex>
struct EvaluatorLayout {
  // Definitions.
  using Index = TIndex;
  using Offsets = std::vector<Index>;
  using Sizes = std::vector<std::int32_t>;

  // Evaluator indices.
  struct Indices {
    Index static_state_idx;
    Index static_sensor_idx;
    Index dynamic_sensor_idx;
    Index static_observation_idx;
  };

  // Members.
  Index num_parameters;
  Indices indices;
  Offsets offsets;
  Sizes* sizes;
};

struct CostContext;

template <typename TScalar, typename TIndex>
struct EvaluatorQuery {
  const EvaluatorPointers<TScalar>& pointers;
  const EvaluatorLayout<TIndex>& layout;
  const CostContext& context;
};

class AbstractEvaluator;

template <typename TObservation, typename TManifold>
class Evaluator;

} // namespace hyper