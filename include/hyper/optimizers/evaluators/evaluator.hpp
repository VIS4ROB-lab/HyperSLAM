/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/observations/visual.hpp"
#include "hyper/optimizers/evaluators/abstract.hpp"

namespace hyper {

template <typename TObservation, typename TManifold>
class Evaluator final
    : public AbstractEvaluator {
 public:
  // Definitions.
  using Manifold = TManifold;
  using Observation = TObservation;

  /// Evaluates a query.
  /// \param query Input query.
  /// \return Result.
  [[nodiscard]] auto evaluate(const EvaluatorQuery<Scalar, Index>& query) const
      -> DynamicVector<Scalar> final;
};

template <typename TManifold>
using VisualPixelEvaluator = Evaluator<VisualPixelObservation, TManifold>;

template <typename TManifold>
using VisualBearingEvaluator = Evaluator<VisualBearingObservation, TManifold>;

template <typename TManifold>
using ManifoldEvaluator = Evaluator<ManifoldObservation<TManifold>, TManifold>;

template <typename TManifold>
using InertialEvaluator = Evaluator<InertialObservation<TManifold>, TManifold>;

} // namespace hyper
