/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/observations/forward.hpp"

#include "hyper/optimizers/evaluators/abstract.hpp"

namespace hyper {

template <typename TObservation>
class Evaluator final
    : public AbstractEvaluator {
 public:
  /// Evaluates a query.
  /// \param query Input query.
  /// \param J_e Output Jacobian.
  /// \return Evaluator result.
  [[nodiscard]] auto evaluate(const Query& query, DynamicJacobian<Scalar>* J_e) const -> Result final;
};

using PixelEvaluator = Evaluator<VisualObservation<CameraMeasurement<Pixel<Scalar>>>>;

using BearingEvaluator = Evaluator<VisualObservation<CameraMeasurement<Bearing<Scalar>>>>;

template <typename TManifold>
using ManifoldEvaluator = Evaluator<ManifoldObservation<TManifold>>;

template <typename TManifold>
using InertialEvaluator = Evaluator<InertialObservation<TManifold>>;

} // namespace hyper
