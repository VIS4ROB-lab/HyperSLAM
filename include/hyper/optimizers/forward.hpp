/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/environment/observations/forward.hpp"
#include "hyper/optimizers/evaluators/forward.hpp"
#include "hyper/state/forward.hpp"
#include "hyper/variables/metrics/forward.hpp"

namespace hyper {

enum class OptimizerSuite {
  CERES,
  DEFAULT = CERES
};

class AbstractOptimizer;

template </* typename TManifold, */ OptimizerSuite>
class Optimizer;

template <typename, OptimizerSuite>
class Manifold;

class AbstractEvaluator;

template <typename TScalar>
struct CostConfiguration {
  // Definitions.
  template <typename TScalar_>
  using Weights = Eigen::Matrix<TScalar_, Eigen::Dynamic, Eigen::Dynamic>;

  // Members.
  const Weights<TScalar>* weights;       ///< Weights.
  const AbstractMetric<TScalar>* metric; ///< Metric.
  const AbstractEvaluator* evaluator;    ///< Evaluator.
};

struct CostContext {
  AbstractState* state;             ///< State.
  AbstractObservation* observation; ///< Observation.
};

} // namespace hyper
