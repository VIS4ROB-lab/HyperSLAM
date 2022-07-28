/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

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

} // namespace hyper
