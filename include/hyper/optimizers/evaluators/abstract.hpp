/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/optimizers/evaluators/forward.hpp"

namespace hyper {

class AbstractEvaluator {
 public:
  // Constants.
  static constexpr auto kValueIndex = 0;
  static constexpr auto kVelocityIndex = 1;
  static constexpr auto kAccelerationIndex = 2;
  static constexpr auto kJerkIndex = 3;

  /// Virtual default destructor.
  virtual ~AbstractEvaluator() = default;

  /// Evaluates a query.
  /// \param query Input query.
  /// \return Result.
  [[nodiscard]] virtual auto evaluate(const EvaluatorQuery<Scalar, Index>& query) const -> DynamicVector<Scalar> = 0;
};

} // namespace hyper
