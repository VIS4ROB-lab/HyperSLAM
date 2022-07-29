/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/optimizers/evaluators/forward.hpp"

#include "hyper/optimizers/evaluators/query.hpp"
#include "hyper/variables/jacobian.hpp"

namespace hyper {

class AbstractEvaluator {
 public:
  // Definitions.
  using Query = EvaluatorQuery<Scalar>;
  using Result = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  // Constants.
  static constexpr auto kValueIndex = 0;
  static constexpr auto kVelocityIndex = 1;
  static constexpr auto kAccelerationIndex = 2;
  static constexpr auto kJerkIndex = 3;

  /// Virtual default destructor.
  virtual ~AbstractEvaluator() = default;

  /// Evaluates a query.
  /// \param query Input query.
  /// \param J_e Output Jacobian.
  /// \return Evaluator result.
  [[nodiscard]] virtual auto evaluate(const Query& query, DynamicJacobian<Scalar>* J_e) const -> Result = 0;

 protected:
  /// Default constructor.
  AbstractEvaluator() = default;
};

} // namespace hyper
