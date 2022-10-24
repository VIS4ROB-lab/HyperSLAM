/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/messages/measurements/abstract.hpp"
#include "hyper/optimizers/evaluators/evaluator.hpp"
#include "hyper/state/abstract.hpp"
#include "hyper/variables/adapters.hpp"

namespace hyper {

template <>
auto ManifoldEvaluator<SE3<Scalar>>::evaluate(const EvaluatorQuery<Scalar, Index>& query) const -> DynamicVector<Scalar> {
  // Definitions.
  using Tangent = Tangent<Manifold>;
  using Transformation = Traits<Sensor>::Transformation;

  //  Unpack arguments.
  const auto& [pointers, layout, context] = query;
  const auto& [p_ps, p_J_e, p_js] = pointers;
  const auto& [num_parameters, indices, offsets, sizes] = layout;
  const auto& [state, observation] = context;

  // Retrieve stamp.
  const auto& stamp = observation->measurement().stamp();

  // State pointer.
  const auto p_S_wb_0 = p_ps + indices.static_state_idx;

  // Evaluate offsets.
  const auto o_T_bs = indices.static_sensor_idx + Traits<Sensor>::kTransformationOffset;

  // Retrieve parameters.
  const auto T_bs = Eigen::Map<const Transformation>{p_ps[o_T_bs]};

  if (!p_J_e) {
    const auto q_S_wb = StateQuery{stamp, kValueIndex};
    const auto S_wb = state->evaluate(q_S_wb, p_S_wb_0);

    auto T_ws = S_wb.derivativeAs<Manifold>(kValueIndex).groupPlus(T_bs);

    return T_ws;

  } else {
    // Allocate (full) Jacobian.
    p_J_e->setZero(Traits<Tangent>::kNumParameters, num_parameters);

    DCHECK(state->interpolator() != nullptr);
    const auto p_S_wb_1 = p_S_wb_0 + state->interpolator()->layout().outer.size;
    const auto J_S_wb = std::any_of(p_S_wb_0, p_S_wb_1, [](const auto& arg) { return arg != nullptr; });
    const auto q_S_wb = StateQuery{stamp, kValueIndex, J_S_wb};
    const auto S_wb = state->evaluate(q_S_wb, p_S_wb_0);

    Jacobian<Tangent> J_r_S_wb, J_r_T_bs;
    auto T_ws = S_wb.derivativeAs<Manifold>(kValueIndex).groupPlus(T_bs, J_r_S_wb.data(), J_r_T_bs.data());

    if (J_S_wb) p_J_e->middleCols(offsets[indices.static_state_idx], S_wb.jacobians[kValueIndex].cols()).noalias() = J_r_S_wb * S_wb.jacobians[kValueIndex];
    if (p_js[o_T_bs]) p_J_e->middleCols(offsets[o_T_bs], (*sizes)[o_T_bs]).noalias() = J_r_T_bs * SE3JacobianAdapter(T_bs.data());

    return T_ws;
  }
}

} // namespace hyper
