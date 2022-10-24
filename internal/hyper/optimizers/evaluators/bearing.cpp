/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/sensors/forward.hpp"

#include "hyper/environment/landmarks/variable.hpp"
#include "hyper/optimizers/evaluators/evaluator.hpp"
#include "hyper/state/abstract.hpp"
#include "hyper/variables/adapters.hpp"

namespace hyper {

template <>
auto VisualBearingEvaluator<SE3<Scalar>>::evaluate(const EvaluatorQuery<Scalar, Index>& query) const -> DynamicVector<Scalar> {
  // Definitions.
  using Tangent = Tangent<Manifold>;
  using Transformation = Traits<Sensor>::Transformation;
  using Landmark = Observation::Landmark;
  using Variable = Landmark::Variable;
  using Output = Position<Scalar>;

  //  Unpack arguments.
  const auto& [pointers, layout, context] = query;
  const auto& [p_ps, p_J_e, p_js] = pointers;
  const auto& [num_parameters, indices, offsets, sizes] = layout;
  const auto& [state, observation] = context;

  // Retrieve stamp.
  const auto& stamp = context.observation->measurement().stamp();

  // State pointer.
  const auto p_S_wb_0 = p_ps + indices.static_state_idx;

  // Compute offsets.
  const auto o_T_bs = indices.static_sensor_idx + Traits<Camera>::kTransformationOffset;
  const auto o_p_w = indices.static_observation_idx + Traits<Landmark>::kVariableOffset;

  // Map parameters.
  const auto T_bs = Eigen::Map<const Transformation>{p_ps[o_T_bs]};
  const auto p_w = Eigen::Map<const Variable>{p_ps[o_p_w]};

  if (!p_J_e) {
    const auto q_S_wb = StateQuery{stamp, kValueIndex};
    const auto S_wb = state->evaluate(q_S_wb, p_S_wb_0);

    const auto T_ws = S_wb.derivativeAs<Manifold>(kValueIndex).groupPlus(T_bs);
    const auto T_sw = T_ws.groupInverse();
    auto p_s = T_sw.vectorPlus(p_w);

    return p_s;

  } else {
    // Allocate (full) Jacobian.
    p_J_e->setZero(Traits<Output>::kNumParameters, num_parameters);

    DCHECK(state->interpolator() != nullptr);
    const auto p_S_wb_1 = p_S_wb_0 + state->interpolator()->layout().outer.size;
    const auto J_S_wb = std::any_of(p_S_wb_0, p_S_wb_1, [](const auto& arg) { return arg != nullptr; });
    const auto q_S_wb = StateQuery{stamp, kValueIndex, J_S_wb};
    const auto S_wb = state->evaluate(q_S_wb, p_S_wb_0);

    Jacobian<Tangent> J_T_ws_S_wb, J_T_ws_T_bs;
    const auto T_ws = S_wb.derivativeAs<Manifold>(kValueIndex).groupPlus(T_bs, J_T_ws_S_wb.data(), J_T_ws_T_bs.data());

    Jacobian<Tangent> J_T_sw_T_ws;
    const auto T_sw = T_ws.groupInverse(J_T_sw_T_ws.data());

    Jacobian<Output, Tangent> J_p_s_T_sw;
    auto p_s = T_sw.vectorPlus(p_w, J_p_s_T_sw.data());

    const auto J_p_s_T_ws = (J_p_s_T_sw * J_T_sw_T_ws).eval();

    if (J_S_wb) p_J_e->middleCols(offsets[indices.static_state_idx], S_wb.jacobians[kValueIndex].cols()).noalias() = J_p_s_T_ws * J_T_ws_S_wb * S_wb.jacobians[kValueIndex];
    if (p_js[o_T_bs]) p_J_e->middleCols(offsets[o_T_bs], (*sizes)[o_T_bs]).noalias() = J_p_s_T_ws * J_T_ws_T_bs * SE3JacobianAdapter(T_bs.data());
    if (p_js[o_p_w]) p_J_e->middleCols(offsets[o_p_w], (*sizes)[o_p_w]).noalias() = T_sw.rotation().matrix();

    return p_s;
  }
}

} // namespace hyper
