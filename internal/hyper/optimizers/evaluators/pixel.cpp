/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/environment/landmarks/forward.hpp"
#include "hyper/sensors/forward.hpp"

#include "hyper/optimizers/evaluators/evaluator.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/state/abstract.hpp"
#include "hyper/variables/adapters.hpp"
#include "hyper/variables/intrinsics.hpp"

namespace hyper {

template <>
auto VisualPixelEvaluator<SE3<Scalar>>::evaluate(const EvaluatorQuery<Scalar, Index>& query) const -> DynamicVector<Scalar> {
  // Definitions.
  using Tangent = Tangent<Manifold>;
  using Transformation = Traits<Sensor>::Transformation;
  using Intrinsics = Intrinsics<Scalar>;
  using Position = Position<Scalar>;
  using Landmark = Observation::Landmark;
  using Variable = Landmark::Variable;
  using Output = Pixel<Scalar>;

  //  Unpack arguments.
  const auto& [pointers, layout, context] = query;
  const auto& [p_ps, p_J_e, p_js] = pointers;
  const auto& [num_parameters, indices, offsets, sizes] = layout;
  const auto& [state, observation] = context;

  // Retrieve measurement and sensor.
  const auto& measurement = context.observation->measurement();
  const auto& camera = measurement.sensor().as<Camera>();

  // Retrieve stamp.
  const auto& stamp = observation->measurement().stamp();

  // State pointer.
  const auto p_S_wb_0 = p_ps + indices.static_state_idx;

  // Compute offsets.
  const auto o_T_bs = indices.static_sensor_idx + Traits<Camera>::kTransformationOffset;
  const auto o_i = indices.static_sensor_idx + Traits<Camera>::kIntrinsicsOffset;
  const auto o_d = indices.static_sensor_idx + Traits<Camera>::kDistortionOffset;
  const auto o_p_w = indices.static_observation_idx + Traits<Landmark>::kVariableOffset;

  // Map parameters.
  const auto T_bs = Eigen::Map<const Transformation>{p_ps[o_T_bs]};
  const auto c_i = Eigen::Map<const Intrinsics>{p_ps[o_i]};
  const auto c_d = camera.distortion().map(p_ps[o_d]);
  const auto p_w = Eigen::Map<const Variable>{p_ps[o_p_w]};

  if (!p_J_e) {
    const auto q_S_wb = StateQuery{stamp, kValueIndex};
    const auto S_wb = state->evaluate(q_S_wb, p_S_wb_0);

    const auto T_ws = S_wb.derivativeAs<Manifold>(kValueIndex).groupPlus(T_bs);
    const auto T_sw = T_ws.groupInverse();
    const auto p_s = T_sw.vectorPlus(p_w);
    const auto n_px_s = Camera::ProjectToPlane(p_s);
    const auto d_n_px_s = c_d->distort(n_px_s, nullptr, nullptr);
    auto d_px_s = c_i.denormalize(d_n_px_s);

    return d_px_s;

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

    Jacobian<Position, Tangent> J_p_s_T_sw;
    auto p_s = T_sw.vectorPlus(p_w, J_p_s_T_sw.data());

    Jacobian<Output, Position> J_n_px_s_p_s;
    const auto n_px_s = Camera::ProjectToPlane(p_s, J_n_px_s_p_s.data());

    Output d_px_s;
    Jacobian<Output> J_r_n_px_s;
    if (p_js[o_d] && p_js[o_i]) {
      const auto v_c_d = c_d->asVector();
      Jacobian<Output> J_d_n_px_s_n_px_s;
      DynamicInputJacobian<Output> J_d_n_px_s_d{Traits<Output>::kNumParameters, v_c_d.size()};
      const auto d_n_px_s = c_d->distort(n_px_s, J_d_n_px_s_n_px_s.data(), J_d_n_px_s_d.data());

      Jacobian<Output> J_r_d_n_px_s;
      Jacobian<Output, Intrinsics> J_d_px_s_c_i;
      d_px_s = c_i.denormalize(d_n_px_s, J_r_d_n_px_s.data(), J_d_px_s_c_i.data());

      p_J_e->middleCols(offsets[o_d], (*sizes)[o_d]).noalias() = J_r_d_n_px_s * J_d_n_px_s_d;
      p_J_e->middleCols(offsets[o_i], (*sizes)[o_i]).noalias() = J_d_px_s_c_i;
      J_r_n_px_s = J_r_d_n_px_s * J_d_n_px_s_n_px_s;

    } else if (p_js[o_d]) {
      const auto v_c_d = c_d->asVector();
      Jacobian<Output> J_d_n_px_s_n_px_s;
      DynamicInputJacobian<Output> J_d_n_px_s_d{Traits<Output>::kNumParameters, v_c_d.size()};
      const auto d_n_px_s = c_d->distort(n_px_s, J_d_n_px_s_n_px_s.data(), J_d_n_px_s_d.data());

      Jacobian<Output> J_r_d_n_px_s;
      d_px_s = c_i.denormalize(d_n_px_s, J_r_d_n_px_s.data(), nullptr);

      p_J_e->middleCols(offsets[o_d], (*sizes)[o_d]).noalias() = J_r_d_n_px_s * J_d_n_px_s_d;
      J_r_n_px_s = J_r_d_n_px_s * J_d_n_px_s_n_px_s;

    } else if (p_js[o_i]) {
      Jacobian<Output> J_d_n_px_s_n_px_s;
      const auto d_n_px_s = c_d->distort(n_px_s, J_d_n_px_s_n_px_s.data(), nullptr);

      Jacobian<Output> J_r_d_n_px_s;
      Jacobian<Output, Intrinsics> J_d_px_s_c_i;
      d_px_s = c_i.denormalize(d_n_px_s, J_r_d_n_px_s.data(), J_d_px_s_c_i.data());

      p_J_e->middleCols(offsets[o_i], (*sizes)[o_i]).noalias() = J_d_px_s_c_i;
      J_r_n_px_s = J_r_d_n_px_s * J_d_n_px_s_n_px_s;

    } else {
      Jacobian<Output> J_d_n_px_s_n_px_s;
      const auto d_n_px_s = c_d->distort(n_px_s, J_d_n_px_s_n_px_s.data(), nullptr);

      Jacobian<Output> J_r_d_n_px_s;
      d_px_s = c_i.denormalize(d_n_px_s, J_r_d_n_px_s.data(), nullptr);
      J_r_n_px_s = J_r_d_n_px_s * J_d_n_px_s_n_px_s;
    }

    const auto J_r_p_s = (J_r_n_px_s * J_n_px_s_p_s).eval();
    const auto J_r_T_ws = (J_r_p_s * J_p_s_T_sw * J_T_sw_T_ws).eval();

    if (J_S_wb) p_J_e->middleCols(offsets[indices.static_state_idx], S_wb.jacobians[kValueIndex].cols()).noalias() = J_r_T_ws * J_T_ws_S_wb * S_wb.jacobians[kValueIndex];
    if (p_js[o_T_bs]) p_J_e->middleCols(offsets[o_T_bs], (*sizes)[o_T_bs]).noalias() = J_r_T_ws * J_T_ws_T_bs * SE3JacobianAdapter(T_bs.data());
    if (p_js[o_p_w]) p_J_e->middleCols(offsets[o_p_w], (*sizes)[o_p_w]).noalias() = J_r_p_s * T_sw.rotation().matrix();

    return d_px_s;
  }
}

} // namespace hyper
