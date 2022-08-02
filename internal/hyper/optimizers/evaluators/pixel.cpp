/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/environment/landmarks/forward.hpp"
#include "hyper/sensors/forward.hpp"

#include "hyper/messages/measurements/abstract.hpp"
#include "hyper/optimizers/evaluators/evaluator.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/state/abstract.hpp"
#include "hyper/variables/adapters.hpp"
#include "hyper/variables/intrinsics.hpp"

namespace hyper {

template <>
auto PixelEvaluator::evaluate(const Query& query, DynamicJacobian<Scalar>* J_e) const -> Result {
  // Definitions.
  using Pixel = Pixel<Scalar>;
  using Position = Position<Scalar>;
  using Intrinsics = Intrinsics<Scalar>;
  using Transformation = Traits<Sensor>::Transformation;
  using Manifold = SE3<Scalar>;
  using Tangent = Tangent<Manifold>;

  // Unpack query.
  const auto& [pointers, layout, offsets, sizes, state, measurement] = query;
  const auto& [p_ps, p_js] = pointers;

  // State pointer.
  const auto p_S_wb_0 = p_ps + layout.static_state_idx;

  // Compute offsets.
  const auto o_T_bs = layout.static_sensor_idx + Traits<Camera>::kTransformationOffset;
  const auto o_i = layout.static_sensor_idx + Traits<Camera>::kIntrinsicsOffset;
  const auto o_d = layout.static_sensor_idx + Traits<Camera>::kDistortionOffset;
  const auto o_p_w = layout.static_observation_idx + Traits<VariableLandmark<Position>>::kVariableOffset;

  // Map parameters.
  const auto T_bs = Eigen::Map<const Transformation>{p_ps[o_T_bs]};
  const auto c_i = Eigen::Map<const Intrinsics>{p_ps[o_i]};
  const auto c_d = measurement.sensor().as<Camera>().distortion().map(p_ps[o_d]);
  const auto p_w = Eigen::Map<const Position>{p_ps[o_p_w]};

  if (!J_e) {
    const auto q_S_wb = StateQuery{measurement.stamp(), kValueIndex};
    const auto S_wb = state.evaluate(q_S_wb, p_S_wb_0);

    const auto T_ws = S_wb.derivativeAs<Manifold>(kValueIndex).groupPlus(T_bs);
    const auto T_sw = T_ws.groupInverse();
    const auto p_s = T_sw.vectorPlus(p_w);
    const auto n_px_s = Camera::ProjectToPlane(p_s);
    const auto d_n_px_s = c_d->distort(n_px_s, nullptr, nullptr);
    auto d_px_s = c_i.denormalize(d_n_px_s);

    return d_px_s;

  } else {
    DCHECK(state.interpolator() != nullptr);
    const auto p_S_wb_1 = p_S_wb_0 + state.interpolator()->layout().outer.size() + 1;
    const auto J_S_wb = std::any_of(p_S_wb_0, p_S_wb_1, [](const auto& arg) { return arg != nullptr; });
    const auto q_S_wb = StateQuery{measurement.stamp(), kValueIndex, J_S_wb};
    const auto S_wb = state.evaluate(q_S_wb, p_S_wb_0);

    Jacobian<Tangent> J_T_ws_S_wb, J_T_ws_T_bs;
    const auto T_ws = S_wb.derivativeAs<Manifold>(kValueIndex).groupPlus(T_bs, J_T_ws_S_wb.data(), J_T_ws_T_bs.data());

    Jacobian<Tangent> J_T_sw_T_ws;
    const auto T_sw = T_ws.groupInverse(J_T_sw_T_ws.data());

    Jacobian<Position, Tangent> J_p_s_T_sw;
    auto p_s = T_sw.vectorPlus(p_w, J_p_s_T_sw.data());

    Jacobian<Pixel, Position> J_n_px_s_p_s;
    const auto n_px_s = Camera::ProjectToPlane(p_s, J_n_px_s_p_s.data());

    Pixel d_px_s;
    Jacobian<Pixel> J_r_n_px_s;
    if (p_js[o_d] && p_js[o_i]) {
      const auto [_, size] = c_d->memory();
      Jacobian<Pixel> J_d_n_px_s_n_px_s;
      DynamicInputJacobian<Pixel> J_d_n_px_s_d{Traits<Pixel>::kNumParameters, size};
      const auto d_n_px_s = c_d->distort(n_px_s, J_d_n_px_s_n_px_s.data(), J_d_n_px_s_d.data());

      Jacobian<Pixel> J_r_d_n_px_s;
      Jacobian<Pixel, Intrinsics> J_d_px_s_c_i;
      d_px_s = c_i.denormalize(d_n_px_s, J_r_d_n_px_s.data(), J_d_px_s_c_i.data());

      J_e->middleCols(offsets[o_d], sizes[o_d]).noalias() = J_r_d_n_px_s * J_d_n_px_s_d;
      J_e->middleCols(offsets[o_i], sizes[o_i]).noalias() = J_d_px_s_c_i;
      J_r_n_px_s = J_r_d_n_px_s * J_d_n_px_s_n_px_s;

    } else if (p_js[o_d]) {
      const auto [_, size] = c_d->memory();
      Jacobian<Pixel> J_d_n_px_s_n_px_s;
      DynamicInputJacobian<Pixel> J_d_n_px_s_d{Traits<Pixel>::kNumParameters, size};
      const auto d_n_px_s = c_d->distort(n_px_s, J_d_n_px_s_n_px_s.data(), J_d_n_px_s_d.data());

      Jacobian<Pixel> J_r_d_n_px_s;
      d_px_s = c_i.denormalize(d_n_px_s, J_r_d_n_px_s.data(), nullptr);

      J_e->middleCols(offsets[o_d], sizes[o_d]).noalias() = J_r_d_n_px_s * J_d_n_px_s_d;
      J_r_n_px_s = J_r_d_n_px_s * J_d_n_px_s_n_px_s;

    } else if (p_js[o_i]) {
      Jacobian<Pixel> J_d_n_px_s_n_px_s;
      const auto d_n_px_s = c_d->distort(n_px_s, J_d_n_px_s_n_px_s.data(), nullptr);

      Jacobian<Pixel> J_r_d_n_px_s;
      Jacobian<Pixel, Intrinsics> J_d_px_s_c_i;
      d_px_s = c_i.denormalize(d_n_px_s, J_r_d_n_px_s.data(), J_d_px_s_c_i.data());

      J_e->middleCols(offsets[o_i], sizes[o_i]).noalias() = J_d_px_s_c_i;
      J_r_n_px_s = J_r_d_n_px_s * J_d_n_px_s_n_px_s;

    } else {
      Jacobian<Pixel> J_d_n_px_s_n_px_s;
      const auto d_n_px_s = c_d->distort(n_px_s, J_d_n_px_s_n_px_s.data(), nullptr);

      Jacobian<Pixel> J_r_d_n_px_s;
      d_px_s = c_i.denormalize(d_n_px_s, J_r_d_n_px_s.data(), nullptr);
      J_r_n_px_s = J_r_d_n_px_s * J_d_n_px_s_n_px_s;
    }

    const auto J_r_p_s = (J_r_n_px_s * J_n_px_s_p_s).eval();
    const auto J_r_T_ws = (J_r_p_s * J_p_s_T_sw * J_T_sw_T_ws).eval();

    if (J_S_wb) J_e->middleCols(offsets[layout.static_state_idx], S_wb.jacobians[kValueIndex].cols()).noalias() = J_r_T_ws * J_T_ws_S_wb * S_wb.jacobians[kValueIndex];
    if (p_js[o_T_bs]) J_e->middleCols(offsets[o_T_bs], sizes[o_T_bs]).noalias() = J_r_T_ws * J_T_ws_T_bs * SE3JacobianAdapter(T_bs.data());
    if (p_js[o_p_w]) J_e->middleCols(offsets[o_p_w], sizes[o_p_w]).noalias() = J_r_p_s * T_sw.rotation().matrix();

    return d_px_s;
  }
}

} // namespace hyper
