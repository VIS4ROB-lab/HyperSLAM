/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/environment/observations/inertial.hpp"
#include "hyper/optimizers/evaluators/evaluator.hpp"
#include "hyper/sensors/imu.hpp"
#include "hyper/variables/adapters.hpp"
#include "hyper/variables/gravity.hpp"

namespace hyper {

template <>
auto InertialEvaluator<SE3<Scalar>>::evaluate(const EvaluatorQuery<Scalar, Index>& query) const -> DynamicVector<Scalar> {
  // Definitions.
  using Gravity = Gravity<Scalar>;
  using GyroscopeIntrinsics = Traits<IMU>::GyroscopeIntrinsics;
  using AccelerometerIntrinsics = Traits<IMU>::AccelerometerIntrinsics;
  using Tangent = Tangent<Manifold>;

  //  Unpack arguments.
  const auto& [pointers, layout, context] = query;
  const auto& [p_ps, p_J_e, p_js] = pointers;
  const auto& [num_parameters, indices, offsets, sizes] = layout;
  const auto& [state, observation] = context;

  // Retrieve measurement and sensor.
  const auto& measurement = context.observation->measurement();
  const auto& imu = measurement.sensor().as<IMU>();

  // Retrieve stamp.
  const auto& stamp = measurement.stamp();

  // Compute offsets.
  const auto num_gyroscope_bias_parameters = imu.gyroscopeBias().interpolator()->layout().outer.size;
  const auto o_T_bs = indices.static_sensor_idx + Traits<IMU>::kTransformationOffset;
  const auto o_i_g = indices.static_sensor_idx + Traits<IMU>::kGyroscopeIntrinsicsOffset;
  const auto o_i_a = indices.static_sensor_idx + Traits<IMU>::kAccelerometerIntrinsicsOffset;
  const auto o_S_g = indices.static_sensor_idx + Traits<IMU>::kGyroscopeSensitivityOffset;
  const auto o_X_a = indices.static_sensor_idx + Traits<IMU>::kAccelerometerAxesOffsetsOffset;
  const auto o_b_g = indices.dynamic_sensor_idx;
  const auto o_b_a = o_b_g + num_gyroscope_bias_parameters;
  const auto o_g_w = indices.static_observation_idx;

  // Map parameters.
  const auto T_bs = Eigen::Map<const SE3<Scalar>>{p_ps[o_T_bs]};
  const auto i_g = Eigen::Map<const GyroscopeIntrinsics>{p_ps[o_i_g]};
  const auto i_a = Eigen::Map<const AccelerometerIntrinsics>{p_ps[o_i_a]};
  const auto S_g = Eigen::Map<const Eigen::Matrix<Scalar, 3, 3>>{p_ps[o_S_g]}; // TODO Improve.
  const auto X_a = Eigen::Map<const Eigen::Matrix<Scalar, 3, 3>>{p_ps[o_X_a]}; // TODO Improve.
  const auto g_w = Eigen::Map<const Gravity>{p_ps[o_g_w]};

  if (!p_J_e) {
    // Evaluate state.
    const auto q_S_wb = StateQuery{stamp, kAccelerationIndex};
    const auto S_wb = state->evaluate(q_S_wb, p_ps + indices.static_state_idx);

    // Evaluate Biases.
    const auto q_S_b = StateQuery{stamp, kValueIndex};
    const auto b_g = imu.gyroscopeBias().evaluate(q_S_b, p_ps + o_b_g);
    const auto b_a = imu.accelerometerBias().evaluate(q_S_b, p_ps + o_b_a);

    const auto T_wb = S_wb.derivativeAs<Manifold>(kValueIndex);
    const auto V_wb = S_wb.derivativeAs<Tangent>(kVelocityIndex);
    const auto A_wb = S_wb.derivativeAs<Tangent>(kAccelerationIndex);

    const auto R_bw = T_wb.rotation().inverse().matrix();
    const auto R_sb = T_bs.rotation().inverse().matrix();

    const auto w_b = V_wb.angular();
    const auto w_b_x = w_b.hat();
    const auto a_b_i = (A_wb.linear() - R_bw * g_w).eval(); // Ideal acceleration.

    const auto F_a = (w_b_x * w_b_x + A_wb.angular().hat()).eval();
    const auto a_b_m = (a_b_i + (F_a * (X_a.colwise() + T_bs.translation())).diagonal()).eval(); // Model acceleration.

    Tangent tangent;
    tangent.angular() = i_g.asMatrix() * R_sb * w_b + S_g * a_b_m + b_g.derivatives[kValueIndex];
    tangent.linear() = i_a.asMatrix() * R_sb * a_b_m + b_a.derivatives[kValueIndex];
    return tangent;

  } else {
    // Definitions.
    using Traits = Traits<Tangent>;

    // Allocate (full) Jacobian.
    p_J_e->setZero(Traits::kNumParameters, num_parameters);

    // Evaluate state.
    DCHECK(state->interpolator() != nullptr);
    const auto p_S_wb_0 = p_ps + indices.static_state_idx;
    const auto p_S_wb_1 = p_S_wb_0 + state->interpolator()->layout().outer.size;
    const auto J_S_wb = std::any_of(p_S_wb_0, p_S_wb_1, [](const auto& arg) { return arg != nullptr; });
    const auto q_S_wb = StateQuery{stamp, kAccelerationIndex, J_S_wb};
    const auto S_wb = state->evaluate(q_S_wb, p_S_wb_0);

    // Evaluate gyroscope bias.
    const auto p_b_g_0 = p_ps + o_b_g;
    const auto p_b_g_1 = p_b_g_0 + num_gyroscope_bias_parameters;
    const auto J_b_g = std::any_of(p_b_g_0, p_b_g_1, [](const auto& arg) { return arg != nullptr; });
    const auto q_S_b_g = StateQuery{stamp, kValueIndex, J_b_g};
    const auto b_g = imu.gyroscopeBias().evaluate(q_S_b_g, p_b_g_0);

    // Evaluate accelerometer bias.
    const auto p_b_a_0 = p_ps + o_b_a;
    const auto p_b_a_1 = p_ps + o_g_w;
    const auto J_b_a = std::any_of(p_b_a_0, p_b_a_1, [](const auto& arg) { return arg != nullptr; });
    const auto q_S_b_a = StateQuery{stamp, kValueIndex, J_b_a};
    const auto b_a = imu.accelerometerBias().evaluate(q_S_b_a, p_b_a_0);

    const auto T_wb = S_wb.derivativeAs<Manifold>(kValueIndex);
    const auto V_wb = S_wb.derivativeAs<Tangent>(kVelocityIndex);
    const auto A_wb = S_wb.derivativeAs<Tangent>(kAccelerationIndex);

    const auto R_bw = T_wb.rotation().inverse().matrix();
    const auto R_sb = T_bs.rotation().inverse().matrix();
    const auto t_bs_x = T_bs.translation().hat();

    const auto I_g = i_g.asMatrix();
    const auto I_a = i_a.asMatrix();
    const auto I_g_R_sb = (I_g * R_sb).eval();
    const auto I_a_R_sb = (I_a * R_sb).eval();

    const auto w_b = V_wb.angular();
    const auto w_b_x = w_b.hat();
    const auto a_b_i = (A_wb.linear() - R_bw * g_w).eval(); // Ideal acceleration.

    const auto F_a = (w_b_x * w_b_x + A_wb.angular().hat()).eval();
    const auto a_b_m = (a_b_i + (F_a * (X_a.colwise() + T_bs.translation())).diagonal()).eval(); // Model acceleration.

    const auto w_s = (R_sb * w_b).eval();
    const auto a_s = (R_sb * a_b_m).eval();

    if (J_S_wb) {
      Jacobian<Tangent> J_value;
      J_value.block<Traits::kNumAngularParameters, Traits::kNumAngularParameters>(Traits::kAngularOffset, Traits::kAngularOffset).setZero();
      J_value.block<Traits::kNumLinearParameters, Traits::kNumAngularParameters>(Traits::kLinearOffset, Traits::kAngularOffset).noalias() = I_g_R_sb * a_b_i.hat() * R_bw;
      J_value.block<Traits::kNumAngularParameters, Traits::kNumLinearParameters>(Traits::kAngularOffset, Traits::kLinearOffset).setZero();
      J_value.block<Traits::kNumLinearParameters, Traits::kNumLinearParameters>(Traits::kLinearOffset, Traits::kLinearOffset).setZero();

      Jacobian<Tangent> J_velocity;
      J_velocity.block<Traits::kNumAngularParameters, Traits::kNumAngularParameters>(Traits::kAngularOffset, Traits::kAngularOffset).noalias() = I_g_R_sb;
      J_velocity.block<Traits::kNumLinearParameters, Traits::kNumAngularParameters>(Traits::kLinearOffset, Traits::kAngularOffset).noalias() = Scalar{-1} * I_g_R_sb * (Scalar{2} * w_b_x * t_bs_x - t_bs_x * w_b_x);
      J_velocity.block<Traits::kNumAngularParameters, Traits::kNumLinearParameters>(Traits::kAngularOffset, Traits::kLinearOffset).setZero();
      J_velocity.block<Traits::kNumLinearParameters, Traits::kNumLinearParameters>(Traits::kLinearOffset, Traits::kLinearOffset).setZero();

      Jacobian<Tangent> J_acceleration;
      J_acceleration.block<Traits::kNumAngularParameters, Traits::kNumAngularParameters>(Traits::kAngularOffset, Traits::kAngularOffset).setZero();
      J_acceleration.block<Traits::kNumLinearParameters, Traits::kNumAngularParameters>(Traits::kLinearOffset, Traits::kAngularOffset).noalias() = Scalar{-1} * I_g_R_sb * t_bs_x;
      J_acceleration.block<Traits::kNumAngularParameters, Traits::kNumLinearParameters>(Traits::kAngularOffset, Traits::kLinearOffset).setZero();
      J_acceleration.block<Traits::kNumLinearParameters, Traits::kNumLinearParameters>(Traits::kLinearOffset, Traits::kLinearOffset).noalias() = I_a_R_sb;

      p_J_e->middleCols(offsets[indices.static_state_idx], S_wb.jacobians[kValueIndex].cols()).noalias() = J_value * S_wb.jacobians[kValueIndex] + J_velocity * S_wb.jacobians[kVelocityIndex] + J_acceleration * S_wb.jacobians[kAccelerationIndex];
    }

    if (p_js[o_T_bs]) {
      Jacobian<Tangent> J_T_bs;
      J_T_bs.block<Traits::kNumAngularParameters, Traits::kNumAngularParameters>(Traits::kAngularOffset, Traits::kAngularOffset).noalias() = I_g * w_s.hat();
      J_T_bs.block<Traits::kNumLinearParameters, Traits::kNumAngularParameters>(Traits::kLinearOffset, Traits::kAngularOffset).noalias() = I_g * a_s.hat();
      J_T_bs.block<Traits::kNumAngularParameters, Traits::kNumLinearParameters>(Traits::kAngularOffset, Traits::kLinearOffset).setZero();
      J_T_bs.block<Traits::kNumLinearParameters, Traits::kNumLinearParameters>(Traits::kLinearOffset, Traits::kLinearOffset).noalias() = I_a_R_sb * F_a;
      p_J_e->middleCols(offsets[o_T_bs], (*sizes)[o_T_bs]).noalias() = J_T_bs * SE3JacobianAdapter(T_bs.data());
    }

    if (p_js[o_i_g]) {
      Jacobian<Tangent::Angular, GyroscopeIntrinsics> J_i_g;
      i_g.align(w_s, nullptr, J_i_g.data());
      p_J_e->block(Traits::kAngularOffset, offsets[o_i_g], Traits::kNumAngularParameters, (*sizes)[o_i_g]).noalias() = J_i_g;
    }

    if (p_js[o_i_a]) {
      Jacobian<Tangent::Linear, AccelerometerIntrinsics> J_i_a;
      i_a.align(a_s, nullptr, J_i_a.data());
      p_J_e->block(Traits::kLinearOffset, offsets[o_i_a], Traits::kNumLinearParameters, (*sizes)[o_i_a]).noalias() = J_i_a;
    }

    if (p_js[o_S_g]) {
      auto J_S_g = p_J_e->block(Traits::kAngularOffset, offsets[o_S_g], Traits::kNumAngularParameters, (*sizes)[o_S_g]);
      J_S_g(0, 0) = a_b_m[0];
      J_S_g(1, 1) = a_b_m[0];
      J_S_g(2, 2) = a_b_m[0];
      J_S_g(0, 3) = a_b_m[1];
      J_S_g(1, 4) = a_b_m[1];
      J_S_g(2, 5) = a_b_m[1];
      J_S_g(0, 6) = a_b_m[2];
      J_S_g(1, 7) = a_b_m[2];
      J_S_g(2, 8) = a_b_m[2];
    }

    if (p_js[o_X_a]) {
      auto J_X_a = p_J_e->block(Traits::kLinearOffset, offsets[o_X_a], Traits::kNumLinearParameters, (*sizes)[o_X_a]);
      J_X_a.block<3, 3>(0, 0).noalias() = I_a_R_sb.col(0) * F_a.row(0);
      J_X_a.block<3, 3>(0, 3).noalias() = I_a_R_sb.col(1) * F_a.row(1);
      J_X_a.block<3, 3>(0, 6).noalias() = I_a_R_sb.col(2) * F_a.row(2);
    }

    if (J_b_g) p_J_e->block(Traits::kAngularOffset, offsets[o_b_g], Traits::kNumAngularParameters, b_g.jacobians[kValueIndex].cols()).noalias() = b_g.jacobians[kValueIndex];
    if (J_b_a) p_J_e->block(Traits::kLinearOffset, offsets[o_b_a], Traits::kNumLinearParameters, b_a.jacobians[kValueIndex].cols()).noalias() = b_a.jacobians[kValueIndex];
    if (p_js[o_g_w]) p_J_e->block(Traits::kLinearOffset, offsets[o_g_w], Traits::kNumLinearParameters, (*sizes)[o_g_w]).noalias() = Scalar{-1} * I_a_R_sb * R_bw;

    Tangent tangent;
    tangent.angular() = I_g_R_sb * w_b + S_g * a_b_m + b_g.derivatives[kValueIndex];
    tangent.linear() = I_a_R_sb * a_b_m + b_a.derivatives[kValueIndex];
    return tangent;
  }
}

} // namespace hyper
