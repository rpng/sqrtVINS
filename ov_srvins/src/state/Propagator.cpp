/*
 * Sqrt-VINS: A Sqrt-filter-based Visual-Inertial Navigation System
 * Copyright (C) 2025-2026 Yuxiang Peng
 * Copyright (C) 2025-2026 Chuchu Chen
 * Copyright (C) 2025-2026 Kejian Wu
 * Copyright (C) 2018-2026 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program. If not, see
 * <https://www.gnu.org/licenses/>.
 */





#include "Propagator.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/DataType.h"
#include "utils/Helper.h"
#include "utils/quat_ops.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_srvins;

Propagator::Propagator(NoiseManager noises, DataType gravity_mag)
    : noises_(noises) {
  noises_.sigma_w_2 = std::pow(noises_.sigma_w, 2);
  noises_.sigma_a_2 = std::pow(noises_.sigma_a, 2);
  noises_.sigma_wb_2 = std::pow(noises_.sigma_wb, 2);
  noises_.sigma_ab_2 = std::pow(noises_.sigma_ab, 2);
  last_prop_time_offset_ = 0.0;
  gravity_ << 0.0, 0.0, gravity_mag;
}

void Propagator::feed_imu(const ov_core::ImuData &message, double oldest_time) {

  // Append it to our vector
  std::lock_guard<std::mutex> lck(imu_data_mtx_);
  imu_data_.emplace_back(message);

  // Clean old measurements
  clean_old_imu_measurements(oldest_time);
}

void Propagator::clean_old_imu_measurements(double oldest_time) {
  if (oldest_time < 0)
    return;
  auto it0 = imu_data_.begin();
  while (it0 != imu_data_.end()) {
    if (it0->timestamp < oldest_time) {
      it0 = imu_data_.erase(it0);
    } else {
      it0++;
    }
  }
}

bool Propagator::fast_state_propagate(
    std::shared_ptr<State> state, double timestamp,
    Eigen::Matrix<DataType, 13, 1> &state_plus,
    Eigen::Matrix<DataType, 12, 12> &covariance) {

  // First we will store the current calibration / estimates of the state
  double state_time = state->timestamp;
  MatX state_est = state->imu->value();
  MatX U = StateHelper::get_marginal_U(state, {state->imu});
  MatX state_covariance = U.transpose() * U;
  DataType t_off = state->calib_dt_CAMtoIMU->value()(0);

  // First lets construct an IMU vector of measurements we need
  double time0 = state_time + t_off;
  double time1 = timestamp + t_off;
  std::vector<ov_core::ImuData> prop_data;
  {
    std::lock_guard<std::mutex> lck(imu_data_mtx_);
    prop_data = select_imu_readings(imu_data_, time0, time1, false);
  }
  if (prop_data.size() < 2)
    return false;

  // Biases
  Vec3 bias_g = state_est.block(10, 0, 3, 1);
  Vec3 bias_a = state_est.block(13, 0, 3, 1);

  // Loop through all IMU messages, and use them to move the state forward in
  // time This uses the zero'th order quat, and then constant acceleration
  // discrete
  for (size_t i = 0; i < prop_data.size() - 1; i++) {

    // Corrected imu measurements
    double dt = prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp;
    Vec3 w_hat = 0.5 * (prop_data.at(i + 1).wm + prop_data.at(i).wm) - bias_g;
    Vec3 a_hat = 0.5 * (prop_data.at(i + 1).am + prop_data.at(i).am) - bias_a;
    Mat3 R_Gtoi = quat_2_Rot(state_est.block(0, 0, 4, 1));
    Vec3 v_iinG = state_est.block(7, 0, 3, 1);
    Vec3 p_iinG = state_est.block(4, 0, 3, 1);

    // State transition and noise matrix
    Eigen::Matrix<DataType, 15, 15> F = Eigen::Matrix<DataType, 15, 15>::Zero();
    Eigen::Matrix<DataType, 15, 15> Qd =
        Eigen::Matrix<DataType, 15, 15>::Zero();
    F.block(0, 0, 3, 3) = exp_so3(-w_hat * dt);
    F.block(0, 9, 3, 3).noalias() =
        -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
    F.block(9, 9, 3, 3).setIdentity();
    F.block(6, 0, 3, 3).noalias() = -R_Gtoi.transpose() * skew_x(a_hat * dt);
    F.block(6, 6, 3, 3).setIdentity();
    F.block(6, 12, 3, 3) = -R_Gtoi.transpose() * dt;
    F.block(12, 12, 3, 3).setIdentity();
    F.block(3, 0, 3, 3).noalias() =
        -0.5 * R_Gtoi.transpose() * skew_x(a_hat * dt * dt);
    F.block(3, 6, 3, 3) = Mat3::Identity() * dt;
    F.block(3, 12, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
    F.block(3, 3, 3, 3).setIdentity();
    Eigen::Matrix<DataType, 15, 12> G = Eigen::Matrix<DataType, 15, 12>::Zero();
    G.block(0, 0, 3, 3) = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
    G.block(6, 3, 3, 3) = -R_Gtoi.transpose() * dt;
    G.block(3, 3, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
    G.block(9, 6, 3, 3).setIdentity();
    G.block(12, 9, 3, 3).setIdentity();

    // Construct our discrete noise covariance matrix
    // Note that we need to convert our continuous time noises to discrete
    // Equations (129) amd (130) of Trawny tech report
    Eigen::Matrix<DataType, 12, 12> Qc =
        Eigen::Matrix<DataType, 12, 12>::Zero();
    Qc.block(0, 0, 3, 3) = noises_.sigma_w_2 / dt * Mat3::Identity();
    Qc.block(3, 3, 3, 3) = noises_.sigma_a_2 / dt * Mat3::Identity();
    Qc.block(6, 6, 3, 3) = noises_.sigma_wb_2 * dt * Mat3::Identity();
    Qc.block(9, 9, 3, 3) = noises_.sigma_ab_2 * dt * Mat3::Identity();
    Qd = G * Qc * G.transpose();
    Qd = 0.5 * (Qd + Qd.transpose());
    state_covariance = F * state_covariance * F.transpose() + Qd;

    // Propagate the mean forward
    state_est.block(0, 0, 4, 1) = rot_2_quat(exp_so3(-w_hat * dt) * R_Gtoi);
    state_est.block(4, 0, 3, 1) = p_iinG + v_iinG * dt +
                                  0.5 * R_Gtoi.transpose() * a_hat * dt * dt -
                                  0.5 * gravity_ * dt * dt;
    state_est.block(7, 0, 3, 1) =
        v_iinG + R_Gtoi.transpose() * a_hat * dt - gravity_ * dt;
  }

  // Now record what the predicted state should be
  Vec4 q_Gtoi = state_est.block(0, 0, 4, 1);
  Vec3 v_iinG = state_est.block(7, 0, 3, 1);
  Vec3 p_iinG = state_est.block(4, 0, 3, 1);
  state_plus.setZero();
  state_plus.block(0, 0, 4, 1) = q_Gtoi;
  state_plus.block(4, 0, 3, 1) = p_iinG;
  state_plus.block(7, 0, 3, 1) = quat_2_Rot(q_Gtoi) * v_iinG;
  state_plus.block(10, 0, 3, 1) =
      0.5 * (prop_data.at(prop_data.size() - 1).wm +
             prop_data.at(prop_data.size() - 2).wm) -
      bias_g;

  // Do a covariance propagation for our velocity
  // TODO: more properly do the covariance of the angular velocity here...
  // TODO: it should be dependent on the state bias, thus correlated with the
  // pose
  covariance.setZero();
  Eigen::Matrix<DataType, 15, 15> Phi =
      Eigen::Matrix<DataType, 15, 15>::Identity();
  Phi.block(6, 6, 3, 3) = quat_2_Rot(q_Gtoi);
  state_covariance = Phi * state_covariance * Phi.transpose();
  covariance.block(0, 0, 9, 9) = state_covariance.block(0, 0, 9, 9);
  double dt = prop_data.at(prop_data.size() - 1).timestamp -
              prop_data.at(prop_data.size() - 2).timestamp;
  covariance.block(9, 9, 3, 3) = noises_.sigma_w_2 / dt * Mat3::Identity();
  return true;
}

void Propagator::predict_and_compute(std::shared_ptr<State> state,
                                     const ov_core::ImuData &data_minus,
                                     const ov_core::ImuData &data_plus,
                                     Eigen::Matrix<DataType, 15, 15> &F,
                                     Eigen::Matrix<DataType, 15, 15> &Qd) {

  // Set them to zero
  F.setZero();
  Qd.setIdentity();

  // Time elapsed over interval
  double dt = data_plus.timestamp - data_minus.timestamp;
  // assert(data_plus.timestamp>data_minus.timestamp);

  // Corrected imu measurements
  Eigen::Matrix<DataType, 3, 1> w_hat = data_minus.wm - state->imu->bias_g();
  Eigen::Matrix<DataType, 3, 1> a_hat = data_minus.am - state->imu->bias_a();
  Eigen::Matrix<DataType, 3, 1> w_hat2 = data_plus.wm - state->imu->bias_g();
  Eigen::Matrix<DataType, 3, 1> a_hat2 = data_plus.am - state->imu->bias_a();

  // Compute the new state mean value
  Vec4 new_q;
  Vec3 new_v, new_p;
  if (state->options.use_rk4_integration)
    predict_mean_rk4(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v,
                     new_p);
  else
    predict_mean_discrete(state, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v,
                          new_p);

  // Get the locations of each entry of the imu state
  int th_id = state->imu->q()->id() - state->imu->id();
  int p_id = state->imu->p()->id() - state->imu->id();
  int v_id = state->imu->v()->id() - state->imu->id();
  int bg_id = state->imu->bg()->id() - state->imu->id();
  int ba_id = state->imu->ba()->id() - state->imu->id();

  // Now compute Jacobian of new state wrt old state and noise
  if (state->options.do_fej) {

    // This is the change in the orientation from the end of the last prop to
    // the current prop This is needed since we need to include the "k-th"
    // updated orientation information
    Eigen::Matrix<DataType, 3, 3> Rfej = state->imu->Rot_fej();
    Eigen::Matrix<DataType, 3, 3> dR = quat_2_Rot(new_q) * Rfej.transpose();

    Eigen::Matrix<DataType, 3, 1> v_fej = state->imu->vel_fej();
    Eigen::Matrix<DataType, 3, 1> p_fej = state->imu->pos_fej();

    F.block(th_id, th_id, 3, 3) = dR;
    F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-w_hat * dt) * dt;
    // F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-log_so3(dR)) * dt;
    F.block(bg_id, bg_id, 3, 3).setIdentity();
    F.block(v_id, th_id, 3, 3).noalias() =
        -skew_x(new_v - v_fej + gravity_ * dt) * Rfej.transpose();
    // F.block(v_id, th_id, 3, 3).noalias() = -Rfej.transpose() *
    // skew_x(Rfej*(new_v-v_fej+_gravity*dt));
    F.block(v_id, v_id, 3, 3).setIdentity();
    F.block(v_id, ba_id, 3, 3) = -Rfej.transpose() * dt;
    F.block(ba_id, ba_id, 3, 3).setIdentity();
    F.block(p_id, th_id, 3, 3).noalias() =
        -skew_x(new_p - p_fej - v_fej * dt + 0.5 * gravity_ * dt * dt) *
        Rfej.transpose();
    F.block(p_id, v_id, 3, 3) = Eigen::Matrix<DataType, 3, 3>::Identity() * dt;
    F.block(p_id, ba_id, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
    F.block(p_id, p_id, 3, 3).setIdentity();

    // Construct our discrete noise covariance matrix
    // Note that we need to convert our continuous time noises to discrete
    // Equations (129) amd (130) of Trawny tech report
    Qd.block(0, 0, 3, 3).noalias() = dR * Jr_so3(-w_hat * dt);
    Qd.block(0, 0, 3, 3) =
        Qd.block(0, 0, 3, 3) * Qd.block(0, 0, 3, 3).transpose();
    Qd.block(0, 0, 3, 3) *= noises_.sigma_w_2 * dt;
    Qd.block(3, 3, 3, 3) *= (0.25 * noises_.sigma_a_2 * pow(dt, 3));
    Qd.block(3, 6, 3, 3) *= (0.5 * noises_.sigma_a_2 * pow(dt, 2));
    Qd.block(6, 6, 3, 3) *= (noises_.sigma_a_2 * dt);
    Qd.block(9, 9, 3, 3) *= (noises_.sigma_wb_2 * dt);
    Qd.block(12, 12, 3, 3) *= (noises_.sigma_ab_2 * dt);
  } else {

    Eigen::Matrix<DataType, 3, 3> R_Gtoi = state->imu->Rot();

    F.block(th_id, th_id, 3, 3) = exp_so3(-w_hat * dt);
    F.block(th_id, bg_id, 3, 3).noalias() =
        -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
    F.block(bg_id, bg_id, 3, 3).setIdentity();
    F.block(v_id, th_id, 3, 3).noalias() =
        -R_Gtoi.transpose() * skew_x(a_hat * dt);
    F.block(v_id, v_id, 3, 3).setIdentity();
    F.block(v_id, ba_id, 3, 3) = -R_Gtoi.transpose() * dt;
    F.block(ba_id, ba_id, 3, 3).setIdentity();
    F.block(p_id, th_id, 3, 3).noalias() =
        -0.5 * R_Gtoi.transpose() * skew_x(a_hat * dt * dt);
    F.block(p_id, v_id, 3, 3) = Eigen::Matrix<DataType, 3, 3>::Identity() * dt;
    F.block(p_id, ba_id, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
    F.block(p_id, p_id, 3, 3).setIdentity();

    // Construct our discrete noise covariance matrix
    // Note that we need to convert our continuous time noises to discrete
    // Equations (129) amd (130) of Trawny tech report
    Qd.block(0, 0, 3, 3).noalias() = exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt);
    Qd.block(0, 0, 3, 3) =
        Qd.block(0, 0, 3, 3) * Qd.block(0, 0, 3, 3).transpose();
    Qd.block(0, 0, 3, 3) *= noises_.sigma_w_2 * dt;
    Qd.block(3, 3, 3, 3) *= (0.25 * noises_.sigma_a_2 * pow(dt, 3));
    Qd.block(3, 6, 3, 3) *= (0.5 * noises_.sigma_a_2 * pow(dt, 2));
    Qd.block(6, 6, 3, 3) *= (noises_.sigma_a_2 * dt);
    Qd.block(9, 9, 3, 3) *= (noises_.sigma_wb_2 * dt);
    Qd.block(12, 12, 3, 3) *= (noises_.sigma_ab_2 * dt);
  }

  // Compute the noise injected into the state over the interval
  Qd = Qd.selfadjointView<Eigen::Upper>();

  // Now replace imu estimate and fej with propagated values
  Eigen::Matrix<DataType, 16, 1> imu_x = state->imu->value();
  imu_x.block(0, 0, 4, 1) = new_q;
  imu_x.block(4, 0, 3, 1) = new_p;
  imu_x.block(7, 0, 3, 1) = new_v;
  state->imu->set_value(imu_x);
  state->imu->set_fej(imu_x);
}

void Propagator::predict_mean_discrete(std::shared_ptr<State> state, double dt,
                                       const Vec3 &w_hat1, const Vec3 &a_hat1,
                                       const Vec3 &w_hat2, const Vec3 &a_hat2,
                                       Vec4 &new_q, Vec3 &new_v, Vec3 &new_p) {

  // If we are averaging the IMU, then do so
  Vec3 w_hat = w_hat1;
  Vec3 a_hat = a_hat1;
  if (state->options.imu_avg) {
    w_hat = .5 * (w_hat1 + w_hat2);
    a_hat = .5 * (a_hat1 + a_hat2);
  }

  // Pre-compute things
  DataType w_norm = w_hat.norm();
  Eigen::Matrix<DataType, 4, 4> I_4x4 =
      Eigen::Matrix<DataType, 4, 4>::Identity();
  Eigen::Matrix<DataType, 3, 3> R_Gtoi = state->imu->Rot();

  // Orientation: Equation (101) and (103) and of Trawny indirect TR
  Eigen::Matrix<DataType, 4, 4> bigO;
  if (w_norm > 1e-20) {
    bigO = cos(0.5 * w_norm * dt) * I_4x4 +
           1 / w_norm * sin(0.5 * w_norm * dt) * Omega(w_hat);
  } else {
    bigO = I_4x4 + 0.5 * dt * Omega(w_hat);
  }
  new_q = quatnorm(bigO * state->imu->quat());
  // new_q = rot_2_quat(exp_so3(-w_hat*dt)*R_Gtoi);

  // Velocity: just the acceleration in the local frame, minus global gravity
  new_v = state->imu->vel() + R_Gtoi.transpose() * a_hat * dt - gravity_ * dt;

  // Position: just velocity times dt, with the acceleration integrated twice
  new_p = state->imu->pos() + state->imu->vel() * dt +
          0.5 * R_Gtoi.transpose() * a_hat * dt * dt - 0.5 * gravity_ * dt * dt;
}

void Propagator::predict_mean_rk4(std::shared_ptr<State> state, double dt,
                                  const Vec3 &w_hat1, const Vec3 &a_hat1,
                                  const Vec3 &w_hat2, const Vec3 &a_hat2,
                                  Vec4 &new_q, Vec3 &new_v, Vec3 &new_p) {

  // Pre-compute things
  Vec3 w_hat = w_hat1;
  Vec3 a_hat = a_hat1;
  Vec3 w_alpha = (w_hat2 - w_hat1) / dt;
  Vec3 a_jerk = (a_hat2 - a_hat1) / dt;

  // y0 ================
  Vec4 q_0 = state->imu->quat();
  Vec3 p_0 = state->imu->pos();
  Vec3 v_0 = state->imu->vel();

  // k1 ================
  Vec4 dq_0 = {0, 0, 0, 1};
  Vec4 q0_dot = 0.5 * Omega(w_hat) * dq_0;
  Vec3 p0_dot = v_0;
  Mat3 R_Gto0 = quat_2_Rot(quat_multiply(dq_0, q_0));
  Vec3 v0_dot = R_Gto0.transpose() * a_hat - gravity_;

  Vec4 k1_q = q0_dot * dt;
  Vec3 k1_p = p0_dot * dt;
  Vec3 k1_v = v0_dot * dt;

  // k2 ================
  w_hat += 0.5 * w_alpha * dt;
  a_hat += 0.5 * a_jerk * dt;

  Vec4 dq_1 = quatnorm(dq_0 + 0.5 * k1_q);
  // Vec3 p_1 = p_0+0.5*k1_p;
  Vec3 v_1 = v_0 + 0.5 * k1_v;

  Vec4 q1_dot = 0.5 * Omega(w_hat) * dq_1;
  Vec3 p1_dot = v_1;
  Mat3 R_Gto1 = quat_2_Rot(quat_multiply(dq_1, q_0));
  Vec3 v1_dot = R_Gto1.transpose() * a_hat - gravity_;

  Vec4 k2_q = q1_dot * dt;
  Vec3 k2_p = p1_dot * dt;
  Vec3 k2_v = v1_dot * dt;

  // k3 ================
  Vec4 dq_2 = quatnorm(dq_0 + 0.5 * k2_q);
  // Vec3 p_2 = p_0+0.5*k2_p;
  Vec3 v_2 = v_0 + 0.5 * k2_v;

  Vec4 q2_dot = 0.5 * Omega(w_hat) * dq_2;
  Vec3 p2_dot = v_2;
  Mat3 R_Gto2 = quat_2_Rot(quat_multiply(dq_2, q_0));
  Vec3 v2_dot = R_Gto2.transpose() * a_hat - gravity_;

  Vec4 k3_q = q2_dot * dt;
  Vec3 k3_p = p2_dot * dt;
  Vec3 k3_v = v2_dot * dt;

  // k4 ================
  w_hat += 0.5 * w_alpha * dt;
  a_hat += 0.5 * a_jerk * dt;

  Vec4 dq_3 = quatnorm(dq_0 + k3_q);
  // Vec3 p_3 = p_0+k3_p;
  Vec3 v_3 = v_0 + k3_v;

  Vec4 q3_dot = 0.5 * Omega(w_hat) * dq_3;
  Vec3 p3_dot = v_3;
  Mat3 R_Gto3 = quat_2_Rot(quat_multiply(dq_3, q_0));
  Vec3 v3_dot = R_Gto3.transpose() * a_hat - gravity_;

  Vec4 k4_q = q3_dot * dt;
  Vec3 k4_p = p3_dot * dt;
  Vec3 k4_v = v3_dot * dt;

  // y+dt ================
  Vec4 dq = quatnorm(dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q +
                     (1.0 / 3.0) * k3_q + (1.0 / 6.0) * k4_q);
  new_q = quat_multiply(dq, q_0);
  new_p = p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p +
          (1.0 / 6.0) * k4_p;
  new_v = v_0 + (1.0 / 6.0) * k1_v + (1.0 / 3.0) * k2_v + (1.0 / 3.0) * k3_v +
          (1.0 / 6.0) * k4_v;
}

void Propagator::propagate_and_clone(std::shared_ptr<State> state,
                                     double timestamp) {

  // If the difference between the current update time and state is zero
  // We should crash, as this means we would have two clones at the same
  // time!!!!
  if (state->timestamp == timestamp &&
      state->clones_IMU.find(state->timestamp) != state->clones_IMU.end()) {
    PRINT_ERROR(RED "Propagator::propagate_and_clone(): Two same clone in the "
                    "state!!!!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // We should crash if we are trying to propagate backwards
  if (state->timestamp > timestamp) {
    PRINT_ERROR(RED "Propagator::propagate_and_clone(): Propagation called "
                    "trying to propagate backwards in time!!!!\n" RESET);
    PRINT_ERROR(
        RED
        "Propagator::propagate_and_clone(): desired propagation = %.4f\n" RESET,
        (timestamp - state->timestamp));
    std::exit(EXIT_FAILURE);
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================
  if (state->timestamp != timestamp) {
  }
  // Set the last time offset value if we have just started the system up
  if (!have_last_prop_time_offset_) {
    last_prop_time_offset_ = state->calib_dt_CAMtoIMU->value()(0);
    have_last_prop_time_offset_ = true;
  }

  // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
  double t_off_new = state->calib_dt_CAMtoIMU->value()(0);

  // First lets construct an IMU vector of measurements we need
  double time0 = state->timestamp + last_prop_time_offset_;
  double time1 = timestamp + t_off_new;
  std::vector<ov_core::ImuData> prop_data;
  {
    std::lock_guard<std::mutex> lck(imu_data_mtx_);
    prop_data = select_imu_readings(imu_data_, time0, time1);
  }

  // We are going to sum up all the state transition matrices, so we can do a
  // single large multiplication at the end Phi_summed = Phi_i*Phi_summed
  // Q_summed = Phi_i*Q_summed*Phi_i^T + Q_i
  // After summing we can multiple the total phi to get the updated covariance
  // We will then add the noise to the IMU portion of the state
  Eigen::Matrix<DataType, 15, 15> Phi_summed =
      Eigen::Matrix<DataType, 15, 15>::Identity();
  Eigen::Matrix<DataType, 15, 15> Qd_summed =
      Eigen::Matrix<DataType, 15, 15>::Zero();
  double dt_summed = 0;
  MatX R = MatX::Zero(27, 15);

  // Loop through all IMU messages, and use them to move the state forward in
  // time This uses the zero'th order quat, and then constant acceleration
  // discrete
  Timer t1, t2;
  if (prop_data.size() > 1) {
    for (size_t i = 0; i < prop_data.size() - 1; i++) {

      // Get the next state Jacobian and noise Jacobian for this IMU reading
      Eigen::Matrix<DataType, 15, 15> F =
          Eigen::Matrix<DataType, 15, 15>::Zero();
      Eigen::Matrix<DataType, 15, 15> Qd;
      predict_and_compute(state, prop_data.at(i), prop_data.at(i + 1), F, Qd);

      // Next we should propagate our IMU covariance
      // Pii' = F*Pii*F.transpose() + G*Q*G.transpose()
      // Pci' = F*Pci and Pic' = Pic*F.transpose()
      // NOTE: Here we are summing the state transition F so we can do a single
      // mutiplication later NOTE: Phi_summed = Phi_i*Phi_summed NOTE: Q_summed
      // = Phi_i*Q_summed*Phi_i^T + G*Q_i*G^T
      Phi_summed = F * Phi_summed;
      Qd_summed.block(0, 0, 9, 15) = F.block(0, 0, 9, 15) * Qd_summed;
      Qd_summed.block(0, 0, 9, 9) =
          Qd_summed.block(0, 0, 9, 15) * F.transpose().block(0, 0, 15, 9);

      Qd_summed += Qd;
      Qd_summed = Qd_summed.selfadjointView<Eigen::Upper>();
      dt_summed += prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp;
    }
  }

  Qd_summed = Qd_summed.llt().matrixU();
  assert(std::abs((time1 - time0) - dt_summed) < 1e-4);

  // Do the update to the covariance with our "summed" state transition and IMU
  // noise addition...
  StateHelper::propagate(state, Phi_summed, Qd_summed);

  // Set timestamp data
  state->update_timestamp(timestamp);
  last_prop_time_offset_ = t_off_new;

  // Get U before it becomes rank-deficient
  // Clone before Propagation
  // Call on our cloner and add it to our vector of types
  // NOTE: this will clone the clone pose to the END of the covariance...
  std::shared_ptr<Type> posetemp =
      StateHelper::clone(state, state->imu->pose()->clone());

  // Cast to a JPL pose type, check if valid
  std::shared_ptr<PoseJPL> pose = std::dynamic_pointer_cast<PoseJPL>(posetemp);
  if (pose == nullptr) {
    PRINT_ERROR(RED "INVALID OBJECT RETURNED FROM STATEHELPER CLONE, "
                    "EXITING!#!@#!@#\n" RESET);
    std::exit(EXIT_FAILURE);
  }
  // Append the new clone to our clone vector
  state->clones_IMU[state->timestamp] = pose;

  // Do timeoffset calibraton
  if (state->options.do_calib_camera_timeoffset) {
    // Last angular velocity (used for cloning when estimating time offset)
    Eigen::Matrix<DataType, 3, 1> last_w =
        Eigen::Matrix<DataType, 3, 1>::Zero();
    if (prop_data.size() > 1) {
      last_w = prop_data.at(prop_data.size() - 2).wm - state->imu->bias_g();
    } else if (!prop_data.empty()) {
      last_w = prop_data.at(prop_data.size() - 1).wm - state->imu->bias_g();
    }

    Eigen::Matrix<DataType, 6, 1> dnc_dt =
        Eigen::Matrix<DataType, 6, 1>::Zero();
    dnc_dt.block(0, 0, 3, 1) = last_w;
    dnc_dt.block(3, 0, 3, 1) = state->imu->vel();
    StateHelper::propagate_timeoffset(state, dnc_dt);
  }
}