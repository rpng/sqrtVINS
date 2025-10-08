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





#include "UpdaterZeroVelocity.h"

#include "UpdaterHelper.h"

#include "feat/FeatureDatabase.h"
#include "feat/FeatureHelper.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/DataType.h"
#include "utils/Helper.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_srvins;

UpdaterZeroVelocity::UpdaterZeroVelocity(
    UpdaterOptions &options, NoiseManager &noises,
    std::shared_ptr<ov_core::FeatureDatabase> db,
    std::shared_ptr<Propagator> prop, DataType gravity_mag,
    DataType zupt_max_velocity, DataType zupt_noise_multiplier,
    DataType zupt_max_disparity)
    : options_(options), noises_(noises), db_(db), propagator_(prop),
      zupt_max_velocity_(zupt_max_velocity),
      zupt_noise_multiplier_(zupt_noise_multiplier),
      zupt_max_disparity_(zupt_max_disparity) {

  // Gravity
  gravity_ << 0.0, 0.0, gravity_mag;

  // Save our raw pixel noise squared
  noises_.sigma_w_2 = std::pow(noises_.sigma_w, 2);
  noises_.sigma_a_2 = std::pow(noises_.sigma_a, 2);
  noises_.sigma_wb_2 = std::pow(noises_.sigma_wb, 2);
  noises_.sigma_ab_2 = std::pow(noises_.sigma_ab, 2);

  // Initialize the chi squared test table with confidence level 0.95
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 1000; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table_[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

bool UpdaterZeroVelocity::try_update(std::shared_ptr<State> state,
                                     double timestamp) {

  // Return if we don't have any imu data yet
  if (imu_data_.empty()) {
    last_zupt_state_timestamp_ = 0.0;
    return false;
  }

  // Return if the state is already at the desired time
  if (state->timestamp == timestamp) {
    last_zupt_state_timestamp_ = 0.0;
    return false;
  }

  // Set the last time offset value if we have just started the system up
  if (!have_last_prop_time_offset_) {
    last_prop_time_offset_ = state->calib_dt_CAMtoIMU->value()(0);
    have_last_prop_time_offset_ = true;
  }

  // assert that the time we are requesting is in the future
  // assert(timestamp > state->_timestamp);

  // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
  DataType t_off_new = state->calib_dt_CAMtoIMU->value()(0);

  // First lets construct an IMU vector of measurements we need
  // DataType time0 = state->_timestamp+t_off_new;
  double time0 = state->timestamp + last_prop_time_offset_;
  double time1 = timestamp + t_off_new;

  // Select bounding inertial measurements
  std::vector<ov_core::ImuData> imu_recent =
      select_imu_readings(imu_data_, time0, time1);

  // Move forward in time
  last_prop_time_offset_ = t_off_new;

  // Check that we have at least one measurement to propagate with
  if (imu_recent.size() < 2) {
    PRINT_WARNING(RED "[ZUPT]: There are no IMU data to check for zero "
                      "velocity with!!\n" RESET);
    last_zupt_state_timestamp_ = 0.0;
    return false;
  }

  // If we should integrate the acceleration and say the velocity should be zero
  // Also if we should still inflate the bias based on their random walk noises
  bool integrated_accel_constraint = false;
  bool model_time_varying_bias = true;
  bool override_with_disparity_check = true;
  bool explicitly_enforce_zero_motion = false;

  // Order of our Jacobian
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->imu->q());
  Hx_order.push_back(state->imu->bg());
  Hx_order.push_back(state->imu->ba());
  if (integrated_accel_constraint) {
    Hx_order.push_back(state->imu->v());
  }

  // Large final matrices used for update (we will compress these)
  int h_size = (integrated_accel_constraint) ? 12 : 9;
  int m_size = 6 * ((int)imu_recent.size() - 1);
  MatX H = MatX::Zero(m_size, h_size);
  VecX res = VecX::Zero(m_size);

  // Loop through all our IMU and construct the residual and Jacobian
  // State order is: [q_GtoI, bg, ba, v_IinG]
  // Measurement order is: [w_true = 0, a_true = 0 or v_k+1 = 0]
  // w_true = w_m - bw - nw
  // a_true = a_m - ba - R*g - na
  // v_true = v_k - g*dt + R^T*(a_m - ba - na)*dt
  double dt_summed = 0;
  for (size_t i = 0; i < imu_recent.size() - 1; i++) {

    // Precomputed values
    double dt = imu_recent.at(i + 1).timestamp - imu_recent.at(i).timestamp;
    Vec3 a_hat = imu_recent.at(i).am - state->imu->bias_a();

    // Measurement noise (convert from continuous to discrete)
    // NOTE: The dt time might be different if we have "cut" any imu
    // measurements NOTE: We are performing "whittening" thus, we will decompose
    // R_meas^-1 = L*L^t NOTE: This is then multiplied to the residual and
    // Jacobian (equivalent to just updating with R_meas) NOTE: See Maybeck
    // Stochastic Models, Estimation, and Control Vol. 1 Equations
    // (7-21a)-(7-21c)
    DataType w_omega = std::sqrt(dt) / noises_.sigma_w;
    DataType w_accel = std::sqrt(dt) / noises_.sigma_a;
    DataType w_accel_v = 1.0 / (std::sqrt(dt) * noises_.sigma_a);

    // Measurement residual (true value is zero)
    res.block(6 * i + 0, 0, 3, 1) =
        -w_omega * (imu_recent.at(i).wm - state->imu->bias_g());
    if (!integrated_accel_constraint) {
      res.block(6 * i + 3, 0, 3, 1) =
          -w_accel * (a_hat - state->imu->Rot() * gravity_);
    } else {
      assert(false);
      res.block(6 * i + 3, 0, 3, 1) =
          -w_accel_v * (state->imu->vel() - gravity_ * dt +
                        state->imu->Rot().transpose() * a_hat * dt);
    }

    // Measurement Jacobian
    Mat3 R_GtoI_jacob =
        (state->options.do_fej) ? state->imu->Rot_fej() : state->imu->Rot();
    H.block(6 * i + 0, 3, 3, 3) = -w_omega * Mat3::Identity();
    if (!integrated_accel_constraint) {
      H.block(6 * i + 3, 0, 3, 3) = -w_accel * skew_x(R_GtoI_jacob * gravity_);
      H.block(6 * i + 3, 6, 3, 3) = -w_accel * Mat3::Identity();
    } else {
      assert(false);
      H.block(6 * i + 3, 0, 3, 3) =
          -w_accel_v * R_GtoI_jacob.transpose() * skew_x(a_hat) * dt;
      H.block(6 * i + 3, 6, 3, 3) = -w_accel_v * R_GtoI_jacob.transpose() * dt;
      H.block(6 * i + 3, 9, 3, 3) = w_accel_v * Mat3::Identity();
    }
    dt_summed += dt;
  }

  // Multiply our noise matrix by a fixed amount
  // We typically need to treat the IMU as being "worst" to detect / not become
  // over confident
  MatX R = zupt_noise_multiplier_ * MatX::Identity(res.rows(), res.rows());

  // Next propagate the biases forward in time
  // NOTE: G*Qd*G^t = dt*Qd*dt = dt*(1/dt*Qc)*dt = dt*Qc
  MatX Q_bias_sqrt = MatX::Identity(6, 6);
  Q_bias_sqrt.block(0, 0, 3, 3) *= sqrt(dt_summed) * noises_.sigma_wb;
  Q_bias_sqrt.block(3, 3, 3, 3) *= sqrt(dt_summed) * noises_.sigma_ab;

  // Chi2 distance check
  // NOTE: we also append the propagation we "would do before the update" if
  // this was to be accepted (just the bias evolution) NOTE: we don't propagate
  // first since if we fail the chi2 then we just want to return and do normal
  // logic
  MatX U_marg = StateHelper::get_marginal_U(state, Hx_order);
  if (model_time_varying_bias) {
    U_marg.conservativeResizeLike(MatX::Zero(U_marg.rows() + 6, U_marg.cols()));
    U_marg.bottomRows(6).middleCols(3, 6) = Q_bias_sqrt;
  }
  efficient_QR(U_marg);
  U_marg.conservativeResizeLike(MatX::Zero(U_marg.cols(), U_marg.cols()));
  MatX HUT = H * U_marg.transpose().triangularView<Eigen::Lower>();
  MatX S = HUT * HUT.transpose();
  S.diagonal() += zupt_noise_multiplier_ * VecX::Ones(S.rows());
  DataType chi2 = res.dot(S.llt().solve(res));

  // Get our threshold (we precompute up to 1000 but handle the case that it is
  // more)
  DataType chi2_check;
  if (res.rows() < 1000) {
    chi2_check = chi_squared_table_[res.rows()];
  } else {
    boost::math::chi_squared chi_squared_dist(res.rows());
    chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    PRINT_WARNING(YELLOW
                  "[ZUPT]: chi2_check over the residual limit - %d\n" RESET,
                  (int)res.rows());
  }

  // Check if the image disparity
  bool disparity_passed = false;
  if (override_with_disparity_check) {

    // Get the disparity statistics from this image to the previous
    double time0_cam = state->timestamp;
    double time1_cam = timestamp;
    int num_features = 0;
    DataType disp_avg = 0.0;
    DataType disp_var = 0.0;
    FeatureHelper::compute_disparity(db_, time0_cam, time1_cam, disp_avg,
                                     disp_var, num_features);

    // Check if this disparity is enough to be classified as moving
    disparity_passed = (disp_avg < zupt_max_disparity_ && num_features > 20);
    if (disparity_passed) {
      PRINT_INFO(CYAN
                 "[ZUPT]: passed disparity (%.3f < %.3f, %d features)\n" RESET,
                 disp_avg, zupt_max_disparity_, (int)num_features);
    } else {
      PRINT_DEBUG(YELLOW
                  "[ZUPT]: failed disparity (%.3f > %.3f, %d features)\n" RESET,
                  disp_avg, zupt_max_disparity_, (int)num_features);
    }
  }

  // Check if we are currently zero velocity
  // We need to pass the chi2 and not be above our velocity threshold
  if (!disparity_passed && (chi2 > options_.chi2_multipler * chi2_check ||
                            state->imu->vel().norm() > zupt_max_velocity_)) {
    last_zupt_state_timestamp_ = 0.0;
    last_zupt_count_ = 0;
    PRINT_DEBUG(
        YELLOW "[ZUPT]: rejected |v_IinG| = %.3f (chi2 %.3f > %.3f)\n" RESET,
        state->imu->vel().norm(), chi2, options_.chi2_multipler * chi2_check);
    return false;
  }
  PRINT_INFO(CYAN "[ZUPT]: accepted |v_IinG| = %.3f (chi2 %.3f < %.3f)\n" RESET,
             state->imu->vel().norm(), chi2,
             options_.chi2_multipler * chi2_check);

  // Do our update, only do this update if we have previously detected
  // If we have succeeded, then we should remove the current timestamp feature
  // tracks This is because we will not clone at this timestep and instead do
  // our zero velocity update NOTE: We want to keep the tracks from the second
  // time we have called the zv-upt since this won't have a clone NOTE: All
  // future times after the second call to this function will also *not* have a
  // clone, so we can remove those
  if (last_zupt_count_ >= 2) {
    db_->cleanup_measurements_exact(last_zupt_state_timestamp_);
  }

  // Else we are good, update the system
  // 1) update with our IMU measurements directly
  // 2) propagate and then explicitly say that our ori, pos, and vel should be
  // zero
  if (!explicitly_enforce_zero_motion) {

    // Next propagate the biases forward in time
    // NOTE: G*Qd*G^t = dt*Qd*dt = dt*Qc
    if (model_time_varying_bias) {
      StateHelper::propagate_zero_motion(state, dt_summed, noises_.sigma_wb,
                                         noises_.sigma_ab);
    }

    VecX RHTr = VecX::Zero(state->get_state_size());
    int local_id = 0;
    const DataType kNoiseInv = 1.0 / zupt_noise_multiplier_;
    for (const auto &var : Hx_order) {
      RHTr.block(var->id(), 0, var->size(), 1).noalias() +=
          H.block(0, local_id, H.rows(), var->size()).transpose() * res *
          kNoiseInv * kNoiseInv;
      local_id += var->size();
    }
    state->store_update_factor(HUT * kNoiseInv, RHTr);

    // Finally move the state time forward
    StateHelper::update_llt(state);
    state->clear();
    state->update_timestamp(timestamp);
  } else {

    // Propagate the state forward in time
    double time0_cam = last_zupt_state_timestamp_;
    double time1_cam = timestamp;
    propagator_->propagate_and_clone(state, time1_cam);

    // Create the update system!
    H = MatX::Zero(9, 15);
    res = VecX::Zero(9);
    R = MatX::Identity(9, 9);

    // residual (order is ori, pos, vel)
    Mat3 R_GtoI0 = state->clones_IMU.at(time0_cam)->Rot();
    Vec3 p_I0inG = state->clones_IMU.at(time0_cam)->pos();
    Mat3 R_GtoI1 = state->clones_IMU.at(time1_cam)->Rot();
    Vec3 p_I1inG = state->clones_IMU.at(time1_cam)->pos();
    res.block(0, 0, 3, 1) = -log_so3(R_GtoI0 * R_GtoI1.transpose());
    res.block(3, 0, 3, 1) = p_I1inG - p_I0inG;
    res.block(6, 0, 3, 1) = state->imu->vel();
    res *= -1;

    // jacobian (order is q0, p0, q1, p1, v0)
    Hx_order.clear();
    Hx_order.push_back(state->clones_IMU.at(time0_cam));
    Hx_order.push_back(state->clones_IMU.at(time1_cam));
    Hx_order.push_back(state->imu->v());
    if (state->options.do_fej) {
      R_GtoI0 = state->clones_IMU.at(time0_cam)->Rot_fej();
    }
    H.block(0, 0, 3, 3) = Mat3::Identity();
    H.block(0, 6, 3, 3) = -R_GtoI0;
    H.block(3, 3, 3, 3) = -Mat3::Identity();
    H.block(3, 9, 3, 3) = Mat3::Identity();
    H.block(6, 12, 3, 3) = Mat3::Identity();

    // noise (order is ori, pos, vel)
    R.block(0, 0, 3, 3) *= std::pow(1e-2, 2);
    R.block(3, 3, 3, 3) *= std::pow(1e-1, 2);
    R.block(6, 6, 3, 3) *= std::pow(1e-1, 2);

    MatX U_dense, U_tri;
    StateHelper::get_marginal_U_block(state, Hx_order, U_dense,
                                      U_tri); // TODO: DOUBLE CHECK THIS
    MatX HUT = MatX::Zero(H.rows(), U_dense.rows() + U_tri.rows());

    HUT.leftCols(U_dense.rows()).noalias() = H * U_dense.transpose();
    HUT.rightCols(U_tri.rows()).noalias() =
        H * U_tri.transpose().triangularView<Eigen::Lower>();

    VecX RHTr = VecX::Zero(state->get_state_size());
    int local_id = 0;
    for (const auto &var : Hx_order) {
      RHTr.block(var->id(), 0, var->size(), 1).noalias() +=
          H.block(0, local_id, H.rows(), var->size()).transpose() * res /
          zupt_noise_multiplier_ / zupt_noise_multiplier_;
      local_id += var->size();
    }
    state->store_update_factor(HUT / zupt_noise_multiplier_, RHTr);

    // Finally move the state time forward
    StateHelper::update_llt(state);
    state->clear();
    state->add_marginal_state(state->clones_IMU.at(time1_cam));
    StateHelper::marginalize(state);
    state->remove_timestamp(time1_cam);
    state->clones_IMU.erase(time1_cam);
  }

  // Finally return
  last_zupt_state_timestamp_ = timestamp;
  last_zupt_count_++;
  return true;
}