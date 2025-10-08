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





#include "State.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_srvins;

State::State(StateOptions &options_,
             ov_srvins::InertialInitializerOptions &init_options_)
    : cam_pose_buffer(options_.max_clone_size + 2, options_.num_cameras),
      cam_pose_fej_buffer(options_.max_clone_size + 2, options_.num_cameras),
      kCloneStartId(15 + // IMU
                    (options_.do_calib_camera_timeoffset ? 1 : 0) +
                    options_.num_cameras *
                        (options_.do_calib_camera_pose ? 6 : 0) +
                    options_.num_cameras *
                        (options_.do_calib_camera_intrinsics ? 8 : 0)) {

  // Save our options
  options = options_;
  init_options = init_options_;

  // Setup buffer size
  const int kMarginalOffset = 50;
  const int kFeatSize = 3;
  const int kMaxStateSize =
      options.max_slam_features * kFeatSize + options.max_clone_size * 6 +
      15 + // IMU
      (options.do_calib_camera_timeoffset ? 1 : 0) +
      options.num_cameras *
          (options.do_calib_camera_pose ? options.num_cameras * 6 : 0) +
      options.num_cameras *
          (options.do_calib_camera_intrinsics ? options.num_cameras * 8 : 0) +
      kMarginalOffset;
  const int kMaxMeasurementSize =
      2 * options.num_cameras *
          (options.max_msckf_in_update + options.max_slam_features) *
          (options.max_clone_size + 1) +
      kMarginalOffset;
  const int kMaxSlamFeatStateSize = options.max_slam_features * kFeatSize;

  R_sqrt_inv_H_UT_ = EigenMatrixBuffer(kMaxMeasurementSize, kMaxStateSize);
  HT_R_inv_res_ = EigenMatrixBuffer(kMaxStateSize, 1);

  factor_init_dense_ = EigenMatrixBuffer(kMaxStateSize, kMaxSlamFeatStateSize);
  factor_init_tri_ = EigenMatrixBuffer(kMaxSlamFeatStateSize, kFeatSize);

  H_update_ = EigenMatrixBuffer(kMaxMeasurementSize, kMaxStateSize);
  res_update_ = EigenMatrixBuffer(kMaxMeasurementSize, 1);

  int current_id = 0;
  // Append the imu to the state and covariance
  imu = std::make_shared<IMU>();
  imu->set_local_id(current_id);
  variables_.push_back(imu);
  current_id += imu->size();

  // Camera to IMU time offset
  calib_dt_CAMtoIMU = std::make_shared<Vec>(1);
  if (options.do_calib_camera_timeoffset) {
    calib_dt_CAMtoIMU->set_local_id(current_id);
    variables_.push_back(calib_dt_CAMtoIMU);
    current_id += calib_dt_CAMtoIMU->size();
  }

  // Loop through each camera and create extrinsic and intrinsics
  for (int i = 0; i < options.num_cameras; i++) {

    // Allocate extrinsic transform
    auto pose = std::make_shared<PoseJPL>();

    // Allocate intrinsics for this camera
    auto intrin = std::make_shared<Vec>(8);

    // Add these to the corresponding maps
    calib_IMUtoCAM.insert({i, pose});
    cam_intrinsics.insert({i, intrin});

    // If calibrating camera-imu pose, add to variables
    if (options.do_calib_camera_pose) {
      pose->set_local_id(current_id);
      variables_.push_back(pose);
      current_id += pose->size();
    }

    // If calibrating camera intrinsics, add to variables
    if (options.do_calib_camera_intrinsics) {
      intrin->set_local_id(current_id);
      variables_.push_back(intrin);
      current_id += intrin->size();
    }
  }

  // Finally initialize our covariance to small value
  U_ = 1e-3 * MatX::Identity(current_id, current_id);

  // Finally, set some of our priors for our calibration parameters
  if (options.do_calib_camera_timeoffset) {
    U_(calib_dt_CAMtoIMU->id(), calib_dt_CAMtoIMU->id()) = 0.01;
  }
  if (options.do_calib_camera_pose) {
    for (int i = 0; i < options.num_cameras; i++) {
      U_.block(calib_IMUtoCAM.at(i)->id(), calib_IMUtoCAM.at(i)->id(), 3, 3) =
          0.005 * MatX::Identity(3, 3);
      U_.block(calib_IMUtoCAM.at(i)->id() + 3, calib_IMUtoCAM.at(i)->id() + 3,
               3, 3) = 0.015 * MatX::Identity(3, 3);
    }
  }
  if (options.do_calib_camera_intrinsics) {
    for (int i = 0; i < options.num_cameras; i++) {
      U_.block(cam_intrinsics.at(i)->id(), cam_intrinsics.at(i)->id(), 4, 4) =
          1.0 * MatX::Identity(4, 4);
      U_.block(cam_intrinsics.at(i)->id() + 4, cam_intrinsics.at(i)->id() + 4,
               4, 4) = 0.005 * MatX::Identity(4, 4);
    }
  }
}

double State::margtimestep() { return clones_IMU.begin()->first; }

void State::clear(bool clean_msckf_state) {
  if (clean_msckf_state) {
    features_MSCKF.clear();
  }

  // Clean the buffer
  HT_R_inv_res_.reset();
  R_sqrt_inv_H_UT_.reset();
  if (!is_initialized) {
    H_update_.reset();
    res_update_.reset();
  }

  // Clear the buffer
  x_init_.clear();
  factor_init_tri_.reset();
  factor_init_dense_.reset();
}

double State::get_x_squared_norm() {
  double x_norm_sq = 0.0;
  for (auto var : variables_) {
    x_norm_sq += var->value().squaredNorm();
  }

  for (auto var : features_MSCKF) {
    x_norm_sq += var.second->value().squaredNorm();
  }
  return x_norm_sq;
}

double State::get_dx_squared_norm() {
  double dx_norm_sq = xk_minus_xk1_.squaredNorm();
  for (const auto &feat_msckf : features_MSCKF) {
    dx_norm_sq += feat_msckf.second->feat_dx.squaredNorm();
  }
  return dx_norm_sq;
}

double State::get_residual_squared_norm() {
  double res_norm = 0.0;
  for (const auto &feat_msckf : features_MSCKF) {
    res_norm += feat_msckf.second->res_msckf.squaredNorm();
  }

  res_norm += res_update_.get().squaredNorm();
  return res_norm;
}

std::shared_ptr<ov_type::PoseJPL>
State::get_clone_pose(double query_timestamp) {
  if (clones_IMU.find(query_timestamp) != clones_IMU.end()) {
    return clones_IMU.at(query_timestamp);
  } else if (query_timestamp == timestamp) {
    return imu->pose();
  } else {
    return nullptr;
  }
}

void State::resize_U_to_square() {
  int state_size = (int)U_.cols();
  U_.conservativeResizeLike(MatX::Zero(state_size, state_size));
  xk_minus_x0_.conservativeResizeLike(VecX::Zero(state_size, 1));
  xk_minus_xk1_.conservativeResizeLike(VecX::Zero(state_size, 1));
}

void State::calculate_clone_poses(bool fej) {
  static Mat3 R_ItoC, R_GtoI;
  static Vec3 p_IinC, p_IinG;
  for (const auto &calib : calib_IMUtoCAM) {
    // Follow OpenVINS convention here, we dont FEJ calibration
    R_ItoC = calib.second->Rot();
    p_IinC = calib.second->pos();
    for (const auto &clone : clones_IMU) {
      R_GtoI = fej ? clone.second->Rot_fej() : clone.second->Rot();
      p_IinG = fej ? clone.second->pos_fej() : clone.second->pos();
      auto &cam_pose =
          fej ? cam_pose_fej_buffer.get_buffer_unsafe(calib.first, clone.first)
              : cam_pose_buffer.get_buffer_unsafe(calib.first, clone.first);
      cam_pose.R_GtoC = R_ItoC * R_GtoI;
      cam_pose.p_CinG = cam_pose.R_GtoC.transpose() * -p_IinC + p_IinG;
    }
  }
}

void State::calculate_clone_poses_fej() { calculate_clone_poses(true); }

void State::erase_feat(size_t feat_id) {
  // Skip if this feature is not in the state
  if (features_MSCKF.find(feat_id) == features_MSCKF.end()) {
    return;
  }
  features_MSCKF.erase(feat_id);
}

void State::store_init_factor(const std::shared_ptr<Type> &new_var,
                              const MatX &tri_factor,
                              const MatX &dense_factor) {
  x_init_.emplace_back(new_var);
  factor_init_tri_.append_rows(tri_factor);
  factor_init_dense_.append_top_cols_and_resize(dense_factor);
}

void State::store_update_factor(const MatX &R_sinv_H_UT,
                                const VecX &R_inv_res) {
  R_sqrt_inv_H_UT_.append_left_rows(R_sinv_H_UT);
  HT_R_inv_res_.get().noalias() += R_inv_res;
}

void State::store_update_jacobians(
    const MatX &Hx, const VecX &res,
    const std::vector<std::shared_ptr<ov_type::Type>> &x_order) {
  std::vector<std::pair<EigenMatrixBuffer::Id, EigenMatrixBuffer::Size>> order;
  for (const auto &var : x_order) {
    order.emplace_back(var->id(), var->size());
  }
  H_update_.append_block_rows_with_order(Hx, order);
  res_update_.append_rows(res);
}

int State::get_state_size() { return (int)U_.cols(); }

void State::setup_matrix_buffer() {
  const int kStateSize = get_state_size();
  const int kFeatSize = 3;
  R_sqrt_inv_H_UT_.set_size(0, kStateSize);
  HT_R_inv_res_.set_size(kStateSize, 1);
  factor_init_tri_.set_size(0, kFeatSize);

  H_update_.set_size(0, kStateSize);
  res_update_.set_size(0, 1);
}
void State::update_timestamp(double new_time) {
  timestamp = new_time;
  cam_pose_buffer.add_timestamp(new_time);
  cam_pose_fej_buffer.add_timestamp(new_time);
}

void State::remove_timestamp(double old_time) {
  cam_pose_buffer.remove_timestamp(old_time);
  cam_pose_fej_buffer.remove_timestamp(old_time);
}

Eigen::Ref<const VecX> State::get_xk_minus_xk1() { return xk_minus_xk1_; }

void State::add_marginal_state(std::shared_ptr<ov_type::Type> var) {
  state_to_marg_.push_back(var);
}