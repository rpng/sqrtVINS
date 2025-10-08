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





#include "DynamicInitializer.h"

#include "cpi/CpiV1.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "feat/FeatureHelper.h"

#include "OpengvHelper.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/Landmark.h"
#include "utils/DataType.h"
#include "utils/Helper.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

#include "Solver.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_srvins;

DynamicInitializer::DynamicInitializer(
    const InertialInitializerOptions &params,
    std::shared_ptr<ov_core::FeatureDatabase> db,
    std::shared_ptr<ov_srvins::Propagator> propagator,
    std::shared_ptr<ov_srvins::UpdaterMSCKF> updater_msckf,
    std::shared_ptr<ov_srvins::UpdaterSLAM> updater_slam)
    : params_(params), db_(db), propagator_(propagator),
      updater_msckf_(updater_msckf), updater_slam_(updater_slam) {
  gravity_ << 0.0, 0.0, params_.gravity_mag;
}

bool DynamicInitializer::initialize(std::shared_ptr<State> &state) {

  // Get the newest and oldest timestamps we will try to initialize between!
  auto rT1 = boost::posix_time::microsec_clock::local_time();

  // Make sure we have IMU reading before the oldest time
  std::vector<ov_core::ImuData> imu_data;
  propagator_->get_imu_data(imu_data);

  // Now we will make a copy of our features here
  // We do this to ensure that the feature database can continue to have new
  // measurements appended to it in an async-manor so this initialization
  // can be performed in a secondary thread while feature tracking is still
  // performed.
  double latest_cam_time = -1;
  std::unordered_map<size_t, std::shared_ptr<Feature>> features =
      db_->get_internal_data();
  std::set<double> timestamps;
  for (const auto &feat : db_->get_internal_data()) {
    for (auto cam_t : feat.second->timestamps) {
      for (auto t : cam_t.second) {
        latest_cam_time = std::max(latest_cam_time, t);
        timestamps.insert(t);
      }
    }
  }

  // ======================================================
  // Check if the window size is large enough
  double window_length = *timestamps.rbegin() - *timestamps.begin();
  if (window_length + params_.init_window_offset < params_.init_window_time) {
    PRINT_ERROR(RED "[init-d]: window too small (%.2f < %.2f)!!\n" RESET,
                window_length, params_.init_window_time);
    return false;
  }

  if ((int)timestamps.size() < params_.init_dyn_num_pose) {
    PRINT_ERROR(RED "[init-d]: not enough camera poses (%zu < %d)!!\n" RESET,
                timestamps.size(), params_.init_dyn_num_pose);
    return false;
  }

  // ======================================================
  // ======================================================
  gravity_ << 0.0, 0.0, params_.gravity_mag;

  std::set<double> map_camera_times;
  keyframing(features, map_camera_times);

  double oldest_camera_time = *map_camera_times.begin();

  // Return if we do not have our full window or not enough measurements
  // Also check that we have enough features to initialize with
  if ((int)map_camera_times.size() < params_.init_dyn_num_pose) {
    std::cout << "acctually you have " << timestamps.size() << std::endl;
    PRINT_ERROR(RED "[init-d]: not enough camera poses (%zu < %d)!!\n" RESET,
                map_camera_times.size(), params_.init_dyn_num_pose);
    return false;
  }

  // Use device factory calibration parameters for biases
  Vec3 gyroscope_bias = params_.init_dyn_bias_g;
  Vec3 accelerometer_bias = params_.init_dyn_bias_a;

  // Check that we have some angular velocity / orientation change
  double accel_inI_norm = 0.0;
  double theta_inI_norm = 0.0;
  double time0_in_imu = oldest_camera_time + params_.calib_camimu_dt;
  double time1_in_imu = latest_cam_time + params_.calib_camimu_dt;
  std::vector<ov_core::ImuData> readings =
      select_imu_readings(imu_data, time0_in_imu, time1_in_imu);
  assert(readings.size() > 2);
  for (int k = 0; k < (int)readings.size() - 1; k++) {
    auto imu0 = readings.at(k);
    auto imu1 = readings.at(k + 1);
    double dt = imu1.timestamp - imu0.timestamp;
    Vec3 wm = 0.5 * (imu0.wm + imu1.wm) - gyroscope_bias;
    Vec3 am = 0.5 * (imu0.am + imu1.am) - accelerometer_bias;
    theta_inI_norm += (-wm * dt).norm();
    accel_inI_norm += am.norm();
  }
  accel_inI_norm /= (double)(readings.size() - 1);
  if (180.0 / M_PI * theta_inI_norm < params_.init_dyn_min_deg) {
    PRINT_WARNING(
        YELLOW
        "[init-d]: gyroscope only %.2f degree change (%.2f thresh)\n" RESET,
        180.0 / M_PI * theta_inI_norm, params_.init_dyn_min_deg);
    return false;
  }
  PRINT_DEBUG("[init-d]: |theta_I| = %.4f deg and |accel| = %.4f\n",
              180.0 / M_PI * theta_inI_norm, accel_inI_norm);

  auto rT2 = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // Preintegration
  // ======================================================
  std::map<double, std::shared_ptr<ov_core::CpiV1>> map_camera_cpi_I0toIi;
  if (!preintegrate(imu_data, map_camera_times, gyroscope_bias,
                    accelerometer_bias, map_camera_cpi_I0toIi)) {
    return false;
  }

  auto rT3 = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // Gyro bias estimator
  // ======================================================
  // Get timestamp to uv_norm mapping, avoid redundant calculation
  if (params_.use_bg_estimator) {
    std::unordered_map<
        size_t, std::unordered_map<size_t, std::unordered_map<double, Vec2>>>
        feat_norms;
    for (auto feat : features) {
      for (size_t cam_id = 0; cam_id < feat.second->uvs_norm.size(); cam_id++) {
        if (feat.second->uvs_norm.find(cam_id) == feat.second->uvs_norm.end()) {
          continue;
        }
        for (size_t i = 0; i < feat.second->uvs_norm.at(cam_id).size(); i++) {
          double time = feat.second->timestamps.at(cam_id).at(i);
          Vec2 uv = feat.second->uvs_norm.at(cam_id).at(i);
          feat_norms[feat.first][cam_id][time] = uv;
        }
      }
    }

    // Bg estimation with rotation only constraint
    if (!solve_bg(map_camera_cpi_I0toIi, feat_norms, gyroscope_bias))
      return false;

    // ======================================================
    // Redo preintegration with new bias
    // ======================================================
    if (!preintegrate(imu_data, map_camera_times, gyroscope_bias,
                      accelerometer_bias, map_camera_cpi_I0toIi)) {
      return false;
    }
  }

  auto rT4 = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // Find translation directions
  // ======================================================
  std::map<size_t, std::map<double, Mat3>> map_R_CktoI0;
  // Precompute R_CktoI0
  for (int i = 0; i < params_.num_cameras; i++) {
    Vec4 q_ItoC = params_.camera_extrinsics.at(i).block(0, 0, 4, 1);
    Mat3 R_ItoC = quat_2_Rot(q_ItoC);
    for (auto time : map_camera_times) {
      if (map_camera_cpi_I0toIi.find(time) != map_camera_cpi_I0toIi.end() &&
          map_camera_cpi_I0toIi.at(time) != nullptr) {
        map_R_CktoI0[i][time] =
            map_camera_cpi_I0toIi.at(time)->R_k2tau.transpose() *
            R_ItoC.transpose();
      }
    }
  }

  std::map<size_t, Mat3> id_to_eigenvectors;
  std::map<size_t, Mat3> id_to_H_planes;
  std::map<size_t, std::pair<double, double>> id_to_timepair;
  std::map<size_t, std::pair<size_t, size_t>> id_to_campair;

  // Get all the bearing in I0
  // Map by
  // timestamp
  // - camid
  //  - feat_id
  //   - bearing_in_I0
  std::map<double,
           std::unordered_map<size_t, std::map<size_t, std::shared_ptr<Vec3>>>>
      bearings_in_I0;

  Vec3 uv_norm_tmp;
  for (const auto &feat : features) {
    for (const auto &cam_time : feat.second->timestamps) {
      size_t cam_id = cam_time.first;
      for (size_t t = 0; t < cam_time.second.size(); t++) {
        double time = cam_time.second.at(t);
        // Skip non-keyframe
        if (map_camera_times.find(time) == map_camera_times.end())
          continue;
        Vec2 uv = feat.second->uvs_norm.at(cam_id).at(t);
        uv_norm_tmp << uv(0), uv(1), 1;
        uv_norm_tmp = map_R_CktoI0.at(cam_id).at(time) * uv_norm_tmp;
        bearings_in_I0[time][cam_id][feat.first] =
            std::make_shared<Vec3>(uv_norm_tmp);
      }
    }
  }

  // Sanity check
  for (const auto &time_camid : bearings_in_I0) {
    double time = time_camid.first;
    for (const auto &camid_featid : bearings_in_I0.at(time)) {
      size_t camid = camid_featid.first;
      std::cout << std::setprecision(16) << "time: " << time
                << " camid: " << camid
                << " size: " << camid_featid.second.size() << std::endl;
    }
  }
  // Find translation directions, which is the eigenvector that corresponds the
  // smallest eigenvalue
  size_t id_curr = 0;
  std::set<size_t> outlier_ids_total;
  for (const auto &time0 : map_camera_times) {
    for (const auto &time1 : map_camera_times) {
      for (int cam_id0 = 0; cam_id0 < params_.num_cameras; cam_id0++) {
        for (int cam_id1 = 0; cam_id1 < params_.num_cameras; cam_id1++) {
          // Skip the same timestamp
          if (time0 >= time1)
            continue;

          std::set<size_t> outliers;
          Mat3 eigen_vectors;
          if (!find_eigen_vectors(map_R_CktoI0, bearings_in_I0, time0, time1,
                                  cam_id0, cam_id1, outliers, eigen_vectors))
            continue;

          // Get inlier ids (use union logic for now)
          for (auto id : outliers) {
            outlier_ids_total.insert(id);
          }

          id_to_eigenvectors.insert({id_curr, eigen_vectors});
          id_to_timepair.insert({id_curr, {time0, time1}});
          id_to_campair.insert({id_curr, {cam_id0, cam_id1}});
          id_curr++;
        }
      }
    }
  }

  auto rT5 = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // Create linear system and solve it!
  // ======================================================
  Eigen::Matrix<DataType, kVelocityAndGravitySize, kVelocityAndGravitySize> A;
  Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> b;
  Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> x_init;

  build_linear_system(map_camera_cpi_I0toIi, id_to_eigenvectors, id_to_timepair,
                      id_to_campair, A, b, x_init);
  auto rT6 = boost::posix_time::microsec_clock::local_time();

  // ======================================================
  // Gravity refinement
  // ======================================================
  Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> x_refined;
  if (!refine_gravity(A, b, x_init, x_refined)) {
    PRINT_WARNING(RED "[init-d]: gravity did not converge\n" RESET);
    return false;
  }
  auto rT7 = boost::posix_time::microsec_clock::local_time();

  Vec3 v_I0inI0 = x_refined.topRows<3>();
  Vec3 gravity_inI0 = x_refined.bottomRows<3>();

  PRINT_INFO("[init-d]: gravity in I0 was %.3f,%.3f,%.3f and |g| = %.4f\n",
             gravity_inI0(0), gravity_inI0(1), gravity_inI0(2),
             gravity_inI0.norm());

  // ======================================================
  // Iterative refinement with iterative SRF
  // ======================================================
  // Convert our states to be a gravity aligned global frame of reference
  // Here we say that the I0 frame is at 0,0,0 and shared the global origin
  Mat3 R_GtoI0;
  gram_schmidt(gravity_inI0, R_GtoI0);

  // Try to solve by filter
  Eigen::Matrix<DataType, 16, 1> imu_init =
      Eigen::Matrix<DataType, 16, 1>::Zero();
  imu_init.topRows<4>() = ov_core::rot_2_quat(R_GtoI0);
  imu_init.middleRows<3>(7) = R_GtoI0.transpose() * v_I0inI0;
  imu_init.middleRows<3>(10) = gyroscope_bias;
  imu_init.bottomRows<3>() = accelerometer_bias;
  if (imu_init.hasNaN()) {
    PRINT_DEBUG("" RED "[init-d]: NaN in imu_init\n" RESET);
    return false;
  }

  // Change feature map to vectors for solver
  std::vector<std::shared_ptr<Feature>> features_vec;
  for (auto const &feat : features) {
    if (outlier_ids_total.find(feat.first) == outlier_ids_total.end())
      features_vec.push_back(feat.second);
  }

  // Initialize the first state
  Solver srf_solver(state, imu_init, features_vec, map_camera_times,
                    propagator_, updater_msckf_, updater_slam_);

  // Solve the problem
  if (!srf_solver.solve()) {
    state = std::make_shared<State>(state->options,
                                    state->init_options); // Reset the state
    StateHelper::initialize_state(state, VecX::Zero(15), -1);
    return false;
  }

  // Debug timing information about how long it took to initialize!!
  auto rT8 = boost::posix_time::microsec_clock::local_time();
  PRINT_DEBUG("[TIME]: %.2f ms for prelim tests\n",
              (rT2 - rT1).total_microseconds() * 1e-3);
  PRINT_DEBUG("[TIME]: %.2f ms for preintegration\n",
              (rT3 - rT2).total_microseconds() * 1e-3);
  PRINT_DEBUG("[TIME]: %.2f ms for bg estimation\n",
              (rT4 - rT3).total_microseconds() * 1e-3);
  PRINT_DEBUG("[TIME]: %.2f ms for rotation-only\n",
              (rT5 - rT4).total_microseconds() * 1e-3);
  PRINT_DEBUG("[TIME]: %.2f ms for linsys\n",
              (rT6 - rT5).total_microseconds() * 1e-3);
  PRINT_DEBUG("[TIME]: %.2f ms for graivty refinement\n",
              (rT7 - rT6).total_microseconds() * 1e-3);
  PRINT_DEBUG("[TIME]: %.2f ms for srf + cov recovery\n",
              (rT8 - rT7).total_microseconds() * 1e-3);
  PRINT_DEBUG("[TIME]: %.2f ms total for initialization\n",
              (rT8 - rT1).total_microseconds() * 1e-3);

  // Logger
  // Write the output of initialization window to file for evaluation
  std::ofstream outfile;
  if (params_.record_init_pose) {

    // Log poses
    outfile.open(params_.init_poses_log_file_path.c_str());
    outfile << "# timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 "
               "Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33"
            << std::endl;

    for (auto const &clones : state->clones_IMU) {
      double timestamp = clones.first;

      // timestamp
      outfile.precision(5);
      outfile.setf(std::ios::fixed, std::ios::floatfield);
      outfile << timestamp << " ";

      Vec4 quat = clones.second->quat();
      Vec3 pos = clones.second->pos();

      // pose
      outfile.precision(6);

      // px py pz qx qy qz qw
      // p is IinG, q is ItoG Hamilton (same as GtoI JPL) <<<=== Good Job
      outfile << pos[0] << " " << pos[1] << " " << pos[2] << " " << quat[0]
              << " " << quat[1] << " " << quat[2] << " " << quat[3];
      outfile << std::endl;
    }
    outfile.close();
  }

  // Properly setup state before return
  // Erase the other clones
  auto it = state->clones_IMU.begin();
  while (it != state->clones_IMU.end()) {
    if (it->first != state->timestamp) {
      state->add_marginal_state(it->second);

      state->remove_timestamp(it->first);
      it = state->clones_IMU.erase(it);

    } else {
      it++;
    }
  }
  StateHelper::marginalize(state);

  // Setup fej value
  if (state->options.do_fej) {
    state->imu->set_fej(state->imu->value());
    state->clones_IMU[state->timestamp]->set_fej(
        state->clones_IMU[state->timestamp]->value());
  }

  if (params_.record_init_timing) {
    // Log timing NOTE: it's in a different format than normal timing since we
    // don't have time for multiple timestamps
    outfile.open(params_.init_metadata_log_file_path.c_str());
    // Int variables
    outfile.precision(5);
    outfile.setf(std::ios::fixed, std::ios::floatfield);
    outfile << "start_time: " << state->timestamp << std::endl;
    outfile.precision(6);
    outfile << "vel: " << state->imu->vel().transpose() << std::endl;
    outfile << "time_pre_test: " << (rT2 - rT1).total_microseconds() * 1e-3
            << std::endl;
    outfile << "time_preint: " << (rT3 - rT2).total_microseconds() * 1e-3
            << std::endl;
    outfile << "time_bg: " << (rT4 - rT3).total_microseconds() * 1e-3
            << std::endl;
    outfile << "time_rot_only: " << (rT5 - rT4).total_microseconds() * 1e-3
            << std::endl;
    outfile << "time_linsys: " << (rT6 - rT5).total_microseconds() * 1e-3
            << std::endl;
    outfile << "time_grav_ref: " << (rT7 - rT6).total_microseconds() * 1e-3
            << std::endl;
    outfile << "time_srf_cov: " << (rT8 - rT7).total_microseconds() * 1e-3
            << std::endl;
    outfile << "time_total: " << (rT8 - rT1).total_microseconds() * 1e-3
            << std::endl;
    outfile.close();
  }
  return true;
}

void DynamicInitializer::keyframing(
    const std::unordered_map<size_t, std::shared_ptr<ov_core::Feature>>
        &feature_database,
    std::set<double> &keyframe_timestamps) {

  keyframe_timestamps.clear();

  // 1. Get all the valid timestamps
  std::set<double> all_timestamps;
  for (const auto &feat : feature_database) {
    for (const auto &camtimes : feat.second->timestamps) {
      for (const auto &time : camtimes.second) {
        all_timestamps.insert(time);
      }
    }
  }

  if (all_timestamps.empty() ||
      (int)all_timestamps.size() < params_.init_dyn_num_pose) {
    PRINT_ERROR("No valid timestamps for keyframeing, You are screwed up bro!");
    std::abort();
  }

  // 2. Insert the first and last timestamp
  keyframe_timestamps.insert(*all_timestamps.begin());
  keyframe_timestamps.insert(*all_timestamps.rbegin());

  // 3. Get the other timestamps with greedy search of next new keyframe that
  // maximizes the minimal disparity
  int remaining_keyframes = params_.init_dyn_num_pose - 2;
  DataType max_disp, min_disp;
  double max_time0 = 0;
  while (remaining_keyframes > 0) {
    for (const auto &time0_disps : db_->map_disp) {
      if (keyframe_timestamps.find(time0_disps.first) !=
          keyframe_timestamps.end())
        continue;
      max_disp = -1;
      max_time0 = -1;
      min_disp = std::numeric_limits<DataType>::max();
      for (const auto &time_disp_pair : time0_disps.second) {
        // Skip the ones that are not valid
        if (time_disp_pair.second == -1)
          continue;
        min_disp = std::min(min_disp, time_disp_pair.second);
      }
      if (min_disp > max_disp) {
        max_disp = min_disp;
        max_time0 = time0_disps.first;
      }
    }
    // Insert the timestamp that maximizes the disparity
    keyframe_timestamps.insert(max_time0);
    remaining_keyframes--;
  }
}

bool DynamicInitializer::preintegrate(
    const std::vector<ov_core::ImuData> &imu_data,
    const std::set<double> &keyframe_timestamps, const Vec3 &gyroscope_bias,
    const Vec3 &accelerometer_bias,
    std::map<double, std::shared_ptr<ov_core::CpiV1>> &map_camera_cpi_I0toIi) {
  double oldest_camera_time = *keyframe_timestamps.begin();
  map_camera_cpi_I0toIi.clear();
  for (auto const &timepair : keyframe_timestamps) {

    // No preintegration at the first timestamp
    double current_time = timepair;
    if (current_time == oldest_camera_time) {
      std::shared_ptr<ov_core::CpiV1> cpi_dummy =
          std::make_shared<ov_core::CpiV1>(params_.sigma_w, params_.sigma_wb,
                                           params_.sigma_a, params_.sigma_ab,
                                           true);
      cpi_dummy->DT = 0.0;
      cpi_dummy->R_k2tau = Mat3::Identity();
      cpi_dummy->alpha_tau = Vec3::Zero();
      cpi_dummy->H_a = Mat3::Identity();
      map_camera_cpi_I0toIi.insert({current_time, cpi_dummy});
      continue;
    }

    // Perform our preintegration from I0 to Ii (used in the linear system)
    double cpiI0toIi1_time0_in_imu =
        oldest_camera_time + params_.calib_camimu_dt;
    double cpiI0toIi1_time1_in_imu = current_time + params_.calib_camimu_dt;
    auto cpiI0toIi1 = std::make_shared<ov_core::CpiV1>(
        params_.sigma_w, params_.sigma_wb, params_.sigma_a, params_.sigma_ab);
    cpiI0toIi1->setLinearizationPoints(gyroscope_bias, accelerometer_bias);
    std::vector<ov_core::ImuData> cpiI0toIi1_readings = select_imu_readings(
        imu_data, cpiI0toIi1_time0_in_imu, cpiI0toIi1_time1_in_imu);
    if (cpiI0toIi1_readings.size() < 2) {
      PRINT_DEBUG(YELLOW
                  "[init-d]: camera %.2f in has %zu IMU readings!\n" RESET,
                  (cpiI0toIi1_time1_in_imu - cpiI0toIi1_time0_in_imu),
                  cpiI0toIi1_readings.size());
      return false;
    }
    double cpiI0toIi1_dt_imu =
        cpiI0toIi1_readings.at(cpiI0toIi1_readings.size() - 1).timestamp -
        cpiI0toIi1_readings.at(0).timestamp;
    if (std::abs(cpiI0toIi1_dt_imu -
                 (cpiI0toIi1_time1_in_imu - cpiI0toIi1_time0_in_imu)) > 0.01) {
      PRINT_DEBUG(
          YELLOW
          "[init-d]: camera IMU was only propagated %.3f of %.3f\n" RESET,
          cpiI0toIi1_dt_imu,
          (cpiI0toIi1_time1_in_imu - cpiI0toIi1_time0_in_imu));
      return false;
    }
    for (size_t k = 0; k < cpiI0toIi1_readings.size() - 1; k++) {
      auto imu0 = cpiI0toIi1_readings.at(k);
      auto imu1 = cpiI0toIi1_readings.at(k + 1);
      // If we want to estimate gyro bias , we would require Jq
      if (params_.use_bg_estimator) {
        cpiI0toIi1->feed_IMU_Jq(imu0.timestamp, imu1.timestamp, imu0.wm,
                                imu0.am, imu1.wm, imu1.am);
      } else {
        cpiI0toIi1->feed_IMU_meanonly(imu0.timestamp, imu1.timestamp, imu0.wm,
                                      imu0.am, imu1.wm, imu1.am);
      }
    }

    // Finally push back our integrations!
    map_camera_cpi_I0toIi.insert({current_time, cpiI0toIi1});
  }
  return true;
}

bool DynamicInitializer::find_eigen_vectors(
    const std::map<size_t, std::map<double, Mat3>> &map_R_CktoI0,
    const std::map<double, std::unordered_map<
                               size_t, std::map<size_t, std::shared_ptr<Vec3>>>>
        &bearings_in_I0,
    double time0, double time1, size_t cam_id0, size_t cam_id1,
    std::set<size_t> &outliers, Mat3 &eigen_vectors) {
  const int kMinMeas = 5;
  int num_meas = 0;
  const auto &bearings_C0_in_I0 = bearings_in_I0.at(time0).at(cam_id0);
  const auto &bearings_C1_in_I0 = bearings_in_I0.at(time1).at(cam_id1);
  MatX n_plane = MatX::Zero(3, bearings_in_I0.at(time0).at(cam_id0).size());
  Mat3 H_plane = Mat3::Zero();

  // Get inlier ids too
  std::vector<size_t> feat_ids;
  Vec3 f0, f1;

  for (auto const &feat0 : bearings_C0_in_I0) {
    auto feat1_itr = bearings_C1_in_I0.find(feat0.first);
    if (feat1_itr == bearings_C1_in_I0.end()) {
      continue;
    }

    f0 = *(feat0.second);
    f1 = *(feat1_itr)->second;

    Vec3 pn = (f0.cross(f1));
    H_plane +=
        pn * pn.transpose() / pn.squaredNorm(); // NOTE: Normalize this to avoid
                                                // outlier dominate H_plane

    feat_ids.push_back(feat0.first);
    n_plane.col(num_meas) = pn;
    num_meas++;
  }

  if (num_meas < kMinMeas)
    return false;

  n_plane.conservativeResize(3, num_meas);
  Eigen::SelfAdjointEigenSolver<Mat3> eigensolver(H_plane);
  Vec3 t_direction = eigensolver.eigenvectors().col(0);

  // Statistical filtering
  VecX residuals = (n_plane.transpose() * t_direction).cwiseAbs();
  DataType mean = residuals.mean();

  std::vector<DataType> res_vec(residuals.data(),
                                residuals.data() + residuals.size());
  std::sort(res_vec.begin(), res_vec.end());

  size_t n = res_vec.size();
  DataType median;
  if (n % 2 == 0) {
    // If even, return the average of the two middle elements
    median = (res_vec[n / 2 - 1] + res_vec[n / 2]) / 2.0;
  } else {
    // If odd, return the middle element
    median = res_vec[n / 2];
  }

  // Statistical filtering to remove outliers
  DataType std =
      sqrt((residuals - mean * VecX::Ones(residuals.rows())).squaredNorm() /
           (residuals.rows() - 1));
  H_plane = Mat3::Zero();
  for (int i = 0; i < num_meas; i++) {
    if (std::abs(residuals(i) - median) < 3 * std) {
      H_plane += n_plane.col(i) * n_plane.col(i).transpose();
    } else {
      outliers.insert(feat_ids.at(i));
    }
  }

  // Return false if we does not have enough measurements for this pair
  if (num_meas - outliers.size() < kMinMeas)
    return false;

  eigensolver.compute(H_plane);

  if (params_.eigenvector_debug) {
    std::cout << std::setprecision(16) << "timestamps: " << time0 << " "
              << time1 << std::endl;
    std::cout << "cameras: " << cam_id0 << " " << cam_id1 << std::endl;
    std::cout << "eigenvalues: " << std::endl
              << std::setprecision(3) << eigensolver.eigenvalues().transpose()
              << std::endl;
    std::cout << "eigenvectors: " << std::endl
              << std::setprecision(3) << eigensolver.eigenvectors()
              << std::endl;
  }
  eigen_vectors = eigensolver.eigenvectors();
  return true;
}

void DynamicInitializer::build_linear_system(
    const std::map<double, std::shared_ptr<ov_core::CpiV1>>
        &map_camera_cpi_I0toIi,
    const std::map<size_t, Mat3> &id_to_eigenvectors,
    const std::map<size_t, std::pair<double, double>> &id_to_timepair,
    const std::map<size_t, std::pair<size_t, size_t>> &id_to_campair,
    Eigen::Matrix<DataType, kVelocityAndGravitySize, kVelocityAndGravitySize>
        &A_out,
    Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> &b_out,
    Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> &x_init) {

  int row_size = 2 * id_to_eigenvectors.size();
  int col_size = kVelocityAndGravitySize;
  MatX A = MatX::Zero(row_size, col_size);
  VecX b = VecX::Zero(row_size);

  std::map<size_t, Vec3> map_to_p_CinI;
  for (auto ex : params_.camera_extrinsics) {
    Vec4 q_ItoC = params_.camera_extrinsics.at(ex.first).topRows<4>();
    Vec3 p_IinC = params_.camera_extrinsics.at(ex.first).bottomRows<3>();
    Mat3 R_ItoC = quat_2_Rot(q_ItoC);
    Vec3 p_CinI = -R_ItoC.transpose() * p_IinC;
    map_to_p_CinI.insert({ex.first, p_CinI});
  }

  int row_count = 0;
  constexpr int kVelocityId = 0;
  constexpr int kGravityId = 3;
  constexpr int kMeasurementSize = 2;
  Mat3 R_I0toIk0, R_I0toIk1;
  for (auto data : id_to_eigenvectors) {
    size_t id = data.first;
    double time0 = id_to_timepair.at(id).first;
    double time1 = id_to_timepair.at(id).second;
    size_t cam_id0 = id_to_campair.at(id).first;
    size_t cam_id1 = id_to_campair.at(id).second;

    auto cpi0 = map_camera_cpi_I0toIi.at(time0);
    auto cpi1 = map_camera_cpi_I0toIi.at(time1);
    double t0 = cpi0->DT;
    double t1 = cpi1->DT;

    R_I0toIk0 = map_camera_cpi_I0toIi.at(time0)->R_k2tau;
    R_I0toIk1 = map_camera_cpi_I0toIi.at(time1)->R_k2tau;
    // const auto &eig_vec = data.second.rightCols(2);
    auto eig_vec = data.second.rightCols(2); // The one with smallest eigenvalue
    A.block<kMeasurementSize, kVelocitySize>(row_count, kVelocityId) =
        -(t1 - t0) * eig_vec.transpose();
    A.block<kMeasurementSize, kGravitySize>(row_count, kGravityId) =
        0.5 * (t1 * t1 - t0 * t0) * eig_vec.transpose();
    b.block<kMeasurementSize, 1>(row_count, 0) =
        eig_vec.transpose() *
        (cpi1->alpha_tau - cpi0->alpha_tau +
         (R_I0toIk1.transpose() * map_to_p_CinI.at(cam_id1) -
          R_I0toIk0.transpose() * map_to_p_CinI.at(cam_id0)));
    row_count += kMeasurementSize;
  }

  // Compress the linear system by QR factorization
  efficient_QR(A, b);
  A_out = A.topRows<kVelocityAndGravitySize>();
  b_out = b.topRows<kVelocityAndGravitySize>();
  x_init = A_out.triangularView<Eigen::Upper>().solve(b_out);
}

bool DynamicInitializer::refine_gravity(
    const Eigen::Matrix<DataType, kVelocityAndGravitySize,
                        kVelocityAndGravitySize> &A,
    const Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> &b,
    const Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> &vg_init,
    Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> &vg) {
  // Get gravity
  Vec3 g_init = vg_init.bottomRows<3>();
  g_init = g_init / g_init.norm();
  DataType beta = acos(g_init(2));
  DataType alpha = atan2(g_init(1), g_init(0));

  Eigen::Matrix<DataType, kVelocityAndGravityErrorSize, 1> x;
  x.topRows<3>() = vg_init.topRows<3>();
  x.bottomRows<2>() << alpha, beta;
  Eigen::Matrix<DataType, kVelocityAndGravitySize, kVelocityAndGravityErrorSize>
      A_new(Eigen::Matrix<DataType, kVelocityAndGravitySize,
                          kVelocityAndGravityErrorSize>::Zero());
  Eigen::Matrix<DataType, kVelocityAndGravitySize, kVelocityErrorSize> A1 =
      A.leftCols<kVelocityErrorSize>();
  Eigen::Matrix<DataType, kVelocityAndGravitySize, kGravitySize> A2 =
      A.rightCols<kGravitySize>();

  A_new.leftCols<kVelocityErrorSize>() = A1;
  DataType lambda = params_.init_grav_opt_init_lambda;
  Eigen::Matrix<DataType, kGravitySize, kGravityErrorSize> J;
  Vec3 gravity, gravity_new;
  Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> r;
  Eigen::Matrix<DataType, kVelocityAndGravityErrorSize, 1> dx;
  Eigen::Matrix<DataType, kVelocityAndGravityErrorSize,
                kVelocityAndGravityErrorSize>
      H;
  DataType f_true, f_pred, f_curr, rho;
  for (int i = 0; i < params_.init_grav_opt_max_iter; i++) {
    J = get_gravity_Jacobian(alpha, beta, params_.gravity_mag);
    A_new.rightCols<kGravityErrorSize>() = A2 * J;
    gravity = get_gravity(alpha, beta, params_.gravity_mag);
    r = -(A1 * x.topRows<kVelocityErrorSize>() + A2 * gravity - b);
    H = A_new.transpose() * A_new;
    H.diagonal() = (lambda + 1) * H.diagonal();
    dx = H.llt().solve(A_new.transpose() * r);

    // Do LM in here
    gravity_new =
        get_gravity(alpha + dx(kVelocityErrorSize),
                    beta + dx(kVelocityErrorSize + 1), params_.gravity_mag);
    f_true = (A1 * (x.topRows<kVelocityErrorSize>() +
                    dx.topRows<kVelocityErrorSize>()) +
              A2 * gravity_new - b)
                 .squaredNorm();
    f_pred = (r - A_new * dx).squaredNorm();
    f_curr = r.squaredNorm();

    rho = (f_pred - f_curr) / (f_true - f_curr);
    bool do_update = true;

    if (rho < 0.25) {
      do_update = false;
      lambda /= params_.init_grav_opt_lambda_decay; // add more damping
    } else if (rho > 0.75) {
      lambda *= params_.init_grav_opt_lambda_decay; // reduce damping
    }

    if (do_update)
      x += dx;
    else
      continue;

    if (params_.optimizer_debug) {
      std::cout << "(Gravity refinement) res: " << r.norm() << ", rho: " << rho
                << ", lambda: " << lambda
                << ", vel: " << x.topRows<3>().transpose();
      std::cout << std::endl;
    }

    alpha = x(kVelocityErrorSize);
    beta = x(kVelocityErrorSize + 1);

    if (dx.norm() / x.norm() < params_.init_grav_opt_converge_thres) {
      break;
    }
  }

  vg.topRows<kVelocityErrorSize>() = x.topRows<kVelocityErrorSize>();
  vg.bottomRows<kGravitySize>() = get_gravity(alpha, beta, params_.gravity_mag);
  return true;
}

bool DynamicInitializer::solve_rotation(
    const std::map<double, std::shared_ptr<ov_core::CpiV1>>
        &map_camera_cpi_I0toIi,
    const std::unordered_map<
        size_t, std::unordered_map<size_t, std::unordered_map<double, Vec2>>>
        &feat_norms,
    double time0, double time1, size_t cam_id0, size_t cam_id1, Mat3 &R) {
  Mat3 xxF = Mat3::Zero();
  Mat3 yyF = Mat3::Zero();
  Mat3 zzF = Mat3::Zero();
  Mat3 xyF = Mat3::Zero();
  Mat3 yzF = Mat3::Zero();
  Mat3 zxF = Mat3::Zero();

  Mat3 R_ItoC0 =
      quat_2_Rot(params_.camera_extrinsics.at(cam_id0).block(0, 0, 4, 1));
  Mat3 R_ItoC1 =
      quat_2_Rot(params_.camera_extrinsics.at(cam_id1).block(0, 0, 4, 1));

  Mat3 R_CtoC0 =
      R_ItoC0 * map_camera_cpi_I0toIi.at(time0)->R_k2tau * R_ItoC0.transpose();
  Mat3 R_CtoC1 =
      R_ItoC1 * map_camera_cpi_I0toIi.at(time1)->R_k2tau * R_ItoC1.transpose();
  Mat3 R_C1toC0 = R_CtoC0 * R_CtoC1.transpose();

  for (auto const &feat : feat_norms) {
    if (feat.second.find(cam_id0) == feat.second.end() ||
        feat.second.find(cam_id1) == feat.second.end()) {
      continue;
    }
    auto cam_vec0 = feat.second.at(cam_id0);
    auto cam_vec1 = feat.second.at(cam_id1);
    if (cam_vec0.find(time0) == cam_vec0.end() ||
        cam_vec1.find(time1) == cam_vec1.end()) {
      continue;
    }

    // Loop through each observation
    Vec2 uv_norm0, uv_norm1;

    if (cam_vec0.find(time0) == cam_vec0.end() ||
        cam_vec1.find(time1) == cam_vec1.end()) {
      continue;
    }

    uv_norm0 = cam_vec0.at(time0);
    uv_norm1 = cam_vec1.at(time1);

    Vec3 f1 = uv_norm0.homogeneous();
    Vec3 f2 = uv_norm1.homogeneous();
    Mat3 F = f2 * f2.transpose();
    xxF = xxF + f1[0] * f1[0] * F;
    yyF = yyF + f1[1] * f1[1] * F;
    zzF = zzF + f1[2] * f1[2] * F;
    xyF = xyF + f1[0] * f1[1] * F;
    yzF = yzF + f1[1] * f1[2] * F;
    zxF = zxF + f1[2] * f1[0] * F;
  }

  Vec3 cayley = Rot2Cayley(R_C1toC0);
  Vec3 cayley_tmp = Rot2Cayley(R_C1toC0);
  Eigen::Matrix<DataType, 1, 3> jacobian;

  DataType r =
      GetSmallestEVwithJacobian(xxF, yyF, zzF, xyF, yzF, zxF, cayley, jacobian);
  DataType alpha = r;
  DataType res_prev = r;
  VecX dx;

  const size_t num_iter = params_.init_grav_opt_max_iter;
  for (size_t i = 0; i < num_iter; i++) {

    r = GetSmallestEVwithJacobian(xxF, yyF, zzF, xyF, yzF, zxF, cayley,
                                  jacobian);
    dx = jacobian * alpha;
    cayley_tmp = cayley - dx;
    r = GetSmallestEVwithJacobian(xxF, yyF, zzF, xyF, yzF, zxF, cayley_tmp,
                                  jacobian);
    while (r > res_prev) {
      alpha = alpha / 2;
      dx = jacobian * alpha;
      cayley_tmp = cayley - dx;
      r = GetSmallestEVwithJacobian(xxF, yyF, zzF, xyF, yzF, zxF, cayley_tmp,
                                    jacobian);
    }

    cayley = cayley_tmp;
    if (params_.optimizer_debug) {
      std::cout << "res: " << r << ", rot: " << cayley.transpose() << std::endl;
    }

    res_prev = r;
    if (dx.norm() / cayley.norm() < params_.init_grav_opt_converge_thres) {
      break;
    }
  }
  R = R_ItoC0.transpose() * Cayley2RotReduced(cayley) * R_ItoC1;
  return true;
}

bool DynamicInitializer::solve_bg(
    const std::map<double, std::shared_ptr<ov_core::CpiV1>>
        &map_camera_cpi_I0toIi,
    const std::unordered_map<
        size_t, std::unordered_map<size_t, std::unordered_map<double, Vec2>>>
        &feat_norms,
    Vec3 &bg) {
  // Prepare data for bg solver
  std::map<size_t, BgSolverData> bg_solver_data_all;
  std::vector<double> kf_timestamps;
  for (auto cpi : map_camera_cpi_I0toIi) {
    kf_timestamps.push_back(cpi.first);
  }
  prepare_bg_solver_data(feat_norms, kf_timestamps, bg_solver_data_all);

  // Start solver
  Mat3 H = Mat3::Zero();
  Mat3 H_total = Mat3::Zero();
  Eigen::Matrix<DataType, 1, 3> J, J_perturb;
  Eigen::Matrix<DataType, 1, 3> J_total = MatX::Zero(1, 3);
  DataType res_curr, res_new, res_tmp, res;
  DataType lambda = params_.init_grav_opt_init_lambda;
  Vec3 dbg = Vec3::Zero();
  for (int i = 0; i < params_.init_grav_opt_max_iter; i++) {
    // Set zeros
    H.setZero();
    H_total.setZero();
    J.setZero();
    J_perturb.setZero();
    J_total.setZero();
    res_curr = 0;
    res_new = 0;

    for (auto data : bg_solver_data_all) {
      get_bg_jacobians(map_camera_cpi_I0toIi, data.second, dbg, J, res);
      const DataType perturb = 1e-6;
      for (size_t m = 0; m < 3; m++) {
        Vec3 dbg_tmp = dbg;
        dbg_tmp(m) += perturb;
        get_bg_jacobians(map_camera_cpi_I0toIi, data.second, dbg_tmp, J_perturb,
                         res_tmp);
        H.row(m) = (J_perturb - J) / perturb;
      }
      H_total += H;
      J_total += J;
      res_curr += res_tmp;
    }

    MatX D = H_total.diagonal().matrix();
    Mat3 H_total_tmp = H_total;
    H_total_tmp.diagonal().array() += lambda * D.array();

    Vec3 dx = H_total_tmp.llt().solve(-J_total.transpose());
    Vec3 dbg_tmp = dbg + dx;

    for (auto data : bg_solver_data_all) {
      get_bg_jacobians(map_camera_cpi_I0toIi, data.second, dbg_tmp, J, res);
      res_new += res;
    }

    DataType rho = (J_total * dx + 0.5 * dx.transpose() * H_total * dx)(0) /
                   (res_new - res_curr);
    bool do_update = true;

    if (rho < 0.25) {
      lambda /= params_.init_grav_opt_lambda_decay;
      do_update = false;
    } else if (rho > 0.75) {
      lambda *= params_.init_grav_opt_lambda_decay;
    }

    if (do_update) {
      dbg += dx;
      res_curr = res_new;
    }

    if (dx.norm() < params_.init_grav_opt_converge_thres) {
      break;
    }

    if (params_.optimizer_debug) {
      std::cout << "(Bg optimizer) res: " << res_curr << ", rho: " << rho
                << ", lambda: " << lambda << ", bg: " << (bg + dbg).transpose()
                << std::endl;
    }
  }

  bg += dbg;
  if (bg.hasNaN()) {
    PRINT_WARNING("Bias has NaN");
    return false;
  }

  return true;
}

void DynamicInitializer::get_bg_jacobians(
    const std::map<double, std::shared_ptr<ov_core::CpiV1>>
        &map_camera_cpi_I0toIi,
    const BgSolverData &data, const Vec3 &delta_bg,
    Eigen::Matrix<DataType, 1, 3> &jacobian, DataType &res) {
  double time0 = data.time0;
  double time1 = data.time1;
  size_t cam_id0 = data.cam_id0;
  size_t cam_id1 = data.cam_id1;

  auto cpi0 = map_camera_cpi_I0toIi.at(time0);
  auto cpi1 = map_camera_cpi_I0toIi.at(time1);
  auto cam0 = params_.camera_extrinsics.at(cam_id0);
  auto cam1 = params_.camera_extrinsics.at(cam_id1);
  Mat3 R_I0toC0 = quat_2_Rot(cam0.topRows<4>());
  Mat3 R_I1toC1 = quat_2_Rot(cam1.topRows<4>());
  Mat3 Jq0 = cpi0->J_q;
  Mat3 Jq1 = cpi1->J_q;

  Mat3 R_ItoI0 = exp_so3(Jq0 * delta_bg) * cpi0->R_k2tau;
  Mat3 R_ItoI1 = exp_so3(Jq1 * delta_bg) * cpi1->R_k2tau;
  Mat3 R_C1toC0 =
      R_I0toC0 * R_ItoI0 * R_ItoI1.transpose() * R_I1toC1.transpose();

  Vec3 cayley = Rot2Cayley(R_C1toC0);
  Eigen::Matrix<DataType, 1, 3> dlambda_dcayley;
  res = GetSmallestEVwithJacobian(data.xxF, data.yyF, data.zzF, data.xyF,
                                  data.yzF, data.zxF, cayley, dlambda_dcayley);
  Mat3 dcayley_dtheta = GetNumericalJacobians(R_C1toC0);

  jacobian = dlambda_dcayley * dcayley_dtheta *
             (-R_I0toC0 * Jq0 + R_I0toC0 * R_ItoI0 * R_ItoI1.transpose() * Jq1);
}

void DynamicInitializer::prepare_bg_solver_data(
    const std::unordered_map<
        size_t, std::unordered_map<size_t, std::unordered_map<double, Vec2>>>
        &feat_norms,
    const std::vector<double> &kf_timestamps,
    std::map<size_t, BgSolverData> &bg_solver_data) {
  size_t curr_id = 0;
  for (size_t i = 0; i < kf_timestamps.size() - 1; i++) {
    for (size_t j = i + 1; j < kf_timestamps.size(); j++) {
      const double time0 = kf_timestamps.at(i);
      const double time1 = kf_timestamps.at(j);
      for (int cam_id0 = 0; cam_id0 < params_.num_cameras; cam_id0++) {
        for (int cam_id1 = 0; cam_id1 < params_.num_cameras; cam_id1++) {
          BgSolverData data;
          // Get pair info
          data.time0 = time0;
          data.time1 = time1;
          data.cam_id0 = cam_id0;
          data.cam_id1 = cam_id1;

          // Get jacobain related stuff
          int total_feat = 0;
          for (auto const &feat : feat_norms) {
            if (feat.second.find(cam_id0) == feat.second.end() ||
                feat.second.find(cam_id1) == feat.second.end()) {
              continue;
            }
            auto cam_vec0 = feat.second.at(cam_id0);
            auto cam_vec1 = feat.second.at(cam_id1);
            if (cam_vec0.find(time0) == cam_vec0.end() ||
                cam_vec1.find(time1) == cam_vec1.end()) {
              continue;
            }

            // Loop through each observation
            Vec2 uv_norm0, uv_norm1;
            if (cam_vec0.find(time0) == cam_vec0.end() ||
                cam_vec1.find(time1) == cam_vec1.end()) {
              continue;
            }

            uv_norm0 = cam_vec0.at(time0);
            uv_norm1 = cam_vec1.at(time1);

            Vec3 f1 = uv_norm0.homogeneous();
            Vec3 f2 = uv_norm1.homogeneous();
            Mat3 F = f2 * f2.transpose();

            // avoid redundant computation
            data.xxF = data.xxF + f1[0] * f1[0] * F;
            data.yyF = data.yyF + f1[1] * f1[1] * F;
            data.zzF = data.zzF + f1[2] * f1[2] * F;
            data.xyF = data.xyF + f1[0] * f1[1] * F;
            data.yzF = data.yzF + f1[1] * f1[2] * F;
            data.zxF = data.zxF + f1[2] * f1[0] * F;
            total_feat++;
          }
          data.num_feat = total_feat;
          bg_solver_data.insert({curr_id, data});
          curr_id++;
        }
      }
    }
  }
}