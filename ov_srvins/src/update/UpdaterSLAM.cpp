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





#include "UpdaterSLAM.h"

#include "UpdaterHelper.h"

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
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

UpdaterSLAM::UpdaterSLAM(UpdaterOptions &options_slam,
                         UpdaterOptions &options_aruco,
                         ov_core::FeatureInitializerOptions &feat_init_options)
    : options_slam_(options_slam), options_aruco_(options_aruco) {

  // Save our feature initializer
  initializer_feat_ = std::shared_ptr<ov_core::FeatureInitializer>(
      new ov_core::FeatureInitializer(feat_init_options));

  // Initialize the chi squared test table with confidence level 0.95
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 500; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table_[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

void UpdaterSLAM::delayed_init(
    std::shared_ptr<State> state,
    std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // Return if no features
  if (feature_vec.empty())
    return;

  // Start timing
  boost::posix_time::ptime rT0, rT1, rT2, rT3;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // 0. Get all timestamps our clones are at (and thus valid measurement times)
  std::vector<double> clonetimes;
  //  clonetimes.emplace_back(state->_timestamp); // Take IMU poses as an extra
  //  clones
  for (const auto &clone_imu : state->clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. Clean all feature measurements and make sure they all have valid clone
  // times
  auto it0 = feature_vec.begin();
  while (it0 != feature_vec.end()) {

    // Clean the feature
    (*it0)->clean_old_measurements(clonetimes);

    // Count how many measurements
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (*it0)->timestamps[pair.first].size();
    }

    // Remove if we don't have enough
    if (ct_meas < 2) {
      (*it0)->to_delete = true;
      it0 = feature_vec.erase(it0);
    } else {
      it0++;
    }
  }
  rT1 = boost::posix_time::microsec_clock::local_time();

  // 2. Create vector of cloned *CAMERA* poses at each of our clone timesteps
  std::unordered_map<size_t,
                     std::unordered_map<double, FeatureInitializer::ClonePose>>
      clones_cam;
  for (const auto &clone_calib : state->calib_IMUtoCAM) {
    // For this camera, create the vector of camera poses
    std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
    for (const auto &clone_imu : state->clones_IMU) {

      const auto cam_pose = state->cam_pose_buffer.get_buffer_unsafe(
          clone_calib.first, clone_imu.first);

      // Append to our map
      clones_cami.insert(
          {clone_imu.first,
           FeatureInitializer::ClonePose(cam_pose.R_GtoC, cam_pose.p_CinG)});
    }

    // Append to our map
    clones_cam.insert({clone_calib.first, clones_cami});
  }

  // 3. Try to triangulate all MSCKF or new SLAM features that have measurements
  auto it1 = feature_vec.begin();
  while (it1 != feature_vec.end()) {
    // Skip those that has been triangulated
    if (state->features_MSCKF.find((*it1)->featid) !=
        state->features_MSCKF.end()) {
      it1++;
      continue;
    }

    // Triangulate the feature and remove if it fails
    bool success_tri = true;
    if (initializer_feat_->config().triangulate_1d) {
      success_tri =
          initializer_feat_->single_triangulation_1d(*it1, clones_cam);
    } else {
      success_tri = initializer_feat_->single_triangulation(*it1, clones_cam);
    }

    // Gauss-newton refine the feature
    bool success_refine = true;
    if (initializer_feat_->config().refine_features) {
      success_refine = initializer_feat_->single_gaussnewton(*it1, clones_cam);
    }

    // Remove the feature if not a success
    if (!success_tri || !success_refine) {
      (*it1)->to_delete = true;
      it1 = feature_vec.erase(it1);
      continue;
    }
    it1++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // 4. Compute linear system for each feature, nullspace project, and reject
  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {

    // Convert our feature into our current format
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = (*it2)->featid;
    feat.uvs = (*it2)->uvs;
    feat.uvs_norm = (*it2)->uvs_norm;
    feat.timestamps = (*it2)->timestamps;

    // If we are using single inverse depth, then it is equivalent to using the
    // msckf inverse depth
    auto feat_rep = ((int)feat.featid < state->options.max_aruco_features)
                        ? state->options.feat_rep_aruco
                        : state->options.feat_rep_slam;
    feat.feat_representation = feat_rep;

    // Save the position and its fej value
    if (LandmarkRepresentation::is_relative_representation(
            feat.feat_representation)) {
      feat.anchor_cam_id = (*it2)->anchor_cam_id;
      feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
      feat.p_FinA = (*it2)->p_FinA;
      feat.p_FinA_fej = (*it2)->p_FinA;
    } else {
      feat.p_FinG = (*it2)->p_FinG;
      feat.p_FinG_fej = (*it2)->p_FinG;
    }

    // Our return values (feature jacobian, state jacobian, residual, and order
    // of state jacobian)
    MatX H_f;
    MatX H_x;
    VecX res;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // Get the Jacobian for this feature
    bool get_jacobian_success = UpdaterHelper::get_feature_jacobian_full(
        state, feat, H_f, H_x, res, Hx_order);
    if (!get_jacobian_success) {
      it2 = feature_vec.erase(it2);
      continue;
    }

    // Create feature pointer (we will always create it of size three since we
    // initialize the single invese depth as a msckf anchored representation)
    int landmark_size = 3;
    auto landmark = std::make_shared<Landmark>(landmark_size);
    landmark->featid = feat.featid;
    landmark->feat_representation = feat_rep;
    landmark->unique_camera_id = (*it2)->anchor_cam_id;
    if (LandmarkRepresentation::is_relative_representation(
            feat.feat_representation)) {
      landmark->anchor_cam_id = feat.anchor_cam_id;
      landmark->anchor_clone_timestamp = feat.anchor_clone_timestamp;
      landmark->set_from_xyz(feat.p_FinA, false);
      landmark->set_from_xyz(feat.p_FinA_fej, true);
    } else {
      landmark->set_from_xyz(feat.p_FinG, false);
      landmark->set_from_xyz(feat.p_FinG_fej, true);
    }

    // Measurement noise matrix
    DataType sigma_pix_inv =
        ((int)feat.featid < state->options.max_aruco_features)
            ? options_aruco_.sigma_pix_inv
            : options_slam_.sigma_pix_inv;

    // Try to initialize, delete new pointer if we failed
    DataType chi2_multipler =
        ((int)feat.featid < state->options.max_aruco_features)
            ? options_aruco_.chi2_multipler
            : options_slam_.chi2_multipler;
    if (StateHelper::initialize(state, landmark, Hx_order, H_x, H_f, res,
                                chi2_multipler, sigma_pix_inv)) {
      state->features_SLAM.insert({(*it2)->featid, landmark});
      (*it2)->to_delete = true;
      state->erase_feat((*it2)->featid);
      it2++;
    } else {
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
    }
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // Debug print timing information
  if (!feature_vec.empty()) {
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds to clean\n",
              (rT1 - rT0).total_microseconds() * 1e-6);
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds to triangulate\n",
              (rT2 - rT1).total_microseconds() * 1e-6);
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds initialize (%d features)\n",
              (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size());
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds total\n",
              (rT3 - rT1).total_microseconds() * 1e-6);
  }
}

void UpdaterSLAM::delayed_init_from_MSCKF(
    std::shared_ptr<State> state,
    std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // Return if no features
  if (feature_vec.empty())
    return;

  // 4. Compute linear system for each feature, nullspace project, and reject
  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {

    // Convert our feature into our current format
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = (*it2)->featid;
    feat.uvs = (*it2)->uvs;
    feat.uvs_norm = (*it2)->uvs_norm;
    feat.timestamps = (*it2)->timestamps;

    // If we are using single inverse depth, then it is equivalent to using the
    // msckf inverse depth
    auto feat_rep = ((int)feat.featid < state->options.max_aruco_features)
                        ? state->options.feat_rep_aruco
                        : state->options.feat_rep_slam;
    feat.feat_representation = feat_rep;

    // Save the position and its fej value
    if (LandmarkRepresentation::is_relative_representation(
            feat.feat_representation)) {
      feat.anchor_cam_id = (*it2)->anchor_cam_id;
      feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
      feat.p_FinA = (*it2)->p_FinA;
      feat.p_FinA_fej = (*it2)->p_FinA;
    } else {
      feat.p_FinG = (*it2)->p_FinG;
      feat.p_FinG_fej = (*it2)->p_FinG;
    }

    // Our return values (feature jacobian, state jacobian, residual, and order
    // of state jacobian)
    MatX H_f;
    MatX H_x;
    VecX res;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // Get the Jacobian for this feature
    bool get_jacobian_success = UpdaterHelper::get_feature_jacobian_full(
        state, feat, H_f, H_x, res, Hx_order);
    if (!get_jacobian_success) {
      it2 = feature_vec.erase(it2);
      continue;
    }

    // Create feature pointer (we will always create it of size three since we
    // initialize the single invese depth as a msckf anchored representation)
    auto landmark = state->features_MSCKF.at((*it2)->featid);

    // Measurement noise matrix
    DataType sigma_pix_inv =
        ((int)feat.featid < state->options.max_aruco_features)
            ? options_aruco_.sigma_pix_inv
            : options_slam_.sigma_pix_inv;

    StateHelper::initialize(state, landmark, Hx_order, H_x, H_f, res, -1,
                            sigma_pix_inv);
    state->features_SLAM.insert({(*it2)->featid, landmark});
    it2++;
  }
}

void UpdaterSLAM::update(std::shared_ptr<State> state,
                         std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // Return if no features
  if (feature_vec.empty())
    return;

  // Start timing
  boost::posix_time::ptime rT0, rT1, rT2, rT3;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // 0. Get all timestamps our clones are at (and thus valid measurement times)
  std::vector<double> clonetimes;
  for (const auto &clone_imu : state->clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. Clean all feature measurements and make sure they all have valid clone
  // times
  auto it0 = feature_vec.begin();
  while (it0 != feature_vec.end()) {

    // Clean the feature
    (*it0)->clean_old_measurements(clonetimes);

    // Count how many measurements
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (*it0)->timestamps[pair.first].size();
    }

    // Get the landmark and its representation
    // For single depth representation we need at least two measurement
    // This is because we do nullspace projection
    std::shared_ptr<Landmark> landmark =
        state->features_SLAM.at((*it0)->featid);

    // Remove if we don't have enough
    if (ct_meas < 1) {
      (*it0)->to_delete = true;
      it0 = feature_vec.erase(it0);
    } else {
      it0++;
    }
  }
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Calculate the max possible measurement size
  size_t max_meas_size = 0;
  for (size_t i = 0; i < feature_vec.size(); i++) {
    for (const auto &pair : feature_vec.at(i)->timestamps) {
      max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
    }
  }

  // 4. Compute linear system for each feature, nullspace project, and reject
  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {

    // Ensure we have the landmark and it is the same
    assert(state->features_SLAM.find((*it2)->featid) !=
           state->features_SLAM.end());
    assert(state->features_SLAM.at((*it2)->featid)->featid == (*it2)->featid);

    // Get our landmark from the state
    std::shared_ptr<Landmark> landmark =
        state->features_SLAM.at((*it2)->featid);

    // Convert the state landmark into our current format
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = (*it2)->featid;
    feat.uvs = (*it2)->uvs;
    feat.uvs_norm = (*it2)->uvs_norm;
    feat.timestamps = (*it2)->timestamps;

    // If we are using single inverse depth, then it is equivalent to using the
    // msckf inverse depth
    feat.feat_representation = landmark->feat_representation;

    // Save the position and its fej value
    if (LandmarkRepresentation::is_relative_representation(
            feat.feat_representation)) {
      feat.anchor_cam_id = landmark->anchor_cam_id;
      feat.anchor_clone_timestamp = landmark->anchor_clone_timestamp;
      feat.p_FinA = landmark->get_xyz(false);
      feat.p_FinA_fej = landmark->get_xyz(true);
    } else {
      feat.p_FinG = landmark->get_xyz(false);
      feat.p_FinG_fej = landmark->get_xyz(true);
    }

    // Our return values (feature jacobian, state jacobian, residual, and order
    // of state jacobian)
    MatX H_f;
    MatX H_x;
    VecX res;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // Get the Jacobian for this feature
    bool get_jacobian_success = UpdaterHelper::get_feature_jacobian_full(
        state, feat, H_f, H_x, res, Hx_order);
    if (!get_jacobian_success) {
      it2 = feature_vec.erase(it2);
      continue;
    }

    if (state->init_options.residual_debug) {
      std::cout << "feat id: " << feat.featid
                << ", p_FinA: " << feat.p_FinA.transpose()
                << ", res: " << res.transpose() << std::endl;
    }

    // Place Jacobians in one big Jacobian, since the landmark is already in our
    // state vector
    MatX H_xf = H_x;
    // Else we have the full feature in our state, so just append it
    H_xf.conservativeResize(H_x.rows(), H_x.cols() + H_f.cols());
    H_xf.block(0, H_x.cols(), H_x.rows(), H_f.cols()) = H_f;

    // Append to our Jacobian order vector
    std::vector<std::shared_ptr<Type>> Hxf_order = Hx_order;
    Hxf_order.push_back(landmark);

    const int kMeasSize = H_xf.rows();
    VecX RHTr = VecX::Zero(state->get_state_size());
    MatX HUT = MatX::Zero(kMeasSize, landmark->size() + landmark->id());
    int local_id = 0;
    for (const auto &var : Hxf_order) {
      RHTr.block(var->id(), 0, var->size(), 1).noalias() +=
          H_xf.block(0, local_id, H_xf.rows(), var->size()).transpose() * res *
          options_slam_.sigma_pix_sq_inv;
      local_id += var->size();
    }
    StateHelper::get_factors_for_slam_feature(state, Hxf_order, H_xf, HUT);

    // Cholesky here is more efficient than QR
    MatX S = HUT * HUT.transpose();
    DataType sigma_pix_sq =
        ((int)feat.featid < state->options.max_aruco_features)
            ? options_aruco_.sigma_pix_sq
            : options_slam_.sigma_pix_sq;
    S.diagonal() += sigma_pix_sq * VecX::Ones(S.rows());
    DataType chi2 = res.dot(S.llt().solve(res));

    // Get our threshold (we precompute up to 500 but handle the case that it is
    // more)
    DataType chi2_check;
    if (res.rows() < 500) {
      chi2_check = chi_squared_table_[res.rows()];
    } else {
      boost::math::chi_squared chi_squared_dist(res.rows());
      chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
      PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET,
                    (int)res.rows());
    }

    // Check if we should delete or not
    DataType chi2_multipler =
        ((int)feat.featid < state->options.max_aruco_features)
            ? options_aruco_.chi2_multipler
            : options_slam_.chi2_multipler;
    if (chi2 > chi2_multipler * chi2_check) {
      if ((int)feat.featid < state->options.max_aruco_features) {
        PRINT_WARNING(YELLOW "[SLAM-UP]: rejecting aruco tag %d for chi2 "
                             "thresh (%.3f > %.3f)\n" RESET,
                      (int)feat.featid, chi2, chi2_multipler * chi2_check);
      } else {
        landmark->update_fail_count++;
      }
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
      max_meas_size -= 2 * (*it2)->uvs.size();
      continue;
    }

    // Debug print when we are going to update the aruco tags
    if ((int)feat.featid < state->options.max_aruco_features) {
      PRINT_DEBUG(
          "[SLAM-UP]: accepted aruco tag %d for chi2 thresh (%.3f < %.3f)\n",
          (int)feat.featid, chi2, chi2_multipler * chi2_check);
    }

    it2++;
    state->store_update_factor(HUT * options_slam_.sigma_pix_inv, RHTr);
    if (!state->is_initialized) {
      state->store_update_jacobians(H_xf * options_slam_.sigma_pix_inv,
                                    res * options_slam_.sigma_pix_inv,
                                    Hxf_order);
    }
  }

  rT2 = boost::posix_time::microsec_clock::local_time();

  // We have appended all features to our Hx_big, res_big
  // Delete it so we do not reuse information
  for (size_t f = 0; f < feature_vec.size(); f++) {
    feature_vec[f]->to_delete = true;
  }

  rT3 = boost::posix_time::microsec_clock::local_time();

  // Debug print timing information
  PRINT_ALL("[SLAM-UP]: %.4f seconds to clean\n",
            (rT1 - rT0).total_microseconds() * 1e-6);
  PRINT_ALL("[SLAM-UP]: %.4f seconds creating linear system\n",
            (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_ALL("[SLAM-UP]: %.4f seconds to update (%d feats of %d size)\n",
            (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size(),
            max_meas_size);
  PRINT_ALL("[SLAM-UP]: %.4f seconds total\n",
            (rT3 - rT1).total_microseconds() * 1e-6);
}

void UpdaterSLAM::change_anchors(std::shared_ptr<State> state) {

  // Return if we do not have enough clones
  if ((int)state->clones_IMU.size() <= state->options.max_clone_size + 1) {
    return;
  }

  // Get the marginalization timestep, and change the anchor for any feature
  // seen from it NOTE: for now we have anchor the feature in the same camera as
  // it is before NOTE: this also does not change the representation of the
  // feature at all right now
  double marg_timestep = state->margtimestep();
  for (auto &f : state->features_SLAM) {
    // Skip any features that are in the global frame
    if (f.second->feat_representation ==
            LandmarkRepresentation::Representation::GLOBAL_3D ||
        f.second->feat_representation ==
            LandmarkRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH)
      continue;
    // Else lets see if it is anchored in the clone that will be marginalized
    assert(marg_timestep <= f.second->anchor_clone_timestamp);
    if (f.second->anchor_clone_timestamp == marg_timestep) {
      perform_anchor_change(state, f.second, state->timestamp,
                            f.second->anchor_cam_id);
    }
  }
}

void UpdaterSLAM::perform_anchor_change(std::shared_ptr<State> state,
                                        std::shared_ptr<Landmark> landmark,
                                        double new_anchor_timestamp,
                                        size_t new_cam_id) {

  // Assert that this is an anchored representation
  assert(LandmarkRepresentation::is_relative_representation(
      landmark->feat_representation));
  assert(landmark->anchor_cam_id != -1);

  // Create current feature representation
  UpdaterHelper::UpdaterHelperFeature old_feat;
  old_feat.featid = landmark->featid;
  old_feat.feat_representation = landmark->feat_representation;
  old_feat.anchor_cam_id = landmark->anchor_cam_id;
  old_feat.anchor_clone_timestamp = landmark->anchor_clone_timestamp;
  old_feat.p_FinA = landmark->get_xyz(false);
  old_feat.p_FinA_fej = landmark->get_xyz(true);

  // Get Jacobians of p_FinG wrt old representation
  Mat3 H_f_old;
  std::vector<MatX> H_x_old;
  std::vector<std::shared_ptr<Type>> x_order_old;
  UpdaterHelper::get_feature_jacobian_representation(state, old_feat, H_f_old,
                                                     H_x_old, x_order_old);

  // Create future feature representation
  UpdaterHelper::UpdaterHelperFeature new_feat;
  new_feat.featid = landmark->featid;
  new_feat.feat_representation = landmark->feat_representation;
  new_feat.anchor_cam_id = new_cam_id;
  new_feat.anchor_clone_timestamp = new_anchor_timestamp;

  //==========================================================================
  //==========================================================================

  // OLD: anchor camera position and orientation
  const auto pose_A_old = state->cam_pose_buffer.get_buffer_unsafe(
      old_feat.anchor_cam_id, old_feat.anchor_clone_timestamp);
  Mat3 R_GtoOLD = pose_A_old.R_GtoC;
  Vec3 p_OLDinG = pose_A_old.p_CinG;

  // NEW: anchor camera position and orientation
  const auto pose_A_new = state->cam_pose_buffer.get_buffer_unsafe(
      new_feat.anchor_cam_id, new_feat.anchor_clone_timestamp);
  Mat3 R_GtoNEW = pose_A_new.R_GtoC;
  Vec3 p_NEWinG = pose_A_new.p_CinG;

  // Calculate transform between the old anchor and new one
  Mat3 R_OLDtoNEW = R_GtoNEW * R_GtoOLD.transpose();
  Vec3 p_OLDinNEW = R_GtoNEW * (p_OLDinG - p_NEWinG);
  new_feat.p_FinA = R_OLDtoNEW * landmark->get_xyz(false) + p_OLDinNEW;

  //==========================================================================
  //==========================================================================

  // OLD: anchor camera position and orientation
  const auto pose_A_old_fej = state->cam_pose_fej_buffer.get_buffer_unsafe(
      old_feat.anchor_cam_id, old_feat.anchor_clone_timestamp);
  Mat3 R_GtoOLD_fej = pose_A_old_fej.R_GtoC;
  Vec3 p_OLDinG_fej = pose_A_old_fej.p_CinG;

  // NEW: anchor camera position and orientation
  const auto pose_A_new_fej = state->cam_pose_fej_buffer.get_buffer_unsafe(
      new_feat.anchor_cam_id, new_feat.anchor_clone_timestamp);
  Mat3 R_GtoNEW_fej = pose_A_new_fej.R_GtoC;
  Vec3 p_NEWinG_fej = pose_A_new_fej.p_CinG;

  // Calculate transform between the old anchor and new one
  Mat3 R_OLDtoNEW_fej = R_GtoNEW_fej * R_GtoOLD_fej.transpose();
  Vec3 p_OLDinNEW_fej = R_GtoNEW_fej * (p_OLDinG_fej - p_NEWinG_fej);
  new_feat.p_FinA_fej =
      R_OLDtoNEW_fej * landmark->get_xyz(true) + p_OLDinNEW_fej;

  // Get Jacobians of p_FinG wrt new representation
  Mat3 H_f_new;
  std::vector<MatX> H_x_new;
  std::vector<std::shared_ptr<Type>> x_order_new;
  UpdaterHelper::get_feature_jacobian_representation(state, new_feat, H_f_new,
                                                     H_x_new, x_order_new);

  //==========================================================================
  //==========================================================================
  // Loop through all our orders and append them
  std::vector<std::shared_ptr<Type>> phi_order_OLD;
  int current_it = 0;
  std::map<std::shared_ptr<Type>, int> Phi_id_map;
  for (const auto &var : x_order_old) {
    if (Phi_id_map.find(var) == Phi_id_map.end()) {
      Phi_id_map.insert({var, current_it});
      phi_order_OLD.push_back(var);
      current_it += var->size();
    }
  }
  for (const auto &var : x_order_new) {
    if (Phi_id_map.find(var) == Phi_id_map.end()) {
      Phi_id_map.insert({var, current_it});
      phi_order_OLD.push_back(var);
      current_it += var->size();
    }
  }
  Phi_id_map.insert({landmark, current_it});
  phi_order_OLD.push_back(landmark);
  current_it += landmark->size();

  // Anchor change Jacobian
  const int kFeatSize = 3;
  MatX Phi = MatX::Zero(kFeatSize, current_it);
  MatX Q = MatX::Zero(kFeatSize, kFeatSize);

  // Inverse of our new representation
  // pf_new_error =
  // Hfnew^{-1}*(Hfold*pf_olderror+Hxold*x_olderror-Hxnew*x_newerror)
  MatX H_f_new_inv;
  H_f_new_inv = H_f_new.colPivHouseholderQr().solve(
      Eigen::Matrix<DataType, 3, 3>::Identity());

  // Place Jacobians for old anchor
  for (size_t i = 0; i < H_x_old.size(); i++) {
    Phi.block(0, Phi_id_map.at(x_order_old[i]), kFeatSize,
              x_order_old[i]->size())
        .noalias() += H_f_new_inv * H_x_old[i];
  }

  // Place Jacobians for old feat
  Phi.block(0, Phi_id_map.at(landmark), kFeatSize, kFeatSize) =
      H_f_new_inv * H_f_old;

  // Place Jacobians for new anchor
  for (size_t i = 0; i < H_x_new.size(); i++) {
    Phi.block(0, Phi_id_map.at(x_order_new[i]), kFeatSize,
              x_order_new[i]->size())
        .noalias() -= H_f_new_inv * H_x_new[i];
  }

  // Set state from new feature
  landmark->featid = new_feat.featid;
  landmark->feat_representation = new_feat.feat_representation;
  landmark->anchor_cam_id = new_feat.anchor_cam_id;
  landmark->anchor_clone_timestamp = new_feat.anchor_clone_timestamp;
  landmark->set_from_xyz(new_feat.p_FinA, false);
  landmark->set_from_xyz(new_feat.p_FinA_fej, true);

  // Perform covariance propagation
  StateHelper::propagate_slam_anchor_feature(state, landmark, Phi,
                                             phi_order_OLD);
}
