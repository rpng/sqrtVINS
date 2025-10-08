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




#include "Solver.h"
#include "state/Propagator.h"
#include "state/StateHelper.h"
#include "utils/Helper.h"
#include "utils/quat_ops.h"

#include "feat/FeatureInitializer.h"
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_srvins;
using namespace ov_type;
using namespace ov_core;
using namespace ov_srvins;

Solver::Solver(
    std::shared_ptr<State> state, const VecX &x_init,
    const std::vector<std::shared_ptr<ov_core::Feature>> &features_vec,
    const std::set<double> &map_camera_times,
    std::shared_ptr<ov_srvins::Propagator> propagator,
    std::shared_ptr<ov_srvins::UpdaterMSCKF> updater_msckf,
    std::shared_ptr<ov_srvins::UpdaterSLAM> updater_slam)
    : state_(state), features_vec_(features_vec),
      map_camera_times_(map_camera_times), propagator_(propagator),
      updater_msckf_(updater_msckf), updater_slam_(updater_slam) {
  StateHelper::initialize_state(state_, x_init, *map_camera_times_.begin());
}

bool Solver::solve() {
  // Don't use FEJ during initialization to avoid slow convergence
  // Reference: Is Iteration Worth It? Revisit Its Impact in Sliding-Window VIO
  // https://www.yxpeng.com/downloads/papers/Chen2025ICRA.pdf
  bool do_fej_backup = state_->options.do_fej;
  state_->options.do_fej = false;

  ov_type::LandmarkRepresentation::Representation feat_rep_msckf_backup =
      state_->options.feat_rep_msckf;

  // Use anchor feature for robustness in during optimization
  state_->options.feat_rep_msckf =
      ov_type::LandmarkRepresentation::ANCHORED_MSCKF_INVERSE_DEPTH;

  // Rank features_vec by the number of measurements and erase the ones that
  // exceed the budgets. We only count the timestamps in map_camera_times
  std::vector<std::pair<size_t, int>> feats_meas;
  size_t curr_vec_id = 0;
  for (auto const &feat : features_vec_) {
    int num_meas = 0;
    for (auto const &cam_t : feat->timestamps) {
      for (auto const &t : cam_t.second) {
        if (map_camera_times_.find(t) != map_camera_times_.end()) {
          num_meas++;
        }
      }
    }
    feats_meas.push_back({curr_vec_id, num_meas});
    curr_vec_id++;
  }
  std::sort(
      feats_meas.begin(), feats_meas.end(),
      [](const std::pair<size_t, int> &a, const std::pair<size_t, int> &b) {
        return a.second > b.second;
      });

  std::vector<std::shared_ptr<Feature>> feat_vec_new;
  for (int i = 0;
       i < std::min(int(feats_meas.size()), state_->init_options.init_max_feat);
       i++) {
    feat_vec_new.push_back(features_vec_.at(feats_meas.at(i).first));
  }
  features_vec_ = feat_vec_new;
  auto features_vec_bak = features_vec_;
  int num_features_before = features_vec_.size();

  for (auto time : map_camera_times_) {
    propagator_->propagate_and_clone(state_, time);
  }
  state_->resize_U_to_square();

  for (int it = 0; it < state_->init_options.init_dyn_mle_max_iter; it++) {
    // Allows for retriangulation
    features_vec_ = features_vec_bak;
    state_->clear();
    state_->setup_matrix_buffer();
    state_->calculate_clone_poses();
    updater_msckf_->update(state_, features_vec_, true, false);
    if (state_->features_MSCKF.empty()) {
      PRINT_WARNING(RED "[init-d]: no feature triangulated! \n" RESET);
      return false;
    }

    StateHelper::iterative_update_llt(state_);
    updater_msckf_->update_features(state_);
    if (convergence_check() && (int)state_->features_MSCKF.size() >=
                                   state_->init_options.init_min_feat)
      break;

    if (state_->imu->value().hasNaN()) {
      PRINT_WARNING(RED "[init-d]: optimization fail! \n" RESET);
      return false;
    }
  }

  // Update covariance
  if (state_->init_options.init_dyn_mle_max_iter != 0 &&
      (int)state_->features_MSCKF.size() >=
          state_->init_options.init_min_feat) {
    state_->clear();
    state_->calculate_clone_poses();
    if (state_->options.do_fej) {
      state_->calculate_clone_poses_fej();
    }
    state_->setup_matrix_buffer();
    std::vector<std::shared_ptr<ov_core::Feature>> feat_slam_init;
    select_slam_features(feat_slam_init);
    updater_slam_->delayed_init_from_MSCKF(state_, feat_slam_init);
    updater_msckf_->update(state_, features_vec_, true, true);
    StateHelper::initialize_slam_in_U(state_);
    StateHelper::update_llt(state_, true);
  }

  // Reset fej flag
  state_->options.do_fej = do_fej_backup;
  state_->options.feat_rep_msckf = feat_rep_msckf_backup;
  int total_good_feat =
      state_->features_MSCKF.size() + state_->features_SLAM.size();
  if ((total_good_feat < state_->init_options.init_min_feat) &&
      state_->init_options.init_dyn_mle_max_iter != 0) {
    PRINT_WARNING(
        RED "[init-d]: not enough features triangulated (%d < %d)\n" RESET,
        total_good_feat, num_features_before);
    return false;
  } else {
    // Make sure MSCKF are also cleared
    state_->clear(true);
    return true;
  }
}

bool Solver::convergence_check() {
  // Save previous residual (only include the camera part, the prior is not full
  // rank, thus ignored)
  solver_statistics_.res_prev = solver_statistics_.res_curr;

  // Get current residual (only include the camera part, the prior is not full
  // rank, thus ignored)
  solver_statistics_.res_curr = state_->get_residual_squared_norm();

  // Get x
  double x_norm_sq = state_->get_x_squared_norm();

  // Get dx
  double dx_norm_sq = state_->get_dx_squared_norm();

  // Get ratio
  solver_statistics_.dx_ratio = sqrt(dx_norm_sq / x_norm_sq);
  solver_statistics_.res_ratio =
      abs((solver_statistics_.res_prev - solver_statistics_.res_curr) /
          solver_statistics_.res_prev);
  solver_statistics_.print();

  if (solver_statistics_.dx_ratio <
          state_->init_options.init_ba_dx_converge_thres ||
      solver_statistics_.res_ratio <
          state_->init_options.init_ba_res_converge_thres) {
    return true;
  } else {
    return false;
  }
}

void Solver::select_slam_features(
    std::vector<std::shared_ptr<ov_core::Feature>> &feat_slam_init) {
  // If no SLAM features to be initialized, return
  if (state_->init_options.init_max_slam == 0) {
    return;
  }

  // Statistics
  std::set<int> ids;

  // Bad ones (large reprojection error OR lost track)
  std::set<int> ids_to_remove;

  // Check if this feature is tracked to the latest frame for each camera
  for (auto feat : features_vec_) {
    bool is_lost = true;
    for (auto cam_time : feat->timestamps) {
      if (std::find(cam_time.second.begin(), cam_time.second.end(),
                    state_->timestamp) != cam_time.second.end()) {
        is_lost = false;
        break;
      }
    }

    if (is_lost)
      ids_to_remove.insert(feat->featid);
  }

  //  Select features until we reach the max number or run out of features
  // Step 1: build the grid
  std::vector<std::vector<int>> grid_feat_ids(16); // 4x4 grid
  state_->calculate_clone_poses();
  int max_grid = -1;
  for (auto feat : features_vec_) {
    // Skip if we have already added it
    if (ids_to_remove.find(feat->featid) != ids_to_remove.end() ||
        ids.find(feat->featid) != ids.end())
      continue;

    Vec3 p_FinG;
    auto feat_update_type = state_->features_MSCKF.at(feat->featid);
    if (LandmarkRepresentation::is_relative_representation(
            feat_update_type->feat_representation)) {
      const auto &anchor_cam_pose = state_->cam_pose_buffer.get_buffer_unsafe(
          feat_update_type->anchor_cam_id,
          feat_update_type->anchor_clone_timestamp);
      // assert(anchor_cam_pose.has_value());
      Mat3 R_GtoC = anchor_cam_pose.R_GtoC;
      Vec3 p_CinG = anchor_cam_pose.p_CinG;

      // Feature in the global frame
      p_FinG = R_GtoC.transpose() * feat_update_type->get_xyz(false) + p_CinG;
    } else {
      p_FinG = feat_update_type->get_xyz(false);
    }

    // Project it to the latest frame
    size_t cam_id = (feat_update_type->anchor_cam_id == -1)
                        ? 0
                        : feat_update_type->anchor_cam_id;
    const auto &cam_pose =
        state_->cam_pose_buffer.get_buffer_unsafe(cam_id, state_->timestamp);
    // assert(cam_pose.has_value());
    Mat3 R_GtoC = cam_pose.R_GtoC;
    Vec3 p_CinG = cam_pose.p_CinG;
    Vec3 p_FinC = R_GtoC * (p_FinG - p_CinG);
    Vec2 uv_norm = (p_FinC.head(2) / p_FinC(2));
    auto intrinsic = state_->cam_intrinsics_cameras.at(
        cam_id); // TODO:which camera to anchor?
    Vec2 uv_dist = intrinsic->distort(uv_norm);

    // Get the grid id
    int grid_x = int(uv_dist(0) * 4) / intrinsic->w();
    int grid_y = int(uv_dist(1) * 4) / intrinsic->h();

    if (grid_x >= 4 || grid_x < 0 || grid_y >= 4 || grid_y < 0) {
      ids_to_remove.insert(feat_update_type->featid);
      continue;
    }
    int grid_id = grid_y * 4 + grid_x;
    grid_feat_ids.at(grid_id).push_back(feat_update_type->featid);
    max_grid = std::max(max_grid, int(grid_feat_ids.at(grid_id).size()));

    // NOTE: also change the anchor if needed
    if (LandmarkRepresentation::is_relative_representation(
            feat_update_type->feat_representation)) {
      feat_update_type->anchor_clone_timestamp = state_->timestamp;
      feat_update_type->set_from_xyz(p_FinC, true);
      feat_update_type->set_from_xyz(p_FinC, false);
      feat->anchor_clone_timestamp = state_->timestamp;
      feat->p_FinA = p_FinC;
    }
  }

  // Step 2: select features from each grid one by one until we reach the max
  // number
  bool add_all = (int)features_vec_.size() - (int)ids_to_remove.size() <=
                 state_->init_options.init_max_slam;
  if (add_all) {
    for (const auto &feat_ids : grid_feat_ids) {
      for (const auto &id : feat_ids) {
        ids.insert(id);
      }
    }
  } else {
    for (int i = 0; i < max_grid; i++) {
      for (const auto &feat_ids : grid_feat_ids) {
        if (i < (int)feat_ids.size()) {
          ids.insert(feat_ids.at(i));
          if ((int)ids.size() >= state_->init_options.init_max_slam) {
            break;
          }
        }
      }
    }
  }

  // clean up feature_vec by moving slam features into feat_slam_init and
  // removed from features_vec_
  for (auto it = features_vec_.begin(); it != features_vec_.end();) {
    size_t feat_id = (*it)->featid;
    if (ids.find(feat_id) != ids.end()) {
      feat_slam_init.push_back(*it);
      it = features_vec_.erase(it);
    } else {
      it++;
    }
  }
}
