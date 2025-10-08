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





#include "UpdaterMSCKF.h"

#include "UpdaterHelper.h"

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "state/State.h"
#include "state/StateHelper.h"
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

UpdaterMSCKF::UpdaterMSCKF(
    UpdaterOptions &options,
    ov_core::FeatureInitializerOptions &feat_init_options)
    : options_(options) {

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

void UpdaterMSCKF::update(std::shared_ptr<State> state,
                          std::vector<std::shared_ptr<Feature>> &feature_vec,
                          bool is_iterative, bool require_HUT) {
  Timer t, t1, t2, t3, t4;

  // Return if no features
  if (feature_vec.empty())
    return;

  bool do_clean = !is_iterative;

  // Start timing
  boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4, rT5;
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
    int ct_meas = 0;
    if (do_clean) {
      (*it0)->clean_old_measurements(clonetimes);
      // Count how many measurements
      for (const auto &pair : (*it0)->timestamps) {
        ct_meas += (*it0)->timestamps[pair.first].size();
      }
    } else {
      // Count how many measurements and skips those not in the clone times
      for (const auto &pair : (*it0)->timestamps) {
        for (const auto &time : pair.second) {
          if (std::find(clonetimes.begin(), clonetimes.end(), time) !=
              clonetimes.end()) {
            ct_meas++;
          }
        }
      }
    }

    // Remove if we don't have enough
    if (ct_meas < 2) {
      (*it0)->to_delete = true;
      it0 = feature_vec.erase(it0);
    } else {
      it0++;
    }
  }
  // std::cout << "before tri: " << feature_vec.size() << std::endl;
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
    // Skip those that has been triangulated when doing iterative update
    if (is_iterative && state->features_MSCKF.find((*it1)->featid) !=
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
    feat.feat_representation = state->options.feat_rep_msckf;

    // Save the position and its fej value
    auto it_feat_msckf = state->features_MSCKF.find(feat.featid);
    bool is_feat_in_state = (it_feat_msckf != state->features_MSCKF.end());

    if (LandmarkRepresentation::is_relative_representation(
            feat.feat_representation)) {
      feat.anchor_cam_id = (*it2)->anchor_cam_id;
      feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
      feat.p_FinA = is_feat_in_state ? it_feat_msckf->second->get_xyz(false)
                                     : (*it2)->p_FinA;
      feat.p_FinA_fej = is_feat_in_state ? it_feat_msckf->second->get_xyz(true)
                                         : (*it2)->p_FinA;
    } else {
      feat.p_FinG = is_feat_in_state ? it_feat_msckf->second->get_xyz(false)
                                     : (*it2)->p_FinG;
      feat.p_FinG_fej = is_feat_in_state ? it_feat_msckf->second->get_xyz(true)
                                         : (*it2)->p_FinG;
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
    if (state->init_options.residual_debug) {
      std::cout << "id: " << feat.featid
                << ", anchortime: " << std::setprecision(16)
                << feat.anchor_clone_timestamp << std::setprecision(3)
                << ", feat: " << feat.p_FinA.transpose()
                << ", residual: " << res.transpose() << std::endl;
    }

    // Skip this feature if we fail to get the jacobian, these will be the
    // *screw-up features*
    if (!get_jacobian_success) {
      state->erase_feat((*it2)->featid);
      it2 = feature_vec.erase(it2);
      continue;
    }

    // Check residuals in the first iteration for iterative update
    if (is_iterative && state->features_MSCKF.find(feat.featid) ==
                            state->features_MSCKF.end()) {
      bool is_bad = false;

      for (int i = 0; i < res.rows(); i += 2) {
        if (res.middleRows<2>(i).norm() > state->init_options.init_max_reproj) {
          (*it2)->to_delete = true;
          it2 = feature_vec.erase(it2);
          is_bad = true;
          break;
        }
      }

      if (is_bad)
        continue;
    }

    // Add Huber Loss
    if (is_iterative) {
      for (int i = 0; i < res.rows(); i += 2) {
        DataType loss = res.middleRows<2>(i).norm();
        if (loss > state->init_options.init_ba_huber_th) {
          double ratio = sqrt(state->init_options.init_ba_huber_th / loss);
          res.middleRows<2>(i) *= ratio;
          H_x.middleRows<2>(i) *= ratio;
          H_f.middleRows<2>(i) *= ratio;
        }
      }
    }

    // Nullspace project
    MatX H_x1, H_x2, H_f1, HUT;
    VecX res1, res2;
    const int feat_size = H_f.cols();

    reverse_mat(H_x);
    efficient_QR(H_x, res, H_f);
    reverse_mat(H_x);
    if (is_iterative) {
      H_x1 = H_x.topRows(feat_size);
      H_f1 = H_f.topRows(feat_size);
      res1 = res.head(feat_size);
    }

    H_x2 = H_x.bottomRows(H_x.rows() - feat_size);
    res2 = res.tail(res.rows() - feat_size);

    if (require_HUT) {
      MatX U_dense, U_tri;
      StateHelper::get_marginal_U_block(state, Hx_order, U_dense, U_tri);
      HUT = MatX::Zero(H_x2.rows(), U_dense.rows() + U_tri.rows());

      HUT.leftCols(U_dense.rows()).noalias() = H_x2 * U_dense.transpose();
      HUT.rightCols(U_tri.rows()).noalias() =
          H_x2 * U_tri.transpose().triangularView<Eigen::Lower>();

      // Perform Chi2 if not in iterative mode
      // Cholesky here is more efficient than QR
      if (!is_iterative) {
        MatX S = HUT * HUT.transpose();
        S.diagonal() += options_.sigma_pix_sq * VecX::Ones(S.rows());
        DataType chi2 = res2.dot(S.llt().solve(res2));

        // Get our threshold (we precompute up to 500 but handle the case that
        // it is more)
        DataType chi2_check;
        if (res.rows() < 500) {
          chi2_check = chi_squared_table_[res.rows()];
        } else {
          boost::math::chi_squared chi_squared_dist(res.rows());
          chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
          PRINT_WARNING(YELLOW
                        "chi2_check over the residual limit - %d\n" RESET,
                        (int)res.rows());
        }

        // Check if we should delete or not
        if (chi2 > options_.chi2_multipler * chi2_check && do_clean) {
          (*it2)->to_delete = true;
          it2 = feature_vec.erase(it2);
          continue;
        }
      }

      VecX RHTr = VecX::Zero(state->get_state_size());
      VecX RHTr_small = H_x2.transpose() * res2 * options_.sigma_pix_sq_inv;
      int local_id = 0;
      for (const auto &var : Hx_order) {
        RHTr.middleRows(var->id(), var->size()) =
            RHTr_small.middleRows(local_id, var->size());
        local_id += var->size();
      }
      HUT *= options_.sigma_pix_inv;
      state->store_update_factor(HUT,
                                 RHTr); // store the factored version
    }

    if (is_iterative) {
      // Store Jaocbians for iterative update
      H_x2 *= options_.sigma_pix_inv;
      res2 *= options_.sigma_pix_inv;
      state->store_update_jacobians(H_x2, res2,
                                    Hx_order); // store the whiten versin

      // Insert MSCKF feature
      // Create landmarks
      auto it_feat = state->features_MSCKF.find(feat.featid);
      if (it_feat == state->features_MSCKF.end()) {
        int landmark_size = 3;
        auto landmark = std::make_shared<LandmarkMsckf>(landmark_size);
        landmark->featid = feat.featid;
        landmark->feat_representation = feat.feat_representation;
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
        state->features_MSCKF.insert({feat.featid, landmark});
        it_feat = state->features_MSCKF.find(feat.featid);
      }

      // Store Msckf jacobians
      H_f1 *= options_.sigma_pix_inv;
      H_x1 *= options_.sigma_pix_inv;
      res1 *= options_.sigma_pix_inv;
      it_feat->second->store_msckf_jacobians(
          H_f1, H_x1, res1,
          Hx_order); // store the whiten version
    }
    it2++;
  }

  // We have appended all features to our Hx_big, res_big
  // Delete it so we do not reuse information
  for (size_t f = 0; f < feature_vec.size(); f++) {
    feature_vec[f]->to_delete = true;
  }
}

void UpdaterMSCKF::update_features(std::shared_ptr<State> state) {
  // Return if not update required
  if (state->features_MSCKF.empty())
    return;

  VecX dx = state->get_xk_minus_xk1();
  Vec3 feat_dx;
  VecX dx_small;
  for (auto &feat_msckf : state->features_MSCKF) {
    feat_dx = feat_msckf.second->res_msckf;
    dx_small = VecX::Zero(feat_msckf.second->Hx_msckf.cols());
    int curr_it = 0;
    for (const auto &var : feat_msckf.second->x_order_msckf) {
      dx_small.middleRows(curr_it, var->size()) =
          dx.middleRows(var->id(), var->size());
      curr_it += var->size();
    }
    feat_dx -= feat_msckf.second->Hx_msckf * dx_small;

    feat_msckf.second->Hf_msckf.triangularView<Eigen::Upper>().solveInPlace(
        feat_dx);
    feat_msckf.second->update(feat_dx);
    feat_msckf.second->feat_dx = feat_dx;
  }
}