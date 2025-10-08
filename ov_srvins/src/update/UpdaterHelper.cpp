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





#include "UpdaterHelper.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/Helper.h"

#include "utils/DataType.h"
#include "utils/quat_ops.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_srvins;

void UpdaterHelper::get_feature_jacobian_representation(
    std::shared_ptr<State> state, UpdaterHelperFeature &feature, Mat3 &H_f,
    std::vector<MatX> &H_x, std::vector<std::shared_ptr<Type>> &x_order) {

  // Global XYZ representation
  if (feature.feat_representation ==
      LandmarkRepresentation::Representation::GLOBAL_3D) {
    H_f.setIdentity();
    return;
  }

  // Global inverse depth representation
  if (feature.feat_representation ==
      LandmarkRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH) {

    // Get the feature linearization point
    Vec3 p_FinG = (state->options.do_fej) ? feature.p_FinG_fej : feature.p_FinG;

    // Get inverse depth representation (should ma  tch what is in Landmark.cpp)
    DataType g_rho = 1 / p_FinG.norm();
    DataType g_phi = std::acos(g_rho * p_FinG(2));
    // DataType g_theta = std::asin(g_rho*p_FinG(1)/std::sin(g_phi));
    DataType g_theta = std::atan2(p_FinG(1), p_FinG(0));
    Vec3 p_invFinG({g_theta, g_phi, g_rho});
    // p_invFinG(0) = g_theta;
    // p_invFinG(1) = g_phi;
    // p_invFinG(2) = g_rho;

    // Get inverse depth bearings
    DataType sin_th = std::sin(g_theta);
    DataType cos_th = std::cos(g_theta);
    DataType sin_phi = std::sin(g_phi);
    DataType cos_phi = std::cos(g_phi);
    DataType rho = g_rho;

    // Construct the Jacobian
    H_f << -(1.0 / rho) * sin_th * sin_phi, (1.0 / rho) * cos_th * cos_phi,
        -(1.0 / (rho * rho)) * cos_th * sin_phi, (1.0 / rho) * cos_th * sin_phi,
        (1.0 / rho) * sin_th * cos_phi, -(1.0 / (rho * rho)) * sin_th * sin_phi,
        0.0, -(1.0 / rho) * sin_phi, -(1.0 / (rho * rho)) * cos_phi;
    return;
  }

  //======================================================================
  //======================================================================
  //======================================================================

  // Assert that we have an anchor pose for this feature
  assert(feature.anchor_cam_id != -1);
  std::shared_ptr<PoseJPL> clone_Ii =
      state->get_clone_pose(feature.anchor_clone_timestamp);

  // Anchor pose orientation and position, and camera calibration for our anchor
  // camera
  Mat3 R_ItoC = state->calib_IMUtoCAM.at(feature.anchor_cam_id)->Rot();
  Vec3 p_IinC = state->calib_IMUtoCAM.at(feature.anchor_cam_id)->pos();
  Mat3 R_GtoI = clone_Ii->Rot();
  Vec3 p_IinG = clone_Ii->pos();
  Vec3 p_FinA = feature.p_FinA;

  // If I am doing FEJ, I should FEJ the anchor states (should we fej
  // calibration???) Also get the FEJ position of the feature if we are
  if (state->options.do_fej) {
    // "Best" feature in the global frame
    Vec3 p_FinG_best =
        R_GtoI.transpose() * R_ItoC.transpose() * (feature.p_FinA - p_IinC) +
        p_IinG;
    // Transform the best into our anchor frame using FEJ
    R_GtoI = clone_Ii->Rot_fej();
    p_IinG = clone_Ii->pos_fej();
    p_FinA = (R_GtoI.transpose() * R_ItoC.transpose()).transpose() *
                 (p_FinG_best - p_IinG) +
             p_IinC;
  }
  Mat3 R_CtoG = R_GtoI.transpose() * R_ItoC.transpose();

  // Jacobian for our anchor pose
  static Eigen::Matrix<DataType, 3, 6> H_anc;
  H_anc.leftCols<3>().noalias() =
      -R_GtoI.transpose() * skew_x(R_ItoC.transpose() * (p_FinA - p_IinC));
  H_anc.rightCols<3>().setIdentity();

  // Add anchor Jacobians to our return vector
  x_order.push_back(clone_Ii);
  H_x.push_back(H_anc);

  // Get calibration Jacobians (for anchor clone)
  if (state->options.do_calib_camera_pose) {
    static Eigen::Matrix<DataType, 3, 6> H_calib;
    H_calib.leftCols<3>().noalias() = -R_CtoG * skew_x(p_FinA - p_IinC);
    H_calib.rightCols<3>().noalias() = -R_CtoG;
    x_order.push_back(state->calib_IMUtoCAM.at(feature.anchor_cam_id));
    H_x.push_back(H_calib);
  }

  // If we are doing anchored XYZ feature
  if (feature.feat_representation ==
      LandmarkRepresentation::Representation::ANCHORED_3D) {
    H_f = R_CtoG;
    return;
  }

  // If we are doing full inverse depth
  if (feature.feat_representation ==
      LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH) {

    // Get inverse depth representation (should match what is in Landmark.cpp)
    DataType a_rho = 1 / p_FinA.norm();
    DataType a_phi = std::acos(a_rho * p_FinA(2));
    DataType a_theta = std::atan2(p_FinA(1), p_FinA(0));
    Vec3 p_invFinA({a_theta, a_phi, a_rho});

    // Using anchored inverse depth
    DataType sin_th = std::sin(a_theta);
    DataType cos_th = std::cos(a_theta);
    DataType sin_phi = std::sin(a_phi);
    DataType cos_phi = std::cos(a_phi);
    DataType rho = a_rho;

    // Jacobian of anchored 3D position wrt inverse depth parameters
    Mat3 d_pfinA_dpinv;
    d_pfinA_dpinv << -(1.0 / rho) * sin_th * sin_phi,
        (1.0 / rho) * cos_th * cos_phi, -(1.0 / (rho * rho)) * cos_th * sin_phi,
        (1.0 / rho) * cos_th * sin_phi, (1.0 / rho) * sin_th * cos_phi,
        -(1.0 / (rho * rho)) * sin_th * sin_phi, 0.0, -(1.0 / rho) * sin_phi,
        -(1.0 / (rho * rho)) * cos_phi;
    H_f = R_CtoG * d_pfinA_dpinv;
    return;
  }

  // If we are doing the MSCKF version of inverse depth
  if (feature.feat_representation ==
      LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH) {

    // Get inverse depth representation (should match what is in Landmark.cpp)
    Eigen::Matrix<DataType, 3, 1> p_invFinA_MSCKF;
    p_invFinA_MSCKF(0) = p_FinA(0) / p_FinA(2);
    p_invFinA_MSCKF(1) = p_FinA(1) / p_FinA(2);
    p_invFinA_MSCKF(2) = 1 / p_FinA(2);

    // Using the MSCKF version of inverse depth
    DataType alpha = p_invFinA_MSCKF(0);
    DataType beta = p_invFinA_MSCKF(1);
    DataType rho = p_invFinA_MSCKF(2);

    // Jacobian of anchored 3D position wrt inverse depth parameters
    static Mat3 d_pfinA_dpinv;
    d_pfinA_dpinv << (1.0 / rho), 0.0, -(1.0 / (rho * rho)) * alpha, 0.0,
        (1.0 / rho), -(1.0 / (rho * rho)) * beta, 0.0, 0.0,
        -(1.0 / (rho * rho));
    H_f = R_CtoG * d_pfinA_dpinv;
    return;
  }

  // Failure, invalid representation that is not programmed
  assert(false);
}

bool UpdaterHelper::get_feature_jacobian_full(
    std::shared_ptr<State> state, UpdaterHelperFeature &feature, MatX &H_f,
    MatX &H_x, VecX &res, std::vector<std::shared_ptr<Type>> &x_order) {
  // Compute the size of the states involved with this feature
  std::map<std::shared_ptr<Type>, size_t> map_hx;
  int total_hx = 0;
  int total_meas = 0;

  // In iterative mode, this order will be kept
  auto feat_msckf = state->features_MSCKF.find(feature.featid);
  if (feat_msckf == state->features_MSCKF.end()) {

    if (state->options.do_calib_camera_pose ||
        state->options.do_calib_camera_intrinsics) {
      for (auto const &pair : feature.timestamps) {

        // If doing calibration extrinsics
        if (state->options.do_calib_camera_pose) {
          std::shared_ptr<PoseJPL> calibration =
              state->calib_IMUtoCAM.at(pair.first);

          map_hx.insert({calibration, total_hx});
          x_order.push_back(calibration);
          total_hx += calibration->size();
        }

        // If doing calibration intrinsics
        if (state->options.do_calib_camera_intrinsics) {
          std::shared_ptr<Vec> distortion =
              state->cam_intrinsics.at(pair.first);

          map_hx.insert({distortion, total_hx});
          x_order.push_back(distortion);
          total_hx += distortion->size();
        }
      }
    }

    for (auto const &pair : feature.timestamps) {
      for (int m = 0; m < (int)pair.second.size(); m++) {
        std::shared_ptr<PoseJPL> clone_Ci =
            state->get_clone_pose(pair.second.at(m));
        if (clone_Ci == nullptr) {
          continue;
        }
        total_meas++;

        if (map_hx.find(clone_Ci) == map_hx.end()) {
          map_hx.insert({clone_Ci, total_hx});
          x_order.push_back(clone_Ci);
          total_hx += clone_Ci->size();
        }
      }
    }

    // If we are using an anchored representation, make sure that the anchor is
    // also added
    if (LandmarkRepresentation::is_relative_representation(
            feature.feat_representation)) {

      // Assert we have a clone
      assert(feature.anchor_cam_id != -1);

      // Add this anchor if it is not added already
      std::shared_ptr<PoseJPL> clone_Ai =
          state->get_clone_pose(feature.anchor_clone_timestamp);
      if (map_hx.find(clone_Ai) == map_hx.end()) {
        map_hx.insert({clone_Ai, total_hx});
        x_order.push_back(clone_Ai);
        total_hx += clone_Ai->size();
      }

      // Also add its calibration if we are doing calibration
      if (state->options.do_calib_camera_pose) {
        // Add this anchor if it is not added already
        std::shared_ptr<PoseJPL> clone_calib =
            state->calib_IMUtoCAM.at(feature.anchor_cam_id);
        if (map_hx.find(clone_calib) == map_hx.end()) {
          map_hx.insert({clone_calib, total_hx});
          x_order.push_back(clone_calib);
          total_hx += clone_calib->size();
        }
      }
    }
    // Reorder state order by id from small to large
    std::vector<std::shared_ptr<Type>> x_reorder = x_order;
    std::map<std::shared_ptr<Type>, size_t> map_hx_reorder;
    std::sort(x_reorder.begin(), x_reorder.end(),
              [](const std::shared_ptr<Type> &a,
                 const std::shared_ptr<Type> &b) { return a->id() < b->id(); });
    total_hx = 0;
    for (auto const &pair : x_reorder) {
      map_hx_reorder.insert({pair, total_hx});
      total_hx += pair->size();
    }
    map_hx = map_hx_reorder;
    x_order = x_reorder;
  } else {
    // Directly use the order from the state
    x_order = feat_msckf->second->x_order_msckf;
    // Get the total size of the states involved with this feature
    for (auto const &pair : x_order) {
      map_hx.insert({pair, total_hx});
      total_hx += pair->size();
    }
    // Get the total number of measurements
    for (auto const &pair : feature.timestamps) {
      for (int m = 0; m < (int)pair.second.size(); m++) {
        std::shared_ptr<PoseJPL> clone_Ci =
            state->get_clone_pose(pair.second.at(m));
        if (clone_Ci == nullptr) {
          continue;
        } else {
          total_meas++;
        }
      }
    }
  }

  //=========================================================================
  //=========================================================================

  // Calculate the position of this feature in the global frame
  // If anchored, then we need to calculate the position of the feature in the
  // global
  static Vec3 p_FinG;
  p_FinG = feature.p_FinG;
  if (LandmarkRepresentation::is_relative_representation(
          feature.feat_representation)) {
    // Assert that we have an anchor pose for this feature
    assert(feature.anchor_cam_id != -1);

    // Feature in the global frame
    const auto &cam_clone = state->cam_pose_buffer.get_buffer_unsafe(
        feature.anchor_cam_id, feature.anchor_clone_timestamp);
    p_FinG = cam_clone.R_GtoC.transpose() * feature.p_FinA + cam_clone.p_CinG;
  }

  // Calculate the position of this feature in the global frame FEJ
  // If anchored, then we can use the "best" p_FinG since the value of p_FinA
  // does not matter
  static Vec3 p_FinG_fej;
  p_FinG_fej = feature.p_FinG_fej;
  if (LandmarkRepresentation::is_relative_representation(
          feature.feat_representation)) {
    p_FinG_fej = p_FinG;
  }
  //=========================================================================
  //=========================================================================

  // Allocate our residual and Jacobians
  int c = 0;
  int jacobsize = 3;
  res = VecX::Zero(2 * total_meas);
  H_f = MatX::Zero(2 * total_meas, jacobsize);
  H_x = MatX::Zero(2 * total_meas, total_hx);

  // Derivative of p_FinG in respect to feature representation.
  // This only needs to be computed once and thus we pull it out of the loop
  Mat3 dpfg_dlambda;
  std::vector<MatX> dpfg_dx;
  std::vector<std::shared_ptr<Type>> dpfg_dx_order;
  UpdaterHelper::get_feature_jacobian_representation(
      state, feature, dpfg_dlambda, dpfg_dx, dpfg_dx_order);

  // Loop through each camera for this feature
  for (auto const &pair : feature.timestamps) {

    // Our calibration between the IMU and CAMi frames
    std::shared_ptr<Vec> distortion = state->cam_intrinsics.at(pair.first);
    std::shared_ptr<PoseJPL> calibration = state->calib_IMUtoCAM.at(pair.first);

    static Mat3 R_ItoC;
    R_ItoC = calibration->Rot();
    static Vec3 p_IinC;
    p_IinC = calibration->pos();
    // Loop through all measurements for this specific camera
    for (size_t m = 0; m < feature.timestamps[pair.first].size(); m++) {

      //=========================================================================
      //=========================================================================
      auto clone_ptr =
          state->clones_IMU.find(feature.timestamps[pair.first].at(m));
      if (clone_ptr == state->clones_IMU.end()) {
        continue;
      }

      // Get current IMU clone state
      std::shared_ptr<PoseJPL> clone_Ii = clone_ptr->second;

      // Project the current feature into the current frame of reference
      static Vec3 p_FinCi;
      const auto &cam_clone = state->cam_pose_buffer.get_buffer_unsafe(
          pair.first, feature.timestamps[pair.first].at(m));
      // assert(cam_clone.has_value());
      p_FinCi.noalias() = (cam_clone.R_GtoC * (p_FinG - cam_clone.p_CinG));
      static Vec2 uv_norm;
      uv_norm << p_FinCi(0) / p_FinCi(2), p_FinCi(1) / p_FinCi(2);
      static Vec2 uv_dist;
      uv_dist = state->cam_intrinsics_cameras.at(pair.first)->distort(uv_norm);

      // Our residual
      static Vec2 uv_m;
      uv_m = feature.uvs[pair.first].at(m);
      res.middleRows<2>(2 * c) = uv_m - uv_dist;
      if (p_FinCi(2) < 0.1) {
        std::cout << RED << "Negative depth detected, discard this feature"
                  << RESET << std::endl;
        return false;
      }

      //=========================================================================
      //=========================================================================

      // If we are doing first estimate Jacobians, then overwrite with the first
      // estimates
      const auto &cam_clone_fej = state->cam_pose_fej_buffer.get_buffer_unsafe(
          pair.first, feature.timestamps[pair.first].at(m));
      if (state->options.do_fej) {
        p_FinCi = cam_clone_fej.R_GtoC * (p_FinG_fej - cam_clone_fej.p_CinG);
      }
      if (p_FinCi(2) < 0.1) {
        std::cout << RED << "Negative depth detected, discard this feature"
                  << RESET << std::endl;
        return false;
      }

      // Compute Jacobians in respect to normalized image coordinates and
      // possibly the camera intrinsics MatX dz_dzn, dz_dzeta;
      static Mat2 dz_dzn;
      static Eigen::Matrix<DataType, 2, 8> dz_dzeta;

      state->cam_intrinsics_cameras.at(pair.first)
          ->compute_distort_jacobian(uv_norm, dz_dzn, dz_dzeta,
                                     state->options.do_calib_camera_intrinsics);

      // Normalized coordinates in respect to projection function
      static MatX dzn_dpfc = MatX::Zero(2, 3);
      dzn_dpfc << 1 / p_FinCi(2), 0, -p_FinCi(0) / (p_FinCi(2) * p_FinCi(2)), 0,
          1 / p_FinCi(2), -p_FinCi(1) / (p_FinCi(2) * p_FinCi(2));

      // Derivative of p_FinCi in respect to p_FinIi
      // MatX dpfc_dpfg = R_ItoC * R_GtoIi;
      static Mat3 dpfc_dpfg = Mat3::Identity();
      dpfc_dpfg =
          state->options.do_fej ? cam_clone_fej.R_GtoC : cam_clone.R_GtoC;

      // Derivative of p_FinCi in respect to camera clone state
      static Eigen::Matrix<DataType, 3, 6> dpfc_dclone;
      dpfc_dclone.leftCols<3>().noalias() = skew_x(p_FinCi - p_IinC) * R_ItoC;
      dpfc_dclone.rightCols<3>().noalias() = -dpfc_dpfg;

      //=========================================================================
      //=========================================================================

      // Precompute some matrices
      static MatX dz_dpfc = MatX::Zero(2, 3);
      dz_dpfc.noalias() = dz_dzn * dzn_dpfc;
      static MatX dz_dpfg = MatX::Zero(2, 3);
      dz_dpfg.noalias() = dz_dpfc * dpfc_dpfg;

      // CHAINRULE: get the total feature Jacobian
      H_f.middleRows<2>(2 * c).noalias() = dz_dpfg * dpfg_dlambda;

      // CHAINRULE: get state clone Jacobian
      H_x.block(2 * c, map_hx[clone_Ii], 2, clone_Ii->size()).noalias() =
          dz_dpfc * dpfc_dclone;

      // CHAINRULE: loop through all extra states and add their
      // NOTE: we add the Jacobian here as we might be in the anchoring pose for
      // this measurement
      for (size_t i = 0; i < dpfg_dx_order.size(); i++) {
        if (feature.timestamps[pair.first].at(m) ==
                feature.anchor_clone_timestamp &&
            dpfg_dx_order.at(i)->id() == clone_Ii->id()) {
          H_x.block(2 * c, map_hx[dpfg_dx_order.at(i)], 2,
                    dpfg_dx_order.at(i)->size())
              .setZero(); // Anchor pose has zero jacobians
        } else {
          H_x.block(2 * c, map_hx[dpfg_dx_order.at(i)], 2,
                    dpfg_dx_order.at(i)->size())
              .noalias() += dz_dpfg * dpfg_dx.at(i);
        }
      }

      //=========================================================================
      //=========================================================================
      // Derivative of p_FinCi in respect to camera calibration (R_ItoC, p_IinC)
      if (state->options.do_calib_camera_pose) {
        // Calculate the Jacobian
        static Eigen::Matrix<DataType, 3, 6> dpfc_dcalib =
            Eigen::Matrix<DataType, 3, 6>::Zero();
        dpfc_dcalib.leftCols<3>() = skew_x(p_FinCi - p_IinC);
        dpfc_dcalib.rightCols<3>() = Mat3::Identity();

        // Chainrule it and add it to the big jacobian
        H_x.block<2, 6>(2 * c, map_hx[calibration]).noalias() +=
            dz_dpfc * dpfc_dcalib;
      }

      // Derivative of measurement in respect to distortion parameters
      if (state->options.do_calib_camera_intrinsics) {
        H_x.block<2, 8>(2 * c, map_hx[distortion]).noalias() = dz_dzeta;
      }

      // Move the Jacobian and residual index forward
      c++;
    }
  }
  return true;
}
