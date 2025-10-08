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





#ifndef OV_SRVINS_DYNAMICINITIALIZER_H
#define OV_SRVINS_DYNAMICINITIALIZER_H

#include "cpi/CpiV1.h"
#include "feat/FeatureDatabase.h"
#include "initializer/InertialInitializerOptions.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "types/Landmark.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "utils/sensor_data.h"

namespace ov_srvins {

struct BgSolverData {
  double time0, time1;
  size_t cam_id0, cam_id1;
  int num_feat = -1;
  Mat3 xxF = Mat3::Zero();
  Mat3 yyF = Mat3::Zero();
  Mat3 zzF = Mat3::Zero();
  Mat3 xyF = Mat3::Zero();
  Mat3 yzF = Mat3::Zero();
  Mat3 zxF = Mat3::Zero();
};

class DynamicInitializer {
public:
  static constexpr int kVelocityAndGravitySize = 6;
  static constexpr int kVelocitySize = 3;
  static constexpr int kGravitySize = 3;
  static constexpr int kVelocityAndGravityErrorSize = 5;
  static constexpr int kVelocityErrorSize = 3;
  static constexpr int kGravityErrorSize = 2;

  /**
   * @brief Default constructor
   * @param params_ Parameters loaded from either ROS or CMDLINE
   * @param db Feature tracker database with all features in it
   * @param propagator Propagator shared with VIO
   * @param updater_msckf MSCKF feature updater shared with VIO
   * @param updater_slam SLAM feature updater shared with VIO
   */
  explicit DynamicInitializer(
      const InertialInitializerOptions &params_,
      std::shared_ptr<ov_core::FeatureDatabase> db,
      std::shared_ptr<ov_srvins::Propagator> propagator_,
      std::shared_ptr<ov_srvins::UpdaterMSCKF> updater_msckf_,
      std::shared_ptr<ov_srvins::UpdaterSLAM> updater_slam_);

  /**
   * @brief Try to get the initialized system state
   * @param state VIO state to be initialized
   */
  bool initialize(std::shared_ptr<ov_srvins::State> &_state);

private:
  /**
   * @brief Perform keyframing to select the best camera frames for
   * initialization. The oldest and latest frames are always selected as
   * keyframes. The rest are selected to maximize the minimal disparity by
   * greedy search.
   * @param feature_database Feature database with all features in it
   * @param keyframe_timestamps Output set of keyframe timestamps
   */
  void
  keyframing(const std::unordered_map<size_t, std::shared_ptr<ov_core::Feature>>
                 &feature_database,
             std::set<double> &keyframe_timestamps);

  bool preintegrate(
      const std::vector<ov_core::ImuData> &imu_data,
      const std::set<double> &keyframe_timestamps, const Vec3 &gyroscope_bias,
      const Vec3 &accelerometer_bias,
      std::map<double, std::shared_ptr<ov_core::CpiV1>> &map_camera_cpi_I0toIi);

  /**
   * @brief Find the eigen vectors for a given camera pair, where the eigen
   * vector that corresponds to the minimum eigen value is the translation
   * direction up to scale
   * @param map_R_CktoI0 All relative rotations between camera and IMU
   * @param bearings_in_I0 All feature bearings in the first IMU frame
   * @param time0 First camera timestamp
   * @param time1 Second camera timestamp
   * @param cam_id0 First camera ID
   * @param cam_id1 Second camera ID
   * @param inlier_ids Output inlier feature IDs that were used to compute the
   * eigen vectors
   * @param eigen_vectors Output eigen vectors (3x3) for this camera pair
   *
   * @return  bool True if we have successfully found the eigen vectors, false
   * otherwise
   */
  bool find_eigen_vectors(
      const std::map<size_t, std::map<double, Mat3>> &map_R_CktoI0,
      const std::map<
          double,
          std::unordered_map<size_t, std::map<size_t, std::shared_ptr<Vec3>>>>
          &bearings_in_I0,
      double time0, double time1, size_t cam_id0, size_t cam_id1,
      std::set<size_t> &inlier_ids, Mat3 &eigen_vectors);

  /**
   * @brief Build the linear system for solving velocity and gravity
   * @param map_camera_cpi_I0toIi All camera CPI preintegrations
   * @param id_to_eigenvectors All eigen vectors for each pair id
   * @param id_to_timepair All time pairs for each pair id
   * @param id_to_campair All camera pairs for each pair id
   * @param A_out Output linear system matrix A
   * @param b_out Output linear system vector b
   * @param x_init Output initial velocity and gravity vector
   */
  void build_linear_system(
      const std::map<double, std::shared_ptr<ov_core::CpiV1>>
          &map_camera_cpi_I0toIi,
      const std::map<size_t, Mat3> &id_to_eigenvectors,
      const std::map<size_t, std::pair<double, double>> &id_to_timepair,
      const std::map<size_t, std::pair<size_t, size_t>> &id_to_campair,
      Eigen::Matrix<DataType, kVelocityAndGravitySize, kVelocityAndGravitySize>
          &A_out,
      Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> &b_out,
      Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> &x_init);

  /**
   * @brief Refine gravity vector by solving a nonlinear least squares problem
   * as the gravity is only 2DOF unknown
   *
   * @param A 6x6 Linear system matrix A with 3DOF velocity and 3DOF gravity
   * @param b 6x1 Linear system vector b with 3DOF velocity and 3DOF gravity
   * @param vg_init Initial velocity and 3DOF gravity vector
   * @param vg Refined velocity and 3DOF gravity vector
   * @return  bool   True if we have successfully refined the gravity
   */
  bool refine_gravity(
      const Eigen::Matrix<DataType, kVelocityAndGravitySize,
                          kVelocityAndGravitySize> &A,
      const Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> &b,
      const Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> &vg_init,
      Eigen::Matrix<DataType, kVelocityAndGravitySize, 1> &vg);

  /**
   * @brief Solve for the relative rotation between two IMU frames
   * Reference:
   * https://openaccess.thecvf.com/content_iccv_2013/papers/Kneip_Direct_Optimization_of_2013_ICCV_paper.pdf
   * @param map_camera_cpi_I0toIi All camera CPI preintegrations
   * @param feat_norms All feature normalized measurements.
   * Layers: [feat_id][cam_id][timestamp][uv]
   * @param time0 First camera timestamp
   * @param time1 Second camera timestamp
   * @param cam_id0 First camera ID
   * @param cam_id1 Second camera ID
   * @param R Output relative rotation between the two IMU frames, R_I1toI0
   * @return  bool    Return true if we have successfully solved for rotation
   */
  bool solve_rotation(
      const std::map<double, std::shared_ptr<ov_core::CpiV1>>
          &map_camera_cpi_I0toIi,
      const std::unordered_map<
          size_t, std::unordered_map<size_t, std::unordered_map<double, Vec2>>>
          &feat_norms,
      double time0, double time1, size_t cam_id0, size_t cam_id1, Mat3 &R);

  /**
   * Recover gyro bias given all preintegrations and feature measurements
   * Reference:
   * https://openaccess.thecvf.com/content/CVPR2023/papers/He_A_Rotation-Translation-Decoupled_Solution_for_Robust_and_Efficient_Visual-Inertial_Initialization_CVPR_2023_paper.pdf
   * https://openaccess.thecvf.com/content_iccv_2013/papers/Kneip_Direct_Optimization_of_2013_ICCV_paper.pdf
   * @param map_camera_cpi_I0toIi All camera CPI preintegrations
   * @param feat_norms All feature normalized measurements.
   * Layers: [feat_id][cam_id][timestamp][uv]
   * @param bg Output gyro bias
   * @return  bool  True if we have successfully solved for gyro bias
   */
  bool solve_bg(
      const std::map<double, std::shared_ptr<ov_core::CpiV1>>
          &map_camera_cpi_I0toIi,
      const std::unordered_map<
          size_t, std::unordered_map<size_t, std::unordered_map<double, Vec2>>>
          &feat_norms,
      Vec3 &bg);

  /**
   * @brief Get the Jacobians w.r.t. to gyro bias using a LM optimization
   * Reference:
   * https://openaccess.thecvf.com/content/CVPR2023/papers/He_A_Rotation-Translation-Decoupled_Solution_for_Robust_and_Efficient_Visual-Inertial_Initialization_CVPR_2023_paper.pdf
   * https://openaccess.thecvf.com/content_iccv_2013/papers/Kneip_Direct_Optimization_of_2013_ICCV_paper.pdf
   * @param map_camera_cpi_I0toIi All camera CPI preintegrations
   * @param data_all All bg solver data
   * @param delta_bg Small change in gyro bias
   * @param jacobian Output Jacobian of residual w.r.t. gyro bias
   * @param res Output residual
   */
  void get_bg_jacobians(const std::map<double, std::shared_ptr<ov_core::CpiV1>>
                            &map_camera_cpi_I0toIi,
                        const BgSolverData &data_all, const Vec3 &delta_bg,
                        Eigen::Matrix<DataType, 1, 3> &jacobian, DataType &res);

  /**
   * @brief Prepare all bg solver data
   * @param feat_norms All feature normalized measurements, it would be better
   * to refactor this type. 3 layers of map is too much.
   * Layers: [feat_id][cam_id][timestamp][uv]
   * @param kf_timestamps All keyframe timestamps
   * @param bg_solver_data Output map of bg solver data (id to data)
   */
  void prepare_bg_solver_data(
      const std::unordered_map<
          size_t, std::unordered_map<size_t, std::unordered_map<double, Vec2>>>
          &feat_norms,
      const std::vector<double> &kf_timestamps,
      std::map<size_t, BgSolverData> &bg_solver_data);

  /// Initialization parameters
  InertialInitializerOptions params_;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> db_;

  // Propagator shared with VIO
  std::shared_ptr<ov_srvins::Propagator> propagator_;

  // MSCKF feature updater shared with VIO
  std::shared_ptr<ov_srvins::UpdaterMSCKF> updater_msckf_;

  // SLAM feature updater shared with VIO
  std::shared_ptr<ov_srvins::UpdaterSLAM> updater_slam_;

  // Store gravity vector
  Vec3 gravity_;
};

} // namespace ov_srvins

#endif // OV_SRVINS_DYNAMICINITIALIZER_H
