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





#ifndef OV_SRVINS_STATE_H
#define OV_SRVINS_STATE_H

#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include "StateOptions.h"
#include "cam/CamBase.h"
#include "initializer/InertialInitializerOptions.h"
#include "types/IMU.h"
#include "types/Landmark.h"
#include "types/LandmarkMsckf.h"
#include "types/PoseJPL.h"
#include "types/Type.h"
#include "types/Vec.h"
#include "utils/CameraPoseBuffer.h"
#include "utils/DataType.h"
#include "utils/EigenMatrixBuffer.h"
#include "utils/Timer.h"

namespace ov_srvins {

/**
 * @brief State of our filter
 *
 * This state has all the current estimates for the filter.
 * This system is modeled after the MSCKF filter, thus we have a sliding window
 * of clones. We additionally have more parameters for online estimation of
 * calibration and SLAM features. We also have the covariance of the system,
 * which should be managed using the StateHelper class.
 */
class State {

public:
  /**
   * @brief Default Constructor (will initialize variables to defaults)
   * @param options_ Options structure containing filter options
   */
  State(StateOptions &options_,
        ov_srvins::InertialInitializerOptions &init_options_);

  ~State() = default;

  /**
   * @brief Will return the timestep that we will marginalize next.
   * As of right now, since we are using a sliding window, this is the oldest
   * clone. But if you wanted to do a keyframe system, you could selectively
   * marginalize clones.
   * @return timestep of clone we will marginalize
   */
  double margtimestep();

  /**
   * @brief Clean up the stored factors and jacobians
   * @param clean_msckf_state If true, will also clear the MSCKF features, used
   * in iterative update
   */
  void clear(bool clean_msckf_state = false);

  /**
   * @brief Get the squared norm of the state vector
   */
  double get_x_squared_norm();

  /**
   * @brief Get the squared norm of the delta state vector
   */
  double get_dx_squared_norm();

  /**
   * @brief Get the squared norm of the residual vector
   */
  double get_residual_squared_norm();

  /**
   * @brief Get the clone pose at a specific timestamp
   *
   * @param   double  query_timestamp  The timestamp to query
   *
   * @return  std::shared_ptr<ov_type::PoseJPL>  The clone pose at the queried
   * timestamp
   */
  std::shared_ptr<ov_type::PoseJPL> get_clone_pose(double query_timestamp);

  /**
   * @brief Resize the covariance matrix to be square
   */
  void resize_U_to_square();

  /**
   * @brief Calculate the clone poses to avoid recalculation
   * @param fej If true, will calculate the fej clone poses
   */
  void calculate_clone_poses(bool fej = false);

  /**
   * @brief Calculate the FEJ clone poses to avoid recalculation
   */
  void calculate_clone_poses_fej();

  /**
   * @brief Erase a MSCKF feature from the state
   * @param feat_id ID of the feature to be removed
   */
  void erase_feat(size_t feat_id);

  /**
   * @brief Store the initialization factor for a new variable
   *
   * @param new_var New variable being initialized
   * @param tri_factor Triangular block during initialization
   * @param dense_factor Dense block during initialization
   */
  void store_init_factor(const std::shared_ptr<ov_type::Type> &new_var,
                         const MatX &tri_factor, const MatX &dense_factor);

  /**
   * @brief Store the update factor for a new measurement
   * @param R_sinv_H_UT R^{-1/2} * H * U^T
   * @param R_inv_res R^{-1} * residual
   */
  void store_update_factor(const MatX &R_sinv_H_UT, const VecX &R_inv_res);

  /**
   * @brief Store the update jacobians for a new measurement
   * @param Hx Jacobian of measurement w.r.t. state
   * @param res Residual of measurement
   * @param x_order Order of state variables in Hx
   */
  void store_update_jacobians(
      const MatX &Hx, const VecX &res,
      const std::vector<std::shared_ptr<ov_type::Type>> &x_order);

  /**
   * @brief Calculates the current max size of the covariance
   * @return Size of the current covariance matrix
   */
  int get_state_size();

  /**
   * @brief Setup the matrix buffers to the correct size, need to call before
   * each update
   */
  void setup_matrix_buffer();

  /**
   * @brief Update the current timestamp of the state and also add to pose
   * buffers
   * @param new_time New timestamp to set
   */
  void update_timestamp(double new_time);

  /**
   * @brief Remove timestamp from pose buffers
   * @param old_time Timestamp to remove
   */
  void remove_timestamp(double old_time);

  /**
   * @brief Get the x_{k}-x_{k-1}, which is the latest delta state
   * @return Eigen::Ref<const VecX> Reference to x_{k}-x_{k-1}
   */
  Eigen::Ref<const VecX> get_xk_minus_xk1();

  /**
   * @brief Add a variable to the list of variables to be margainalized
   * @param var Variable to be margainalized
   */
  void add_marginal_state(std::shared_ptr<ov_type::Type> var);

  /// Current timestamp (should be the last update time!)
  double timestamp = -1;

  /// Flag to tell us if we have been initialized
  bool is_initialized = false;

  /// Struct containing filter options
  StateOptions options;

  /// Struct containing initialization options
  ov_srvins::InertialInitializerOptions init_options;

  /// Pointer to the "active" IMU state (q_GtoI, p_IinG, v_IinG, bg, ba)
  std::shared_ptr<ov_type::IMU> imu;

  /// Map between imaging times and clone poses (q_GtoIi, p_IiinG)
  std::map<double, std::shared_ptr<ov_type::PoseJPL>> clones_IMU;

  /// Our current set of SLAM features (3d positions)
  std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> features_SLAM;

  std::unordered_map<size_t, std::shared_ptr<ov_type::LandmarkMsckf>>
      features_MSCKF;

  /// Time offset base IMU to camera (t_imu = t_cam + t_off)
  std::shared_ptr<ov_type::Vec> calib_dt_CAMtoIMU;

  /// Calibration poses for each camera (R_ItoC, p_IinC)
  std::unordered_map<size_t, std::shared_ptr<ov_type::PoseJPL>> calib_IMUtoCAM;

  /// Camera intrinsics
  std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>> cam_intrinsics;

  /// Camera intrinsics camera objects
  std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>>
      cam_intrinsics_cameras;

  /// Timer used to record matrix factorization time
  boost::posix_time::ptime rT81, rT82;

  /// Buffer for camera poses (non-fej and fej) to avoid recalculation
  CameraPoseBuffer cam_pose_buffer;
  CameraPoseBuffer cam_pose_fej_buffer;

private:
  // Define that the state helper is a friend class of this class
  // This will allow it to access the below functions which should normally not
  // be called This prevents a developer from thinking that the "insert clone"
  // will actually correctly add it to the covariance
  friend class StateHelper;

  /// Starting ID for clones
  const int kCloneStartId;

  /// Vector of variables
  std::vector<std::shared_ptr<ov_type::Type>> variables_;

  /// Sqaure root of the Covariance
  MatX U_;

  /// H_order update
  EigenMatrixBuffer H_update_;
  EigenMatrixBuffer res_update_;

  // Store the pre-calculated values
  EigenMatrixBuffer R_sqrt_inv_H_UT_;
  EigenMatrixBuffer HT_R_inv_res_;

  // Store the initialization values
  std::vector<std::shared_ptr<ov_type::Type>> x_init_;
  EigenMatrixBuffer factor_init_tri_;
  EigenMatrixBuffer factor_init_dense_;

  // Store x_{k}-x_{k-1} and x_{k}-x_{0}
  VecX xk_minus_xk1_;
  VecX xk_minus_x0_;

  /// Vector of variables to be margainalized
  std::vector<std::shared_ptr<ov_type::Type>> state_to_marg_;
};

} // namespace ov_srvins

#endif // OV_SRVINS_STATE_H