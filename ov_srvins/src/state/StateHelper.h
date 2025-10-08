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





#ifndef OV_SRVINS_STATE_HELPER_H
#define OV_SRVINS_STATE_HELPER_H

#include "state/State.h"
#include "types/Type.h"
#include "utils/DataType.h"
#include <Eigen/Eigen>
#include <memory>

namespace ov_srvins {

/**
 * @brief Helper which manipulates the State and its square root
 * upper-triangular covariance.
 */
class StateHelper {

public:
  /**
   * @brief This will set the initial imu square root covariance of the system.
   * @param state Pointer to state
   * @param diagonal The diagonal of the imu square root covariance
   */
  static void
  set_initial_imu_square_root_covariance(std::shared_ptr<State> state,
                                         const VecX &diagonal);

  /**
   * @brief For a given set of variables, this will this will calculate a
   * smaller covariance.
   *
   * @param state Pointer to state
   * @param small_variables Vector of variables whose marginal covariance is
   * desired
   * @return marginal covariance of the passed variables
   */
  static MatX get_marginal_covariance(
      std::shared_ptr<State> state,
      const std::vector<std::shared_ptr<ov_type::Type>> &small_variables);

  /**
   * @brief For a given set of variables, this will this will extra the
   * smaller square root covariance column blocks without the bottom zero
   * blocks.
   *
   * @param state Pointer to state
   * @param small_variables Vector of variables whose marginal covariance is
   * desired
   * @return marginal square root covariance of the passed variables
   */
  static MatX get_marginal_U(
      std::shared_ptr<State> state,
      const std::vector<std::shared_ptr<ov_type::Type>> &small_variables);

  /**
   * @brief For a given set of variables, this will this will return a
   * smaller square root covariance.
   *
   * @note This function only return the non-zero block (need to be careful
   * about the row size), and need to make sure U is upper triangular when
   * called, and the small_variable are in accending-id order
   * @param state Pointer to state
   * @param small_variables Vector of variables whose marginal covariance is
   * desired
   * @param U_dense The top dense block
   * @param U_tri The bottom upper-triangular block
   */
  static void get_marginal_U_block(
      std::shared_ptr<State> state,
      const std::vector<std::shared_ptr<ov_type::Type>> &small_variables,
      MatX &U_dense, MatX &U_tri);

  /**
   * @brief remove oldest clone from clone_IMU_, and add to state_to_marg, but
   * we have not removed them from variables or remove their covariance
   *
   * @param state Pointer to state
   */
  static void marginalize_old_clone(std::shared_ptr<State> state);

  /**
   * @brief Marginalize bad SLAM features, add to state_to_marg, but we have not
   * removed them from variables or remove their covariance
   * @param state Pointer to state
   */
  static void marginalize_slam(std::shared_ptr<State> state);

  /**
   * @brief Marginalize the state in state_to_marg by removing their variable in
   * variables and removing their corresponding covariance
   * @param state Pointer to state
   */
  static void marginalize(std::shared_ptr<State> state);

  /**
   * @brief Perform SRF propagation on the IMU state given state transition
   * matrix and the sqaure root of noise matrix
   *
   * @param state Pointer to state
   * @param Phi state transition matrix
   * @param Q_sqrt sqaure root of noise matrix
   */
  static void propagate(std::shared_ptr<State> state,
                        const Eigen::Matrix<DataType, 15, 15> &Phi,
                        const Eigen::Matrix<DataType, 15, 15> &Q_sqrt);

  /**
   * @brief Perform SRF clone on the IMU state to create a clone IMU pose
   * @param state Pointer to state
   * @param variable_to_clone The cloned pose
   */
  static std::shared_ptr<ov_type::Type>
  clone(std::shared_ptr<State> state,
        std::shared_ptr<ov_type::Type> variable_to_clone);

  /**
   * @brief Perform the SREKF update step using LLT
   * @param state Pointer to state
   * @param is_iterative If true, this is part of an iterative update
   */
  static void update_llt(std::shared_ptr<State> state,
                         bool is_iterative = false);

  /**
   * @brief Perform the SRIEKF update step using LLT，used in dynamic
   * initialization
   * @param state Pointer to state
   */
  static void iterative_update_llt(std::shared_ptr<State> state);

  /**
   * @brief Perform the SREKF slam feature initialization step, note in here, we
   * only calculate the factors, but not yet update the covariance
   * @param state Pointer to state
   * @param new_variable The variable to be initialized
   * @param H_order The order of the jacobian rows
   * @param H_R The right jacobian block
   * @param H_L The left jacobian block
   * @param res The residual
   * @param chi_2_mult The chi 2 multiplier for the residual
   * @param sigma_pix_inv The inverse of the pixel noise standard deviation
   */
  static bool
  initialize(std::shared_ptr<State> state,
             std::shared_ptr<ov_type::Type> new_variable,
             const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
             Eigen::Ref<MatX> H_R, Eigen::Ref<MatX> H_L, Eigen::Ref<VecX> res,
             DataType chi_2_mult, DataType sigma_pix_inv);

  /**
   * @brief Perform the SREKF slam feature initialization step, note in here, we
   * only calculate the factors, but not yet update the covariance
   * @param state Pointer to state
   * @param new_variable The variable to be initialized
   * @param H_order The order of the jacobian rows
   * @param H_R The right jacobian block
   * @param H_L The left jacobian block
   * @param res The residual
   * @param chi_2_mult The chi 2 multiplier for the residual
   * @param sigma_pix_inv The inverse of the pixel noise standard deviation
   */
  static void initialize_invertible(
      std::shared_ptr<State> state, std::shared_ptr<ov_type::Type> new_variable,
      const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
      Eigen::Ref<const MatX> H_R, Eigen::Ref<const MatX> H_L,
      Eigen::Ref<const VecX> res, DataType sigma_pix_inv);

  /**
   * @brief Perform the SREKF initialization step, in here, we will initialize
   * the SLAM features into the state, and append its corresponding covariance
   * @param state Pointer to state
   */
  static void initialize_slam_in_U(std::shared_ptr<State> state);

  /**
   * @brief Initialize the state with initial imu state
   * @param state Pointer to state
   * @param imu_init Initial imu state, 15 DoF vector
   * @param timestamp Initial timestamp
   */
  static void initialize_state(std::shared_ptr<State> state,
                               Eigen::Ref<const VecX> imu_init,
                               double timestamp);

  /**
   * @brief Propagate the time offset of the IMU state
   * @param state Pointer to state
   * @param dnc_dt The time offset related Jacobian (6x1 vector)
   */
  static void
  propagate_timeoffset(std::shared_ptr<State> state,
                       Eigen::Ref<Eigen::Matrix<DataType, 6, 1>> dnc_dt);

  /**
   * @brief Propagate the state with zero motion (for zero velocity update)
   * @param state Pointer to state
   * @param dt_summed The total time duration of the zero motion
   * @param sigma_wb The gyro bias standard deviation
   * @param sigma_ab The accelerometer bias standard deviation
   */
  static void propagate_zero_motion(std::shared_ptr<State> state,
                                    double dt_summed, DataType sigma_wb,
                                    DataType sigma_ab);

  /**
   * @brief Propagate a SLAM feature whose anchor clone has changed
   * @param state Pointer to state
   * @param landmark The slam feature landmark to be propagated
   * @param Phi The state transition matrix from the old state (the identity
   * part for the new SLAM feature is excluded)
   * @param phi_order_OLD The order of the old state w.r.t. the  state
   * transition matrix (excluding the new SLAM feature)
   */
  static void propagate_slam_anchor_feature(
      std::shared_ptr<State> state, std::shared_ptr<ov_type::Landmark> landmark,
      Eigen::Ref<const MatX> Phi,
      const std::vector<std::shared_ptr<ov_type::Type>> &phi_order_OLD);

  /**
   * @brief Calculate the factors for a SLAM feature, because SLAM feature has
   * special U structure, we do it by block to save time
   * @param state Pointer to state
   * @param H_order The order of the jacobian
   * @param H The full jacobian matrix
   * @param HUT The output H*U^T matrix
   */
  static void get_factors_for_slam_feature(
      std::shared_ptr<State> state,
      const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
      Eigen::Ref<const MatX> H, Eigen::Ref<MatX> HUT);

private:
  /**
   * All function in this class should be static.
   * Thus an instance of this class cannot be created.
   */
  StateHelper() {}
};

} // namespace ov_srvins

#endif // OV_SRVINS_STATE_HELPER_H
