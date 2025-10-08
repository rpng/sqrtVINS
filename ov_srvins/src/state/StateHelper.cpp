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





#include "StateHelper.h"

#include "state/State.h"
#include "utils/Helper.h"

#include "types/Landmark.h"
#include "utils/colors.h"
#include "utils/print.h"

#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_srvins;

void StateHelper::set_initial_imu_square_root_covariance(
    std::shared_ptr<State> state, const VecX &diagonal) {
  assert(state->imu->id() == 0);
  state->U_.topLeftCorner(15, 15).diagonal() = diagonal;
}

MatX StateHelper::get_marginal_covariance(
    std::shared_ptr<State> state,
    const std::vector<std::shared_ptr<Type>> &small_variables) {
  MatX Small_U = get_marginal_U(state, small_variables);
  int state_size = 0;
  for (const auto &var : small_variables) {
    state_size += var->size();
  }
  // Construct our return covariance
  MatX Small_cov = MatX::Zero(state_size, state_size);
  matrix_multiplier_ATA(Small_U, Small_cov);
  return Small_cov;
}

MatX StateHelper::get_marginal_U(
    std::shared_ptr<State> state,
    const std::vector<std::shared_ptr<Type>> &small_variables) {

  // Calculate the marginal covariance size we need to make our matrix
  int U_size = 0;
  int max_row_size = -1;

  for (size_t i = 0; i < small_variables.size(); i++) {
    U_size += small_variables[i]->size();
    max_row_size = std::max(max_row_size, small_variables[i]->id() +
                                              small_variables[i]->size());
  }

  // Construct our return covariance
  MatX Small_U = MatX::Zero(max_row_size, U_size);

  int current_id = 0;
  for (auto var : small_variables) {
    Small_U.block(0, current_id, max_row_size, var->size()) =
        state->U_.block(0, var->id(), max_row_size, var->size());
    current_id += var->size();
  }

  // Return the Square root covariance
  return Small_U;
}

void StateHelper::get_marginal_U_block(
    std::shared_ptr<State> state,
    const std::vector<std::shared_ptr<Type>> &small_variables, MatX &U_dense,
    MatX &U_upper_tri) {

  // Only work with square matrices
  assert(state->U_.rows() == state->U_.cols());

  for (size_t i = 0; i < small_variables.size() - 1; i++) {
    if (small_variables.at(i)->id() > small_variables.at(i + 1)->id()) {
      PRINT_ERROR("small_variables must be in id ascending order");
      std::abort();
    }
  }

  // Calculate the marginal covariance size we need to make our matrix
  int U_size = 0;
  int max_non_zero_rows = small_variables.rbegin()->get()->id() +
                          small_variables.rbegin()->get()->size();

  for (size_t i = 0; i < small_variables.size(); i++) {
    U_size += small_variables[i]->size();
  }

  // Construct our return covariance
  U_dense.resizeLike(MatX::Zero(max_non_zero_rows - U_size, U_size));
  U_upper_tri.resizeLike(MatX::Zero(U_size, U_size));

  int current_id = 0;
  for (auto var : small_variables) {
    U_dense.block(0, current_id, max_non_zero_rows - U_size, var->size()) =
        state->U_.block(0, var->id(), max_non_zero_rows - U_size, var->size());
    U_upper_tri.block(0, current_id, U_size, var->size()) = state->U_.block(
        max_non_zero_rows - U_size, var->id(), U_size, var->size());
    current_id += var->size();
  }
}

void StateHelper::marginalize_old_clone(std::shared_ptr<State> state) {
  if ((int)state->clones_IMU.size() > state->options.max_clone_size + 1) {
    double marginal_time = state->margtimestep();
    state->state_to_marg_.push_back(state->clones_IMU.at(marginal_time));
    state->remove_timestamp(marginal_time);
    state->clones_IMU.erase(marginal_time);
  }
}

void StateHelper::marginalize_slam(std::shared_ptr<State> state) {
  // Remove SLAM features that have their marginalization flag set
  // We also check that we do not remove any aruoctag landmarks
  int ct_marginalized = 0;
  auto it0 = state->features_SLAM.begin();
  while (it0 != state->features_SLAM.end()) {
    if ((*it0).second->should_marg &&
        (int)(*it0).first > 4 * state->options.max_aruco_features) {
      state->state_to_marg_.push_back((*it0).second);
      it0 = state->features_SLAM.erase(it0);
      ct_marginalized++;
    } else {
      it0++;
    }
  }
}

void StateHelper::marginalize(std::shared_ptr<State> state) {

  int marg_size = 0;
  for (auto marg : state->state_to_marg_) {
    marg_size += marg->size();
  }

  // sort the variables first by id in ascending order
  sort(state->variables_.begin(), state->variables_.end(),
       [](const std::shared_ptr<ov_type::Type> &a,
          const std::shared_ptr<ov_type::Type> &b) {
         return a->id() < b->id();
       });

  int rows = state->U_.rows();
  int old_state_size = state->U_.cols();
  int new_state_size = old_state_size - marg_size;
  int curr_id = 0;

  for (auto var = state->variables_.begin(); var != state->variables_.end();) {
    // Erase this is the one to be marginalized
    if (std::find(state->state_to_marg_.begin(), state->state_to_marg_.end(),
                  *var) != state->state_to_marg_.end()) {
      var = state->variables_.erase(var);
      continue;
    }

    // Copy SR covariance block
    state->U_.block(0, curr_id, rows, (*var)->size()) =
        state->U_.block(0, (*var)->id(), rows, (*var)->size());

    (*var)->set_local_id(curr_id);
    curr_id += (*var)->size();
    var++;
  }
  efficient_QR(state->U_);
  state->U_.conservativeResizeLike(MatX::Zero(new_state_size, new_state_size));
  state->U_ = state->U_.triangularView<Eigen::Upper>();
  state->xk_minus_x0_.conservativeResizeLike(VecX::Zero(new_state_size, 1));
  state->xk_minus_xk1_.conservativeResizeLike(VecX::Zero(new_state_size, 1));
  state->state_to_marg_.clear();
}

void StateHelper::propagate(std::shared_ptr<State> state,
                            const Eigen::Matrix<DataType, 15, 15> &Phi,
                            const Eigen::Matrix<DataType, 15, 15> &Q_sqrt) {
  int state_size = state->U_.cols();
  int rows = state->U_.rows();
  int remaining_id = state->imu->id() + state->imu->size();
  MatX U_new = MatX::Zero(rows + 15, state_size);

  // Copy first block if needed
  if (state->imu->id() != 0)
    U_new.block(15, 0, state->imu->id(), state->imu->id())
        .triangularView<Eigen::Upper>() =
        state->U_.block(0, 0, state->imu->id(), state->imu->id());

  // New IMU
  U_new.block(15, state->imu->id(), remaining_id, 15).noalias() =
      state->U_.block(0, state->imu->id(), remaining_id, 15) * Phi.transpose();

  // Remaining
  U_new.bottomRightCorner(rows, state_size - remaining_id) =
      state->U_.bottomRightCorner(rows, state_size - remaining_id);

  // Noise Block
  U_new.block<15, 15>(0, state->imu->id()) = Q_sqrt;

  // Do first QR in here if not reaching the maximal clone size
  if ((int)state->clones_IMU.size() < state->options.max_clone_size + 1) {
    efficient_QR(U_new);
    U_new.conservativeResizeLike(MatX::Zero(state_size, state_size));
    U_new = U_new.triangularView<Eigen::Upper>();
  }

  state->U_ = U_new;
}

std::shared_ptr<Type>
StateHelper::clone(std::shared_ptr<State> state,
                   std::shared_ptr<Type> variable_to_clone) {
  assert(state->U_.rows() == state->U_.cols());

  // Get total size of new cloned variables, and the old covariance size
  int new_size = variable_to_clone->size();
  int state_size = (int)state->U_.cols();
  int rows = (int)state->U_.rows();

  int new_loc = state->kCloneStartId;

  // Resize both our covariance to the new size
  state->U_.conservativeResizeLike(MatX::Zero(rows, state_size + new_size));

  // Copy the remaining blocks
  state->U_.block(0, new_loc + new_size, rows, state_size - new_loc) =
      state->U_.block(0, new_loc, rows, state_size - new_loc).eval();

  // Copy the clone blocks
  state->U_.block(0, new_loc, rows, new_size) =
      state->U_.block(0, state->imu->id(), rows, new_size);

  // Set local id for the new variable
  variable_to_clone->set_local_id(new_loc);

  // Set local id for the remaining variables
  for (size_t k = 0; k < state->variables_.size(); k++) {
    if (state->variables_.at(k)->id() >= new_loc) {
      state->variables_.at(k)->set_local_id(state->variables_.at(k)->id() +
                                            new_size);
    }
  }

  // Add to variable list and return
  state->variables_.push_back(variable_to_clone);

  return variable_to_clone;
}

void StateHelper::update_llt(std::shared_ptr<State> state, bool is_iterative) {
  // Skip if no update is required
  if (state->R_sqrt_inv_H_UT_.rows() == 0)
    return;

  // Calculate measurement size
  int state_size = state->U_.cols();
  int offset = state->x_init_.size() * 3; // offset value to reduce computation

  //==========================================================
  //==========================================================
  // Get the location in small jacobian for each measuring variable
  VecX HT_R_inv_res = VecX::Zero(state_size, 1);
  assert(state_size - offset == state->HT_R_inv_res_.get().cols());
  HT_R_inv_res.topRows(state_size - offset) = state->HT_R_inv_res_.get();
  auto R_sqrt_inv_H_UT = state->R_sqrt_inv_H_UT_.get();

  // Reverse column first before LLT
  reverse_mat(R_sqrt_inv_H_UT);

  // Batch rank update with sparsity
  MatX FT_F = MatX::Zero(state_size - offset, state_size - offset);
  matrix_multiplier_ATA(R_sqrt_inv_H_UT, FT_F);
  FT_F.diagonal() += VecX::Ones(state_size - offset);
  Eigen::LLT<MatX> llt(FT_F.selfadjointView<Eigen::Upper>());
  MatX F = llt.matrixU();

  // Reverse column back
  reverse_mat(F);

  // Reverse rows to have upper triangular structure
  reverse_mat(F, false);

  state->rT81 = boost::posix_time::microsec_clock::local_time();

  // Get H^T * r + H^T * H * dx for iterative update
  if (is_iterative) {
    // Get H^T*H*dx
    HT_R_inv_res.topRows(state_size - offset) +=
        state->H_update_.get().transpose() *
        (state->H_update_.get() * state->xk_minus_x0_);
  }

  VecX dx_xkp1_minus_x0;
  F.transpose().triangularView<Eigen::Upper>().solveInPlace(
      state->U_.topRightCorner(state_size - offset, offset));
  triangular_matrix_inverse_solver(
      F.transpose(),
      state->U_.topLeftCorner(state_size - offset, state_size - offset));
  // state->_U.topLeftCorner(state_size - offset, state_size - offset) = U_temp;
  state->U_ = state->U_.triangularView<Eigen::Upper>();
  dx_xkp1_minus_x0 = state->U_.transpose().triangularView<Eigen::Lower>() *
                     (state->U_.triangularView<Eigen::Upper>() * HT_R_inv_res);

  if (is_iterative) {
    // For iterative SRF, we need to first downdate then update
    for (size_t i = 0; i < state->variables_.size(); i++) {
      state->variables_.at(i)->update(
          -state->xk_minus_x0_.block(state->variables_.at(i)->id(), 0,
                                     state->variables_.at(i)->size(), 1));
    }
  }

  state->xk_minus_x0_ = dx_xkp1_minus_x0;
  for (size_t i = 0; i < state->variables_.size(); i++) {
    state->variables_.at(i)->update(state->xk_minus_x0_.block(
        state->variables_.at(i)->id(), 0, state->variables_.at(i)->size(), 1));
  }

  // If we are doing online intrinsic calibration we should update our camera
  // objects NOTE: is this the best place to put this update logic??? probably..
  if (state->options.do_calib_camera_intrinsics) {
    for (auto const &calib : state->cam_intrinsics) {
      state->cam_intrinsics_cameras.at(calib.first)
          ->set_value(calib.second->value());
    }
  }

  // clean the buffer
  state->rT82 = boost::posix_time::microsec_clock::local_time();
}

void StateHelper::iterative_update_llt(std::shared_ptr<State> state) {
  if (state->H_update_.rows() == 0)
    return;

  // Calculate measurement size
  int state_size = state->U_.cols();

  // Get H^T * r + H^T * H * dx for iterative update
  // Get H_big
  Eigen::Ref<MatX> H_big = state->H_update_.get();
  Eigen::Ref<MatX> r_big = state->res_update_.get();
  reverse_mat(H_big);

  Eigen::Ref<MatX> H_big_block =
      state->H_update_.get().leftCols(state_size - 15); // remove the imu block
  efficient_QR(H_big_block, r_big);

  // Get lower triangular structure H_big
  Eigen::Ref<MatX> H_big_new =
      state->H_update_.get_block(0, 0, state_size, state_size);
  Eigen::Ref<MatX> r_big_new =
      state->res_update_.get_block(0, 0, state_size, 1);
  reverse_mat(H_big_new, false);
  reverse_mat(H_big_new);
  reverse_vec(r_big_new);

  r_big_new += H_big_new.triangularView<Eigen::Lower>() * state->xk_minus_x0_;

  MatX UH_T = MatX::Zero(state_size, state_size);
  MatX S = MatX::Zero(state_size, state_size);
  triangular_matrix_multiplier_UU(state->U_, H_big_new.transpose(), UH_T);
  triangular_matrix_multiplier_LLT(UH_T.transpose(), S);
  S.diagonal() += VecX::Ones(state_size);
  VecX dx_xkp1_minus_x0;

  dx_xkp1_minus_x0 =
      state->U_.transpose() *
      (state->U_ * (H_big_new.transpose().triangularView<Eigen::Upper>() *
                    S.llt().solve(r_big_new)));
  // First downdate then udpate
  for (size_t i = 0; i < state->variables_.size(); i++) {
    state->variables_.at(i)->update(-state->xk_minus_x0_.block(
        state->variables_.at(i)->id(), 0, state->variables_.at(i)->size(), 1));
  }
  state->xk_minus_x0_ = dx_xkp1_minus_x0;
  for (size_t i = 0; i < state->variables_.size(); i++) {
    state->variables_.at(i)->update(state->xk_minus_x0_.block(
        state->variables_.at(i)->id(), 0, state->variables_.at(i)->size(), 1));
  }

  // If we are doing online intrinsic calibration we should update our camera
  // objects NOTE: is this the best place to put this update logic??? probably..
  if (state->options.do_calib_camera_intrinsics) {
    for (auto const &calib : state->cam_intrinsics) {
      state->cam_intrinsics_cameras.at(calib.first)
          ->set_value(calib.second->value());
    }
  }
}

bool StateHelper::initialize(std::shared_ptr<State> state,
                             std::shared_ptr<Type> new_variable,
                             const std::vector<std::shared_ptr<Type>> &H_order,
                             Eigen::Ref<MatX> H_R, Eigen::Ref<MatX> H_L,
                             Eigen::Ref<VecX> res, DataType chi_2_mult,
                             DataType sigma_pix_inv) {
  // Check that this new variable is not already initialized
  if (std::find(state->variables_.begin(), state->variables_.end(),
                new_variable) != state->variables_.end()) {
    PRINT_ERROR("StateHelper::initialize_invertible() - Called on variable "
                "that is already in the state\n");
    PRINT_ERROR("StateHelper::initialize_invertible() - Found this variable at "
                "%d in covariance\n",
                new_variable->id());
    std::exit(EXIT_FAILURE);
  }

  //==========================================================
  //==========================================================
  // First we perform QR givens to seperate the system
  // The top will be a system that depends on the new state, while the bottom
  // does not
  const int kNewVarSize = new_variable->size();
  const int kUpdateStateSize = H_R.cols();
  const int kUpdateMeasSize = H_R.rows() - kNewVarSize;
  assert((int)kNewVarSize == H_L.cols());
  // to make H_L has lower triangular structure, so that we don't need to do QR
  // when we initialize it into our state
  reverse_mat(H_L);

  // do column permutation for efficient nullspace projection
  reverse_mat(H_R);
  efficient_QR(H_R, res, H_L);
  reverse_mat(H_R);

  // Separate into initializing and updating portions
  // 1. Invertible initializing system
  Eigen::Ref<MatX> Hx_init = H_R.block(0, 0, kNewVarSize, kUpdateStateSize);
  Eigen::Ref<MatX> Hf_init = H_L.block(0, 0, kNewVarSize, kNewVarSize);
  Eigen::Ref<VecX> res_init = res.block(0, 0, kNewVarSize, 1);

  // Reverse column of Hf back
  reverse_mat(Hf_init);
  // Reverse rows for the init blocks to make H_fint lower triangular
  reverse_mat(Hx_init, false);
  reverse_mat(Hf_init, false);
  reverse_vec(res_init);

  // 2. Nullspace projected updating system
  Eigen::Ref<MatX> H_update =
      H_R.block(kNewVarSize, 0, kUpdateMeasSize, kUpdateStateSize);
  VecX res_update = res.bottomRows(res.rows() - kNewVarSize);
  //==========================================================
  //==========================================================

  // Do mahalanobis distance testing
  MatX U_dense, U_tri;
  StateHelper::get_marginal_U_block(state, H_order, U_dense, U_tri);
  MatX HUT = MatX::Zero(kUpdateMeasSize, U_dense.rows() + U_tri.rows());

  HUT.leftCols(U_dense.rows()).noalias() = H_update * U_dense.transpose();
  HUT.rightCols(U_tri.rows()).noalias() =
      H_update * U_tri.transpose().triangularView<Eigen::Lower>();

  // Cholesky here is more efficient than QR
  if (chi_2_mult != -1) {
    MatX S = HUT * HUT.transpose();
    S.diagonal() +=
        1.0 / (sigma_pix_inv * sigma_pix_inv) * VecX::Ones(S.rows());
    DataType chi2 = res_update.dot(
        S.selfadjointView<Eigen::Upper>().llt().solve(res_update));
    // Get what our threshold should be
    boost::math::chi_squared chi_squared_dist(res.rows());
    DataType chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    if (chi2 > chi_2_mult * chi2_check) {
      return false;
    }
  }

  //==========================================================
  //==========================================================
  // Finally, initialize it in our state
  StateHelper::initialize_invertible(state, new_variable, H_order, Hx_init,
                                     Hf_init, res_init, sigma_pix_inv);

  // Update with updating portion
  if (H_update.rows() > 0) {
    VecX RHTr = VecX::Zero(state->get_state_size());
    int local_id = 0;
    for (const auto &var : H_order) {
      RHTr.block(var->id(), 0, var->size(), 1).noalias() +=
          H_update.block(0, local_id, H_update.rows(), var->size())
              .transpose() *
          res_update * sigma_pix_inv * sigma_pix_inv;
      local_id += var->size();
    }
    state->store_update_factor(HUT * sigma_pix_inv, RHTr);
    if (!state->is_initialized) {
      state->store_update_jacobians(H_update * sigma_pix_inv,
                                    res_update * sigma_pix_inv, H_order);
    }
  }

  return true;
}

void StateHelper::initialize_invertible(
    std::shared_ptr<State> state, std::shared_ptr<Type> new_variable,
    const std::vector<std::shared_ptr<Type>> &H_order,
    Eigen::Ref<const MatX> H_R, Eigen::Ref<const MatX> H_L,
    Eigen::Ref<const VecX> res, DataType sigma_pix_inv) {

  // Check that this new variable is not already initialized
  if (std::find(state->variables_.begin(), state->variables_.end(),
                new_variable) != state->variables_.end()) {
    PRINT_ERROR("StateHelper::initialize_invertible() - Called on variable "
                "that is already in the state\n");
    PRINT_ERROR("StateHelper::initialize_invertible() - Found this variable at "
                "%d in covariance\n",
                new_variable->id());
    std::exit(EXIT_FAILURE);
  }

  //==========================================================
  //==========================================================
  // Part of the Kalman Gain K = (P*H^T)*S^{-1} = M*S^{-1}
  assert(H_L.rows() == res.rows());
  assert(H_L.rows() == H_R.rows());
  const int kStateSize = state->U_.cols();
  const int kMeasSize = res.rows();
  MatX U_HRT = MatX::Zero(kStateSize, kMeasSize);

  // Get the location in small jacobian for each measuring variable
  int current_it = 0;
  std::vector<int> H_id;
  for (const auto &meas_var : H_order) {
    H_id.push_back(current_it);
    current_it += meas_var->size();
  }

  //==========================================================
  //==========================================================
  // For each active variable find its M = U*H^T
  for (const auto &var : state->variables_) {
    MatX M_i = MatX::Zero(var->size(), kMeasSize);
    for (size_t i = 0; i < H_order.size(); i++) {
      std::shared_ptr<Type> meas_var = H_order.at(i);
      M_i += state->U_.block(var->id(), meas_var->id(), var->size(),
                             meas_var->size()) *
             H_R.block(0, H_id[i], H_R.rows(), meas_var->size()).transpose();
    }
    U_HRT.middleRows(var->id(), var->size()) = M_i;
  }

  //==========================================================
  //==========================================================
  MatX H_Linv = Mat3::Identity();
  H_L.triangularView<Eigen::Lower>().solveInPlace(H_Linv);

  // Update the variable that will be initialized (invertible systems can only
  // update the new variable). However this update should be almost zero if we
  // already used a conditional Gauss-Newton to solve for the initial estimate
  VecX dx = VecX::Zero(H_R.cols());
  if (!state->is_initialized) {
    current_it = 0;
    for (const auto &var : H_order) {
      dx.middleRows(current_it, var->size()) =
          state->xk_minus_x0_.middleRows(var->id(), var->size());
      current_it += var->size();
    }
  }

  new_variable->update(H_Linv * (res + H_R * dx));
  state->store_init_factor(new_variable,
                           (1 / sigma_pix_inv) * H_Linv.transpose(),
                           -U_HRT * H_Linv.transpose());
}

void StateHelper::initialize_slam_in_U(std::shared_ptr<State> state) {
  if (state->x_init_.empty())
    return;

  // Calculate increase of state size
  int new_state_size = 0;
  for (auto var : state->x_init_) {
    new_state_size += var->size();
  }

  // Insert at the end
  int old_size = state->U_.cols();
  int curr_id = old_size;

  MatX U = MatX::Zero(old_size + new_state_size, old_size + new_state_size);
  U.topLeftCorner(curr_id, curr_id) = state->U_.topLeftCorner(curr_id, curr_id);
  if (curr_id != old_size) {
    U.topRightCorner(curr_id, old_size - curr_id) =
        state->U_.topRightCorner(curr_id, old_size - curr_id);
    U.bottomRightCorner(old_size - curr_id, old_size - curr_id) =
        state->U_.bottomRightCorner(old_size - curr_id, old_size - curr_id);
  }

  // Increase the id of all variables after the current id
  for (auto var : state->variables_) {
    if (var->id() >= curr_id) {
      var->set_local_id(var->id() + new_state_size);
    }
  }
  U.block(0, curr_id, state->factor_init_dense_.rows(),
          state->factor_init_dense_.cols()) = state->factor_init_dense_.get();
  for (size_t i = 0; i < state->x_init_.size(); i++) {
    int var_size = state->x_init_.at(i)->size();
    // Init SR covariance block
    U.block(curr_id, curr_id, var_size, var_size)
        .triangularView<Eigen::Upper>() =
        state->factor_init_tri_.get_rows(3 * i, 3);

    // Init state
    state->x_init_.at(i)->set_local_id(curr_id);
    state->variables_.push_back(state->x_init_.at(i));
    curr_id += var_size;
  }
  state->U_.swap(U);

  state->xk_minus_x0_.conservativeResizeLike(
      VecX::Zero(old_size + new_state_size));
  state->xk_minus_xk1_.conservativeResizeLike(
      VecX::Zero(old_size + new_state_size));
}

void StateHelper::initialize_state(std::shared_ptr<State> state,
                                   Eigen::Ref<const VecX> imu_init,
                                   double timestamp) {
  // Clear first
  state->variables_.clear();
  state->calib_IMUtoCAM.clear();
  state->cam_intrinsics.clear();
  state->cam_intrinsics_cameras.clear();

  // Setup timestamp
  state->update_timestamp(timestamp);

  // Append the imu to the state and covariance
  state->imu = std::make_shared<IMU>();
  state->imu->set_local_id(0);
  state->imu->set_value(imu_init);
  state->imu->set_fej(imu_init);
  state->variables_.push_back(state->imu);

  int current_id = state->imu->size();

  // Camera to IMU time offset
  state->calib_dt_CAMtoIMU = std::make_shared<Vec>(1);
  if (state->options.do_calib_camera_timeoffset) {
    state->calib_dt_CAMtoIMU->set_local_id(current_id);
    state->variables_.push_back(state->calib_dt_CAMtoIMU);
    current_id += state->calib_dt_CAMtoIMU->size();
  }

  // Loop through each camera and create extrinsic and intrinsics
  for (int i = 0; i < state->options.num_cameras; i++) {

    // Allocate extrinsic transform
    auto pose = std::make_shared<PoseJPL>();

    // Allocate intrinsics for this camera
    auto intrin = std::make_shared<Vec>(8);

    // Add these to the corresponding maps
    state->calib_IMUtoCAM.insert({i, pose});
    state->cam_intrinsics.insert({i, intrin});

    // If calibrating camera-imu pose, add to variables
    if (state->options.do_calib_camera_pose) {
      pose->set_local_id(current_id);
      state->variables_.push_back(pose);
      current_id += pose->size();
    }

    // If calibrating camera intrinsics, add to variables
    if (state->options.do_calib_camera_intrinsics) {
      intrin->set_local_id(current_id);
      state->variables_.push_back(intrin);
      current_id += intrin->size();
    }
  }

  // Finally initialize our covariance to large value
  state->U_ = MatX::Zero(current_id, current_id);
  MatX U_imu_init = MatX::Identity(15, 15);
  U_imu_init(0, 0) = state->init_options.init_prior_q;
  U_imu_init(1, 1) = state->init_options.init_prior_q;
  U_imu_init(2, 2) = 0.001; // small prior for global yaw
  U_imu_init.topLeftCorner(3, 3) *= state->imu->Rot().transpose();

  U_imu_init.block(3, 3, 3, 3) =
      state->init_options.init_prior_p * Mat3::Identity(); // p
  U_imu_init.block(6, 6, 3, 3) =
      state->init_options.init_prior_v * Mat3::Identity(); // v (dynamic)
  U_imu_init.block(9, 9, 3, 3) =
      state->init_options.init_prior_bg * Mat3::Identity(); // bg
  U_imu_init.block(12, 12, 3, 3) =
      state->init_options.init_prior_ba * Mat3::Identity(); // ba

  efficient_QR(U_imu_init);
  state->U_.block(0, 0, 15, 15) = U_imu_init.triangularView<Eigen::Upper>();

  // Finally, set some of our priors for our calibration parameters
  if (state->options.do_calib_camera_timeoffset) {
    auto calib_dt_CAMtoIMU = state->calib_dt_CAMtoIMU;
    state->U_(calib_dt_CAMtoIMU->id(), calib_dt_CAMtoIMU->id()) =
        state->init_options.init_prior_t;
  }

  if (state->options.do_calib_camera_pose) {
    auto calib_IMUtoCAM = state->calib_IMUtoCAM;
    for (int i = 0; i < state->options.num_cameras; i++) {
      state->U_.block(calib_IMUtoCAM.at(i)->id(), calib_IMUtoCAM.at(i)->id(), 3,
                      3) =
          state->init_options.init_prior_qc * MatX::Identity(3, 3);
      state->U_.block(calib_IMUtoCAM.at(i)->id() + 3,
                      calib_IMUtoCAM.at(i)->id() + 3, 3, 3) =
          state->init_options.init_prior_pc * MatX::Identity(3, 3);
    }
  }

  if (state->options.do_calib_camera_intrinsics) {
    auto cam_intrinsics = state->cam_intrinsics;
    for (int i = 0; i < state->options.num_cameras; i++) {
      state->U_.block(cam_intrinsics.at(i)->id(), cam_intrinsics.at(i)->id(), 4,
                      4) =
          state->init_options.init_prior_fc * MatX::Identity(4, 4);
      state->U_.block(cam_intrinsics.at(i)->id() + 4,
                      cam_intrinsics.at(i)->id() + 4, 2, 2) =
          state->init_options.init_prior_dc1 * MatX::Identity(2, 2);
      state->U_.block(cam_intrinsics.at(i)->id() + 6,
                      cam_intrinsics.at(i)->id() + 6, 2, 2) =
          state->init_options.init_prior_dc2 * MatX::Identity(2, 2);
    }
  }

  // Set the calibration parameters
  // Timeoffset from camera to IMU
  VecX temp_camimu_dt;
  temp_camimu_dt.resize(1);
  temp_camimu_dt(0) = state->init_options.calib_camimu_dt;
  state->calib_dt_CAMtoIMU->set_value(temp_camimu_dt);
  state->calib_dt_CAMtoIMU->set_fej(temp_camimu_dt);

  // Loop through and load each of the cameras
  for (int i = 0; i < state->init_options.num_cameras; i++) {
    state->cam_intrinsics.at(i)->set_value(
        state->init_options.camera_intrinsics.at(i)->get_value());
    state->cam_intrinsics.at(i)->set_fej(
        state->init_options.camera_intrinsics.at(i)->get_value());
    state->calib_IMUtoCAM.at(i)->set_value(
        state->init_options.camera_extrinsics.at(i));
    state->calib_IMUtoCAM.at(i)->set_fej(
        state->init_options.camera_extrinsics.at(i));

    bool is_fisheye =
        (std::dynamic_pointer_cast<ov_core::CamEqui>(
             state->init_options.camera_intrinsics.at(i)) != nullptr);
    if (is_fisheye) {
      state->cam_intrinsics_cameras[i] = std::make_shared<ov_core::CamEqui>(
          state->init_options.camera_intrinsics.at(i)->w(),
          state->init_options.camera_intrinsics.at(i)->h());
    } else {
      state->cam_intrinsics_cameras[i] = std::make_shared<ov_core::CamRadtan>(
          state->init_options.camera_intrinsics.at(i)->w(),
          state->init_options.camera_intrinsics.at(i)->h());
    }
    state->cam_intrinsics_cameras[i]->set_value(
        state->init_options.camera_intrinsics.at(i)->get_value());
  }
}

void StateHelper::propagate_timeoffset(
    std::shared_ptr<State> state,
    Eigen::Ref<Eigen::Matrix<DataType, 6, 1>> dnc_dt) {
  int kStartRow = 15;
  // start with 0 if we did qr before
  if ((int)state->clones_IMU.size() <= state->options.max_clone_size + 1) {
    kStartRow = 0;
  }

  const auto pose = state->clones_IMU.rbegin()->second;
  int dt_id = state->calib_dt_CAMtoIMU->id();
  state->U_.block(dt_id + kStartRow, pose->id(), dt_id + 1, 6) +=
      state->U_.block(dt_id + kStartRow, dt_id, dt_id + 1, 1) *
      dnc_dt.transpose();
}

void StateHelper::propagate_zero_motion(std::shared_ptr<State> state,
                                        double dt_summed, DataType sigma_wb,
                                        DataType sigma_ab) {
  MatX U_new = MatX::Zero(6 + state->U_.rows(), state->U_.cols());
  int bg_id = state->imu->bg()->id();
  int ba_id = state->imu->ba()->id();
  U_new.bottomRows(state->U_.rows()) = state->U_;
  U_new.block(0, bg_id, 3, 3) = Mat3::Identity() * sqrt(dt_summed) * sigma_wb;
  U_new.block(0, ba_id, 3, 3) = Mat3::Identity() * sqrt(dt_summed) * sigma_ab;
  efficient_QR(U_new);
  state->U_ = U_new.topLeftCorner(state->U_.cols(), state->U_.cols())
                  .triangularView<Eigen::Upper>();
}

void StateHelper::propagate_slam_anchor_feature(
    std::shared_ptr<State> state, std::shared_ptr<ov_type::Landmark> landmark,
    Eigen::Ref<const MatX> Phi,
    const std::vector<std::shared_ptr<ov_type::Type>> &phi_order_OLD) {
  const int kFeatSize = landmark->size();
  // Perform covariance propagation

  int current_it = 0;
  std::map<std::shared_ptr<Type>, int> Phi_id_map;
  for (const auto &var : phi_order_OLD) {
    Phi_id_map.insert({var, current_it});
    current_it += var->size();
  }

  MatX U_feat_new = MatX::Zero(state->U_.rows(), kFeatSize);
  const int start_row = 15;
  int offset = 0;
  const auto clone_A_new =
      state->clones_IMU.at(landmark->anchor_clone_timestamp);
  for (auto var : phi_order_OLD) {
    // Generally speaking we have 3 different types of state in here x_calib,
    // x_new_clone, x_old_clone, x_f x_new_clone needs extra care to deal with
    // no-zero Q block state id larger than the new clone need to -6 to get id
    // before clone
    if (var->id() == clone_A_new->id()) {
      U_feat_new.topRows(6).noalias() +=
          state->U_.middleCols(var->id(), var->size()).topRows(6) *
          Phi.block(0, Phi_id_map.at(var), kFeatSize, var->size()).transpose();
      if (state->imu->id() == 0 &&
          state->calib_dt_CAMtoIMU->id() == state->imu->id() + 1) {
        U_feat_new.middleRows(start_row, 16).noalias() +=
            state->U_.middleRows(start_row, 16)
                .middleCols(var->id(), var->size()) *
            Phi.middleCols(Phi_id_map.at(var), var->size()).transpose();
        continue;
      }
    }

    if (var->id() >= state->kCloneStartId)
      offset = 6;
    else
      offset = 0;
    U_feat_new.middleRows(start_row, var->size() + var->id() - offset)
        .noalias() +=
        state->U_.middleRows(start_row, var->size() + var->id() - offset)
            .middleCols(var->id(), var->size()) *
        Phi.middleCols(Phi_id_map.at(var), var->size()).transpose();
  }
  state->U_.middleCols(landmark->id(), kFeatSize) = U_feat_new;
}

void StateHelper::get_factors_for_slam_feature(
    std::shared_ptr<State> state,
    const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
    Eigen::Ref<const MatX> H, Eigen::Ref<MatX> HUT) {
  const int kMeasSize = H.rows();
  int local_id = 0;
  for (const auto &var : H_order) {
    HUT.block(0, 0, kMeasSize, var->id() + var->size()).noalias() +=
        H.block(0, local_id, kMeasSize, var->size()) *
        state->U_.block(0, var->id(), var->id() + var->size(), var->size())
            .transpose();
    local_id += var->size();
  }
}