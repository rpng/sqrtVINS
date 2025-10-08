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




#pragma once
#include "cpi/CpiV1.h"
#include "feat/Feature.h"
#include "initializer/InertialInitializerOptions.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "utils/sensor_data.h"

namespace ov_srvins {

struct SolverMonitor {
  DataType res_prev = -1;
  DataType res_curr = -1;
  DataType dx_ratio = 999;
  DataType res_ratio = 999;

  void print() {
    std::cout << std::setprecision(3) << "res_prev: " << res_prev
              << ", res_curr: " << res_curr << ", dx_ratio: " << dx_ratio
              << ", res_ratio: " << res_ratio << std::endl;
  }
};

class Solver {

public:
  /**
   * @brief Default constructor for our iterative solver
   *
   * This solver will take in a state, initial guess, and a set of features
   * (both MSCKF and SLAM) and will try to optimize the state and feature
   * positions. It will use the propagator and updaters provided to do this.
   *
   * @param state Shared pointer to state to be optimized
   * @param x_init Initial guess for the state 15 DoF
   * @param features_vec Vector of features (both MSCKF and SLAM) that can be
   * used for optimization
   * @param map_camera_times Set of camera times that we have measurements at
   * @param propagator Shared pointer to propagator used for propagation
   * @param updater_msckf Shared pointer to MSCKF updater used for MSCKF update
   * @param updater_slam Shared pointer to SLAM updater used for SLAM update
   */
  explicit Solver(
      std::shared_ptr<State> state, const VecX &x_init,
      const std::vector<std::shared_ptr<ov_core::Feature>> &features_vec,
      const std::set<double> &map_camera_times,
      std::shared_ptr<ov_srvins::Propagator> propagator,
      std::shared_ptr<ov_srvins::UpdaterMSCKF> updater_msckf,
      std::shared_ptr<ov_srvins::UpdaterSLAM> updater_slam);

  /**
   * @brief Solve the optimization problem
   *
   * This function will perform the optimization process using the
   * provided state, initial guess, features, and updaters.
   *
   * @return  bool    True if the optimization was successful, false otherwise
   */
  bool solve();

private:
  /**
   * @brief Select SLAM features for initialization
   *
   * This function will select the features that will be used as SLAM features
   * to initialize in the state
   *
   * @param feat_slam_init Vector of SLAM features to be initialized
   */
  void select_slam_features(
      std::vector<std::shared_ptr<ov_core::Feature>> &feat_slam_init);

  /** @brief Check for convergence
   *
   * This function will check if the optimization has converged based on
   * the provided convergence criteria.
   *
   * @return  bool    True if the optimization has converged, false otherwise
   */
  bool convergence_check();

  // Shared state with VIO to initialize
  std::shared_ptr<State> state_;

  // Feature vector for optimization
  std::vector<std::shared_ptr<ov_core::Feature>> features_vec_;

  // Set of camera times that has measurements
  std::set<double> map_camera_times_;

  // Shared Propagator with VIO
  std::shared_ptr<ov_srvins::Propagator> propagator_;

  // Shared updater for MSCKF features with VIO
  std::shared_ptr<ov_srvins::UpdaterMSCKF> updater_msckf_;

  // Shared updater for SLAM features with VIO
  std::shared_ptr<ov_srvins::UpdaterSLAM> updater_slam_;

  // Record the statistics during optimization
  SolverMonitor solver_statistics_;
};

} // namespace ov_srvins
