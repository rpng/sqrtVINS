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





#ifndef OV_SRVINS_INERTIALINITIALIZER_H
#define OV_SRVINS_INERTIALINITIALIZER_H

#include "InertialInitializerOptions.h"
#include "feat/FeatureDatabase.h"
#include "initializer/dynamic/DynamicInitializer.h"
#include "initializer/static/StaticInitializer.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "utils/sensor_data.h"

namespace ov_srvins {

/**
 * @brief Initializer for visual-inertial system.
 *
 * This will try to do both dynamic and state initialization of the state.
 * The user can request to wait for a jump in our IMU readings (i.e. device is
 * picked up) or to initialize as soon as possible.
 */
class InertialInitializer {

public:
  /**
   * @brief Default constructor
   * @param params_ Parameters loaded from either ROS or CMDLINE
   * @param db Feature tracker database with all features in it
   * @param propagator Propagator shared with VIO
   * @param updater_msckf MSCKF feature updater shared with VIO
   * @param updater_slam SLAM feature updater shared with VIO
   */
  explicit InertialInitializer(
      const InertialInitializerOptions &params,
      std::shared_ptr<ov_core::FeatureDatabase> db,
      std::shared_ptr<ov_srvins::Propagator> propagator,
      std::shared_ptr<ov_srvins::UpdaterMSCKF> updater_msckf,
      std::shared_ptr<ov_srvins::UpdaterSLAM> updater_slam);

  /**
   * @brief Try to get the initialized system
   * @param state VIO state to be initialized
   * @param wait_for_jerk If true we will wait for a "jerk"
   * @return True if we have successfully initialized our system
   */

  bool initialize(std::shared_ptr<ov_srvins::State> &state, bool wait_for_jerk);

protected:
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

  /// Our history of IMU messages (time, angular, linear)
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data_;

  /// Static initialization helper class
  std::unique_ptr<StaticInitializer> init_static_;

  /// Dynamic initialization helper class
  std::unique_ptr<DynamicInitializer> init_dynamic_;

  // Record if the platform is static previously
  bool is_static_prev_ = false;

  // Record the previous static timestamp
  double prev_static_timestamp_ = -1;
};

} // namespace ov_srvins

#endif // OV_SRVINS_INERTIALINITIALIZER_H
