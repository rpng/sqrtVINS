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





#include "InertialInitializer.h"

#include "initializer/dynamic/DynamicInitializer.h"
#include "initializer/static/StaticInitializer.h"

#include "feat/FeatureHelper.h"
#include "state/State.h"
#include "types/Type.h"
#include "utils/Helper.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_srvins;

InertialInitializer::InertialInitializer(
    const InertialInitializerOptions &params,
    std::shared_ptr<ov_core::FeatureDatabase> db,
    std::shared_ptr<ov_srvins::Propagator> propagator,
    std::shared_ptr<ov_srvins::UpdaterMSCKF> updater_msckf,
    std::shared_ptr<ov_srvins::UpdaterSLAM> updater_slam)
    : params_(params), db_(db), propagator_(propagator),
      updater_msckf_(updater_msckf), updater_slam_(updater_slam) {

  // Vector of our IMU data
  imu_data_ = propagator->get_imu_data();

  // Create initializers
  init_static_ = std::make_unique<StaticInitializer>(params_, db_, imu_data_);
  init_dynamic_ = std::make_unique<DynamicInitializer>(
      params_, db_, propagator, updater_msckf, updater_slam);
}

bool InertialInitializer::initialize(std::shared_ptr<ov_srvins::State> &state,
                                     bool wait_for_jerk) {

  // Get the newest and oldest timestamps we will try to initialize between!
  double lastest_cam_time = -1;
  double oldest_cam_time = std::numeric_limits<double>::max();
  for (auto const &feat : db_->get_internal_data()) {
    for (auto const &camtimepair : feat.second->timestamps) {
      for (auto const &time : camtimepair.second) {
        lastest_cam_time = std::max(lastest_cam_time, time);
        oldest_cam_time = std::min(oldest_cam_time, time);
      }
    }
  }
  if (lastest_cam_time - oldest_cam_time < params_.init_window_time) {
    PRINT_DEBUG(RED "[init]: Waiting for camera measurements to reach window "
                    "size!\n" RESET);
    return false;
  }

  double oldest_win_time =
      lastest_cam_time - params_.init_window_time - params_.init_window_offset;
  if (lastest_cam_time < 0 || oldest_win_time < 0) {
    return false;
  }

  // Remove all measurements that are older then our initialization window
  // Then we will try to use all features that are in the feature database!
  db_->cleanup_measurements(oldest_win_time);

  // Check motion with camera disparity (optical flow)
  // Note: IMU is not used in here as the platform might have vibration, and we
  // can not initialize the system either way if there is no disparity
  bool disparity_detected_moving = false;
  db_->update_disp_info();
  bool disparity_check_success =
      disparity_check(oldest_win_time, lastest_cam_time, db_,
                      params_.init_max_disparity, disparity_detected_moving);

  if (!disparity_check_success) {
    PRINT_DEBUG(RED "[init]: static check failed, no enough Camera measurement "
                    "found!\n" RESET);
    return false;
  }

  // Logic: 6 possible cases:
  // Jerk is defined as the platform is stationary previously and start moving
  // now
  //
  // !wait_for_jerk (zero velocity update enabled, no need to check for
  // jerk):
  // 1. is_still: use static initializer
  // 2. !is_still: use dynamic initializer (if it is enabled)
  //
  // wait_for_jerk (zero velocity update disabled):
  // 3. !is_still && has_jerk: use static initializer
  // 4. !is_still && !has_jerk: use dynamic initializer (if it is enabled)
  // (5). is_still && !has_jerk: dont initialize, as zero velocity update is not
  // enabled, initialize in this case might cause the system to diverge
  // (6). is_still && has_jerk: impossible case, as the system should not be
  // static and have jerk at the same time
  // Summary:
  // static initalizer: case 1, 3
  // dynamic initializer (and is enabled): case 2, 4 otherwise: do nothing

  bool is_still = !disparity_detected_moving;
  bool has_jerk = is_static_prev_ && !is_still;
  bool init_success = false;
  std::string msg = (has_jerk) ? "jerk detected, " : "no jerk detected, ";
  msg += (is_still) ? "platform is stationary" : "platform is moving";
  PRINT_INFO(YELLOW "[init]: the current system setup (zvupt: %s, dyna init: "
                    "%s) and system status: %s\n" RESET,
             !wait_for_jerk ? "true" : "false",
             params_.init_dyn_use ? "true" : "false", msg.c_str());
  if ((is_still && !wait_for_jerk) /*Case 1*/ ||
      (!is_still && has_jerk && wait_for_jerk) /*Case 3*/) {
    PRINT_DEBUG(GREEN "[init]: USING STATIC INITIALIZER METHOD!\n" RESET);
    init_success = init_static_->initialize(state, prev_static_timestamp_);
  } else if (params_.init_dyn_use &&
             ((!is_still && !wait_for_jerk) /*Case 2*/ ||
              (!is_still && !has_jerk && wait_for_jerk) /*Case 4*/)) {
    PRINT_DEBUG(GREEN "[init]: USING DYNAMIC INITIALIZER METHOD!\n" RESET);
    init_success = init_dynamic_->initialize(state);
  }

  // Update static timestamp and is_still flag
  if (is_still)
    prev_static_timestamp_ = lastest_cam_time;
  else
    prev_static_timestamp_ = -1;
  is_static_prev_ = is_still;

  return init_success;
}
