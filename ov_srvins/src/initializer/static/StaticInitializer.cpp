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





#include "StaticInitializer.h"

#include "utils/Helper.h"

#include "feat/FeatureHelper.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "utils/Helper.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_srvins;

StaticInitializer::StaticInitializer(
    const InertialInitializerOptions &params,
    std::shared_ptr<ov_core::FeatureDatabase> db,
    std::shared_ptr<std::vector<ov_core::ImuData>> imu_data)
    : params_(params), db_(db), imu_data_(imu_data) {}

bool StaticInitializer::initialize(std::shared_ptr<ov_srvins::State> state,
                                   double last_static_timestamp) {
  if (!imu_data_ || imu_data_->size() < 2) {
    PRINT_INFO(YELLOW "[init-s]: not enough IMU data to initialize\n" RESET);
    return false;
  }

  // Newest and oldest imu timestamp
  double oldesttime = imu_data_->at(0).timestamp;

  // First lets collect a window of IMU readings from the oldest timestamp to
  // the last static timestamp for a static window
  std::vector<ImuData> window;
  for (const ImuData &data : *imu_data_) {
    if (data.timestamp >= oldesttime &&
        data.timestamp <= last_static_timestamp) {
      window.push_back(data);
    }
  }

  // Return if both of these failed
  if (window.size() < 2) {
    PRINT_INFO(YELLOW "[init-s]: unable to select window of IMU readings, not "
                      "enough readings\n" RESET);
    return false;
  }

  // Calculate the sample variance for the newest window from 1 to 0
  Vec3 a_avg = Vec3::Zero();
  Vec3 w_avg = Vec3::Zero();

  for (const ImuData &data : window) {
    a_avg += data.am;
    w_avg += data.wm;
  }
  a_avg /= (int)window.size();
  w_avg /= (int)window.size();

  // Get rotation with z axis aligned with -g (z_in_G=0,0,1)
  Vec3 z_axis = a_avg / a_avg.norm();
  Mat3 Ro;
  gram_schmidt(z_axis, Ro);
  Vec4 q_GtoI = rot_2_quat(Ro);

  // Set our biases equal to our noise (subtract our gravity from accelerometer
  // bias)
  Vec3 gravity_inG;
  gravity_inG << 0.0, 0.0, params_.gravity_mag;
  Vec3 bg = w_avg;
  Vec3 ba = a_avg - quat_2_Rot(q_GtoI) * gravity_inG;

  // Set our state variables
  state->update_timestamp(window.at(window.size() - 1).timestamp);
  VecX imu_state = VecX::Zero(16);
  imu_state.block(0, 0, 4, 1) = q_GtoI;
  imu_state.block(10, 0, 3, 1) = bg;
  imu_state.block(13, 0, 3, 1) = ba;
  state->imu->set_value(imu_state);
  state->imu->set_fej(imu_state);

  // Just keep the default covariance prior
  VecX prior_diag = VecX::Ones(15);
  prior_diag.topRows<3>() *= 0.017;  // q
  prior_diag.segment<3>(3) *= 0.05;  // p
  prior_diag.segment<3>(6) *= 0.01;  // v
  prior_diag.segment<3>(9) *= 0.02;  // bg
  prior_diag.segment<3>(12) *= 0.02; // ba
  StateHelper::set_initial_imu_square_root_covariance(state, prior_diag);

  return true;
}
