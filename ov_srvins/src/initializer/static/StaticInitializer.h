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





#ifndef OV_SRVINS_STATICINITIALIZER_H
#define OV_SRVINS_STATICINITIALIZER_H

#include "initializer/InertialInitializerOptions.h"
#include "state/State.h"

namespace ov_core {
class FeatureDatabase;
struct ImuData;
} // namespace ov_core
namespace ov_type {
class Type;
class IMU;
} // namespace ov_type

namespace ov_srvins {

/**
 * @brief Initializer for a static visual-inertial system.
 *
 * This implementation that assumes that the imu starts from standing still.
 * To initialize from standstill:
 * 1. Collect all inertial measurements from the static period
 * 2. Recover intiail orientation and biases
 * 3. Recover covariance
 *
 */
class StaticInitializer {

public:
  /**
   * @brief Default constructor
   * @param params_ Parameters loaded from either ROS or CMDLINE
   * @param db Feature tracker database with all features in it
   * @param imu_data_ Shared pointer to our IMU vector of historical information
   */
  explicit StaticInitializer(
      const InertialInitializerOptions &params,
      std::shared_ptr<ov_core::FeatureDatabase> db,
      std::shared_ptr<std::vector<ov_core::ImuData>> imu_data);

  /**
   * @brief Use the static window to initialize the state
   *
   * @param state VIO state to be initialized
   * @param last_static_timestamp Last timestamp to consider static to determine
   * the static window
   * @return True if we have successfully initialized our system
   */
  bool initialize(std::shared_ptr<ov_srvins::State> state,
                  double last_static_timestamp);

private:
  /// Initialization parameters
  InertialInitializerOptions params_;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> db_;

  /// Our history of IMU messages (time, angular, linear)
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data_;
};

} // namespace ov_srvins

#endif // OV_SRVINS_STATICINITIALIZER_H
