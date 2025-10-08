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





#ifndef OV_SRVINS_UPDATER_ZEROVELOCITY_H
#define OV_SRVINS_UPDATER_ZEROVELOCITY_H

#include <memory>

#include "UpdaterOptions.h"
#include "feat/FeatureDatabase.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "utils/NoiseManager.h"
#include "utils/sensor_data.h"

namespace ov_srvins {

/**
 * @brief Will try to *detect* and then update using zero velocity assumption.
 *
 * Consider the case that a VIO unit remains stationary for a period time.
 * Typically this can cause issues in a monocular system without SLAM features
 * since no features can be triangulated. Additional, if features could be
 * triangulated (e.g. stereo) the quality can be poor and hurt performance. If
 * we can detect the cases where we are stationary then we can leverage this to
 * prevent the need to do feature update during this period. The main
 * application would be using this on a **wheeled vehicle** which needs to stop
 * (e.g. stop lights or parking).
 *
 * @note This class is just modified from OpenVINS' UpdaterZeroVelocity
 * class, not efficiency optimized.
 */
class UpdaterZeroVelocity {

public:
  /**
   * @brief Default constructor for our zero velocity detector and updater.
   * @param options Updater options (chi2 multiplier)
   * @param noises imu noise characteristics (continuous time)
   * @param db Feature tracker database with all features in it
   * @param prop Propagator class object which can predict the state forward in
   * time
   * @param gravity_mag Global gravity magnitude of the system (normally 9.81)
   * @param zupt_max_velocity Max velocity we should consider to do a update
   * with
   * @param zupt_noise_multiplier Multiplier of our IMU noise matrix (default
   * should be 1.0)
   * @param zupt_max_disparity Max disparity we should consider to do a update
   * with
   */
  UpdaterZeroVelocity(UpdaterOptions &options, NoiseManager &noises,
                      std::shared_ptr<ov_core::FeatureDatabase> db,
                      std::shared_ptr<Propagator> prop, DataType gravity_mag,
                      DataType zupt_max_velocity,
                      DataType zupt_noise_multiplier,
                      DataType zupt_max_disparity);

  /**
   * @brief Feed function for inertial data
   * @param message Contains our timestamp and inertial information
   * @param oldest_time Time that we can discard measurements before
   */
  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1) {

    // Append it to our vector
    imu_data_.emplace_back(message);
    clean_old_imu_measurements(oldest_time - 0.10);
  }

  /**
   * @brief This will remove any IMU measurements that are older then the given
   * measurement time
   * @param oldest_time Time that we can discard measurements before (in IMU
   * clock)
   */
  void clean_old_imu_measurements(double oldest_time) {
    if (oldest_time < 0)
      return;
    auto it0 = imu_data_.begin();
    while (it0 != imu_data_.end()) {
      if (it0->timestamp < oldest_time) {
        it0 = imu_data_.erase(it0);
      } else {
        it0++;
      }
    }
  }

  /**
   * @brief Will first detect if the system is zero velocity, then will update.
   * @param state State of the filter
   * @param timestamp Next camera timestamp we want to see if we should
   * propagate to.
   * @return True if the system is currently at zero velocity
   */
  bool try_update(std::shared_ptr<State> state, double timestamp);

protected:
  /// Options used during update (chi2 multiplier)
  UpdaterOptions options_;

  /// Container for the imu noise values
  NoiseManager noises_;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> db_;

  /// Our propagator!
  std::shared_ptr<Propagator> propagator_;

  /// Gravity vector
  Vec3 gravity_;

  /// Max velocity (m/s) that we should consider a zupt with
  DataType zupt_max_velocity_ = 1.0;

  /// Multiplier of our IMU noise matrix (default should be 1.0)
  DataType zupt_noise_multiplier_ = 1.0;

  /// Max disparity (pixels) that we should consider a zupt with
  DataType zupt_max_disparity_ = 1.0;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, DataType> chi_squared_table_;

  /// Our history of IMU messages (time, angular, linear)
  std::vector<ov_core::ImuData> imu_data_;

  /// Estimate for time offset at last propagation time
  double last_prop_time_offset_ = 0.0;
  bool have_last_prop_time_offset_ = false;

  /// Last timestamp we did zero velocity update with
  double last_zupt_state_timestamp_ = 0.0;

  /// Number of times we have called update
  int last_zupt_count_ = 0;
};

} // namespace ov_srvins

#endif // OV_SRVINS_UPDATER_ZEROVELOCITY_H
