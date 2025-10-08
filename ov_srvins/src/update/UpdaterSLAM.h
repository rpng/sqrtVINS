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





#ifndef OV_SRVINS_UPDATER_SLAM_H
#define OV_SRVINS_UPDATER_SLAM_H

#include <Eigen/Eigen>
#include <memory>

#include "UpdaterOptions.h"
#include "feat/FeatureInitializer.h"
#include "feat/FeatureInitializerOptions.h"
#include "state/State.h"

namespace ov_srvins {

/**
 * @brief Will compute the system for our sparse SLAM features and update the
 * filter.
 *
 * This class is responsible for performing delayed feature initialization, SLAM
 * update, and SLAM anchor change for anchored feature representations.
 *
 * @note This class is modified from OpenVINS' UpdaterSLAM class, Aruco Tag
 * related logic are kept but not optimized.
 */
class UpdaterSLAM {

public:
  /**
   * @brief Default constructor for our SLAM updater
   *
   * Our updater has a feature initializer which we use to initialize features
   * as needed. Also the options allow for one to tune the different parameters
   * for update.
   *
   * @param options_slam Updater options (include measurement noise value) for
   * SLAM features
   * @param options_aruco Updater options (include measurement noise value) for
   * ARUCO features
   * @param feat_init_options Feature initializer options
   */
  UpdaterSLAM(UpdaterOptions &options_slam, UpdaterOptions &options_aruco,
              ov_core::FeatureInitializerOptions &feat_init_options);

  /**
   * @brief Given tracked SLAM features, this will try to use them to update the
   * state.
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   */
  void update(std::shared_ptr<State> state,
              std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

  /**
   * @brief Given max track features, this will try to use them to initialize
   * them in the state.
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   */
  void
  delayed_init(std::shared_ptr<State> state,
               std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

  /**
   * @brief Initialize SLAM features from MSCKF features
   * @param state State of the filter
   * @param feature_vec MSCKF features
   */
  void delayed_init_from_MSCKF(
      std::shared_ptr<State> state,
      std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

  /**
   * @brief Will change SLAM feature anchors if it will be marginalized
   *
   * Makes sure that if any clone is about to be marginalized, it changes anchor
   * representation. By default, this will shift the anchor into the newest IMU
   * clone and keep the camera calibration anchor the same.
   *
   * @param state State of the filter
   */
  void change_anchors(std::shared_ptr<State> state);

protected:
  /**
   * @brief Shifts landmark anchor to new clone
   * @param state State of filter
   * @param landmark landmark whose anchor is being shifter
   * @param new_anchor_timestamp Clone timestamp we want to move to
   * @param new_cam_id Which camera frame we want to move to
   */
  void perform_anchor_change(std::shared_ptr<State> state,
                             std::shared_ptr<ov_type::Landmark> landmark,
                             double new_anchor_timestamp, size_t new_cam_id);

  /// Options used during update for slam features
  UpdaterOptions options_slam_;

  /// Options used during update for aruco features
  UpdaterOptions options_aruco_;

  /// Feature initializer class object
  std::shared_ptr<ov_core::FeatureInitializer> initializer_feat_;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, float> chi_squared_table_;
};

} // namespace ov_srvins

#endif // OV_SRVINS_UPDATER_SLAM_H
