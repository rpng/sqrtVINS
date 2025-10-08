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





#ifndef OV_SRVINS_UPDATER_MSCKF_H
#define OV_SRVINS_UPDATER_MSCKF_H

#include <Eigen/Eigen>
#include <memory>

#include "feat/FeatureInitializer.h"
#include "feat/FeatureInitializerOptions.h"
#include "state/State.h"

#include "UpdaterOptions.h"

namespace ov_srvins {

/**
 * @brief Will compute the system for our sparse features and update the filter.
 *
 * This class is responsible for computing the entire linear system for all
 * features that are going to be used in an update. This follows the original
 * MSCKF, where we first triangulate features, we then nullspace project the
 * feature Jacobian. After this we compress all the measurements to have an
 * efficient update and update the state.
 */
class UpdaterMSCKF {

public:
  /**
   * @brief Default constructor for our MSCKF updater
   *
   * Our updater has a feature initializer which we use to initialize features
   * as needed. Also the options allow for one to tune the different parameters
   * for update.
   *
   * @param options Updater options (include measurement noise value)
   * @param feat_init_options Feature initializer options
   */
  UpdaterMSCKF(UpdaterOptions &options,
               ov_core::FeatureInitializerOptions &feat_init_options);

  /**
   * @brief Given tracked features, this will try to use them to update the
   * state.
   *
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   * @param is_iterative If we are doing an iterative update (store jacobians)
   */
  void update(std::shared_ptr<State> state,
              std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec,
              bool is_iterative = false, bool require_HUT = true);

  /**
   * @brief Update MSCKF features in the state, used in iterative update
   *
   * @param state State of the filter
   * @return  void    [return description]
   */
  void update_features(std::shared_ptr<State> state);

protected:
  /// Options used during update
  UpdaterOptions options_;

  /// Feature initializer class object
  std::shared_ptr<ov_core::FeatureInitializer> initializer_feat_;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, DataType> chi_squared_table_;
};

} // namespace ov_srvins

#endif // OV_SRVINS_UPDATER_MSCKF_H
