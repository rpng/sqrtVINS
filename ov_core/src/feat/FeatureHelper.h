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





#ifndef OV_CORE_FEATURE_HELPER_H
#define OV_CORE_FEATURE_HELPER_H

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <vector>

#include "Feature.h"
#include "FeatureDatabase.h"
#include "utils/print.h"

namespace ov_core {

/**
 * @brief Contains some nice helper functions for features.
 *
 * These functions should only depend on feature and the feature database.
 */
class FeatureHelper {

public:
  /**
   * @brief This functions will compute the disparity between common features in
   * the two frames.
   *
   * First we find all features in the first frame.
   * Then we loop through each and find the uv of it in the next requested
   * frame. Features are skipped if no tracked feature is found (it was lost).
   * NOTE: this is on the RAW coordinates of the feature not the normalized
   * ones. NOTE: This computes the disparity over all cameras!
   *
   * @param db Feature database pointer
   * @param time0 First camera frame timestamp
   * @param time1 Second camera frame timestamp
   * @param disp_mean Average raw disparity
   * @param disp_var Variance of the disparities
   * @param total_feats Total number of common features
   */
  static void compute_disparity(std::shared_ptr<ov_core::FeatureDatabase> db,
                                double time0, double time1, DataType &disp_mean,
                                DataType &disp_var, int &total_feats) {

    // Get features seen from the first image
    std::vector<std::shared_ptr<Feature>> feats0 =
        db->features_containing(time0, false, true);

    // Compute the disparity
    std::vector<DataType> disparities;
    for (auto &feat : feats0) {

      // Get the two uvs for both times
      for (auto &campairs : feat->timestamps) {

        // First find the two timestamps
        size_t camid = campairs.first;
        auto it0 = std::find(feat->timestamps.at(camid).begin(),
                             feat->timestamps.at(camid).end(), time0);
        auto it1 = std::find(feat->timestamps.at(camid).begin(),
                             feat->timestamps.at(camid).end(), time1);
        if (it0 == feat->timestamps.at(camid).end() ||
            it1 == feat->timestamps.at(camid).end())
          continue;
        auto idx0 = std::distance(feat->timestamps.at(camid).begin(), it0);
        auto idx1 = std::distance(feat->timestamps.at(camid).begin(), it1);

        // Now lets calculate the disparity
        Vec2 uv0 = feat->uvs.at(camid).at(idx0);
        Vec2 uv1 = feat->uvs.at(camid).at(idx1);
        disparities.push_back((uv1 - uv0).norm());
      }
    }

    // If no disparities, just return
    if (disparities.size() < 2) {
      disp_mean = -1;
      disp_var = -1;
      total_feats = 0;
    }

    // Compute mean and standard deviation in respect to it
    disp_mean = 0;
    for (const DataType &disp_i : disparities) {
      disp_mean += disp_i;
    }
    disp_mean /= (DataType)disparities.size();
    disp_var = 0;
    for (const DataType &disp_i : disparities) {
      disp_var += std::pow(disp_i - disp_mean, 2);
    }
    disp_var = std::sqrt(disp_var / (DataType)(disparities.size() - 1));
    total_feats = (int)disparities.size();
  }

  /**
   * @brief This functions will compute the disparity over all features we have
   *
   * NOTE: this is on the RAW coordinates of the feature not the normalized
   * ones. NOTE: This computes the disparity over all cameras!
   *
   * @param db Feature database pointer
   * @param disp_mean Average raw disparity
   * @param disp_var Variance of the disparities
   * @param total_feats Total number of common features
   * @param newest_time Only compute disparity for ones older (-1 to disable)
   * @param oldest_time Only compute disparity for ones newer (-1 to disable)
   */
  static void compute_disparity(std::shared_ptr<ov_core::FeatureDatabase> db,
                                DataType &disp_mean, DataType &disp_var,
                                int &total_feats, double time0, double time1) {

    // Compute the disparity
    std::vector<DataType> disparities;
    Vec2 uv0;
    Vec2 uv1;
    for (auto &feat : db->get_internal_data()) {
      for (auto &campairs : feat.second->timestamps) {

        // Skip if only one observation
        if (campairs.second.size() < 2)
          continue;

        // Now lets calculate the disparity (assumes time array is monotonic)
        size_t camid = campairs.first;

        // Find time0
        auto it = std::find(feat.second->timestamps.at(camid).begin(),
                            feat.second->timestamps.at(camid).end(), time0);
        if (it == feat.second->timestamps.at(camid).end()) {
          continue;
        } else {
          auto idx =
              std::distance(feat.second->timestamps.at(camid).begin(), it);
          uv0 = feat.second->uvs.at(camid).at(idx);
        }

        // Find time1
        it = std::find(feat.second->timestamps.at(camid).begin(),
                       feat.second->timestamps.at(camid).end(), time1);
        if (it == feat.second->timestamps.at(camid).end()) {
          continue;
        } else {
          auto idx =
              std::distance(feat.second->timestamps.at(camid).begin(), it);
          uv1 = feat.second->uvs.at(camid).at(idx);
        }

        disparities.push_back((uv1 - uv0).norm());
      }
    }

    // If no disparities, just return
    if (disparities.size() < 2) {
      disp_mean = -1;
      disp_var = -1;
      total_feats = 0;
      return;
    }

    // Compute mean and standard deviation in respect to it
    disp_mean = 0;
    for (DataType disp_i : disparities) {
      disp_mean += disp_i;
    }
    disp_mean /= (DataType)disparities.size();
    disp_var = 0;
    for (DataType &disp_i : disparities) {
      disp_var += std::pow(disp_i - disp_mean, 2);
    }
    disp_var = std::sqrt(disp_var / (disparities.size() - 1));
    total_feats = (int)disparities.size();

    // compute the median
    std::sort(disparities.begin(), disparities.end());
    DataType disp_median = disparities[disparities.size() / 2];

    // Remove outlier and recompute
    std::vector<DataType> disparities_clean;
    for (const DataType &disp_i : disparities) {
      if (std::abs(disp_i - disp_median) < 3 * disp_var) {
        disparities_clean.push_back(disp_i);
      }
    }

    disp_mean = 0;
    for (const DataType &disp_i : disparities_clean) {
      disp_mean += disp_i;
    }
    disp_mean /= (DataType)disparities_clean.size();
    disp_var = 0;
    for (const DataType &disp_i : disparities_clean) {
      disp_var += std::pow(disp_i - disp_mean, 2);
    }
    disp_var = std::sqrt(disp_var / (DataType)(disparities_clean.size() - 1));
    total_feats = (int)disparities_clean.size();
  }

private:
  // Cannot construct this class
  FeatureHelper() {}
}; // namespace ov_core

} // namespace ov_core

#endif /* OV_CORE_FEATURE_HELPER_H */