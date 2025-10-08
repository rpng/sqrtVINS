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





#include "FeatureDatabase.h"
#include "FeatureHelper.h"

#include "Feature.h"
#include "utils/print.h"

using namespace ov_core;

std::shared_ptr<Feature> FeatureDatabase::get_feature(size_t id, bool remove) {
  std::lock_guard<std::mutex> lck(mtx);
  if (features_idlookup.find(id) != features_idlookup.end()) {
    std::shared_ptr<Feature> temp = features_idlookup.at(id);
    if (remove)
      features_idlookup.erase(id);
    return temp;
  } else {
    return nullptr;
  }
}

bool FeatureDatabase::get_feature_clone(size_t id, Feature &feat) {
  std::lock_guard<std::mutex> lck(mtx);
  if (features_idlookup.find(id) == features_idlookup.end())
    return false;
  // TODO: should probably have a copy constructor function in feature class
  std::shared_ptr<Feature> temp = features_idlookup.at(id);
  feat.featid = temp->featid;
  feat.to_delete = temp->to_delete;
  feat.uvs = temp->uvs;
  feat.uvs_norm = temp->uvs_norm;
  feat.timestamps = temp->timestamps;
  feat.anchor_cam_id = temp->anchor_cam_id;
  feat.anchor_clone_timestamp = temp->anchor_clone_timestamp;
  feat.p_FinA = temp->p_FinA;
  feat.p_FinG = temp->p_FinG;
  return true;
}

void FeatureDatabase::update_feature(size_t id, double timestamp, size_t cam_id,
                                     float u, float v, float u_n, float v_n) {

  // Find this feature using the ID lookup
  std::lock_guard<std::mutex> lck(mtx);
  if (features_idlookup.find(id) != features_idlookup.end()) {
    // Get our feature
    std::shared_ptr<Feature> feat = features_idlookup.at(id);
    // Append this new information to it!
    feat->uvs[cam_id].push_back(Vec2(u, v));
    feat->uvs_norm[cam_id].push_back(Vec2(u_n, v_n));
    feat->timestamps[cam_id].push_back(timestamp);
    if (tracking_stat.find(timestamp) == tracking_stat.end()) {
      tracking_stat[timestamp] = 1;
    } else {
      tracking_stat[timestamp]++;
    }
    return;
  }

  // Debug info
  // PRINT_DEBUG("featdb - adding new feature %d",(int)id);

  // Else we have not found the feature, so lets make it be a new one!
  std::shared_ptr<Feature> feat = std::make_shared<Feature>();
  feat->featid = id;
  feat->uvs[cam_id].push_back(Vec2(u, v));
  feat->uvs_norm[cam_id].push_back(Vec2(u_n, v_n));
  feat->timestamps[cam_id].push_back(timestamp);
  if (tracking_stat.find(timestamp) == tracking_stat.end()) {
    tracking_stat[timestamp] = 1;
  } else {
    tracking_stat[timestamp]++;
  }

  // Append this new feature into our database
  features_idlookup[id] = feat;
}

std::vector<std::shared_ptr<Feature>>
FeatureDatabase::features_not_containing_newer(double timestamp, bool remove,
                                               bool skip_deleted) {

  // Our vector of features that do not have measurements after the specified
  // time
  std::vector<std::shared_ptr<Feature>> feats_old;

  // Now lets loop through all features, and just make sure they are not old
  std::lock_guard<std::mutex> lck(mtx);
  for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
    // Skip if already deleted
    if (skip_deleted && (*it).second->to_delete) {
      it++;
      continue;
    }
    // Loop through each camera
    // If we have a measurement greater-than or equal to the specified, this
    // measurement is find
    bool has_newer_measurement = false;
    for (auto const &pair : (*it).second->timestamps) {
      has_newer_measurement =
          (!pair.second.empty() &&
           pair.second.at(pair.second.size() - 1) >= timestamp);
      if (has_newer_measurement) {
        break;
      }
    }
    // If it is not being actively tracked, then it is old
    if (!has_newer_measurement) {
      feats_old.push_back((*it).second);
      if (remove)
        features_idlookup.erase(it++);
      else
        it++;
    } else {
      it++;
    }
  }

  // Debugging
  // PRINT_DEBUG("feature db size = %u\n", features_idlookup.size())

  // Return the old features
  return feats_old;
}

std::vector<std::shared_ptr<Feature>>
FeatureDatabase::features_containing_older(double timestamp, bool remove,
                                           bool skip_deleted) {

  // Our vector of old features
  std::vector<std::shared_ptr<Feature>> feats_old;

  // Now lets loop through all features, and just make sure they are not old
  std::lock_guard<std::mutex> lck(mtx);
  for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
    // Skip if already deleted
    if (skip_deleted && (*it).second->to_delete) {
      it++;
      continue;
    }
    // Loop through each camera
    // Check if we have at least one time older then the requested
    bool found_containing_older = false;
    for (auto const &pair : (*it).second->timestamps) {
      found_containing_older =
          (!pair.second.empty() && pair.second.at(0) < timestamp);
      if (found_containing_older) {
        break;
      }
    }
    // If it has an older timestamp, then add it
    if (found_containing_older) {
      feats_old.push_back((*it).second);
      if (remove)
        features_idlookup.erase(it++);
      else
        it++;
    } else {
      it++;
    }
  }

  // Debugging
  // PRINT_DEBUG("feature db size = %u\n", features_idlookup.size())

  // Return the old features
  return feats_old;
}

std::vector<std::shared_ptr<Feature>>
FeatureDatabase::features_containing(double timestamp, bool remove,
                                     bool skip_deleted) {

  // Our vector of old features
  std::vector<std::shared_ptr<Feature>> feats_has_timestamp;

  // Now lets loop through all features, and just make sure they are not
  std::lock_guard<std::mutex> lck(mtx);
  for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
    // Skip if already deleted
    if (skip_deleted && (*it).second->to_delete) {
      it++;
      continue;
    }
    // Boolean if it has the timestamp
    // Break out if we found a single timestamp that is equal to the specified
    // time
    bool has_timestamp = false;
    for (auto const &pair : (*it).second->timestamps) {
      has_timestamp = (std::find(pair.second.begin(), pair.second.end(),
                                 timestamp) != pair.second.end());
      if (has_timestamp) {
        break;
      }
    }
    // Remove this feature if it contains the specified timestamp
    if (has_timestamp) {
      feats_has_timestamp.push_back((*it).second);
      if (remove)
        features_idlookup.erase(it++);
      else
        it++;
    } else {
      it++;
    }
  }

  // Debugging
  // PRINT_DEBUG("feature db size = %u\n", features_idlookup.size())
  // PRINT_DEBUG("return vector = %u\n", feats_has_timestamp.size())

  // Return the features
  return feats_has_timestamp;
}

void FeatureDatabase::cleanup() {
  // Loop through all features
  // int sizebefore = (int)features_idlookup.size();
  std::lock_guard<std::mutex> lck(mtx);
  for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
    // If delete flag is set, then delete it
    if ((*it).second->to_delete) {
      features_idlookup.erase(it++);
    } else {
      it++;
    }
  }
  // PRINT_DEBUG("feat db = %d -> %d\n", sizebefore,
  // (int)features_idlookup.size() << std::endl;
}

void FeatureDatabase::cleanup_measurements(double timestamp) {
  std::lock_guard<std::mutex> lck(mtx);
  for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
    // Remove the older measurements
    (*it).second->clean_older_measurements(timestamp);
    // Count how many measurements
    int ct_meas = 0;
    for (const auto &pair : (*it).second->timestamps) {
      ct_meas += (int)(pair.second.size());
    }
    // If delete flag is set, then delete it
    if (ct_meas < 1) {
      features_idlookup.erase(it++);
    } else {
      it++;
    }
  }

  // Also clean up tracking stat
  auto it = tracking_stat.begin();
  while (it != tracking_stat.end()) {
    if ((*it).first < timestamp) {
      it = tracking_stat.erase(it);
    } else {
      it++;
    }
  }
}

void FeatureDatabase::cleanup_measurements_exact(double timestamp) {
  std::lock_guard<std::mutex> lck(mtx);
  std::vector<double> timestamps = {timestamp};
  for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
    // Remove the older measurements
    (*it).second->clean_invalid_measurements(timestamps);
    // Count how many measurements
    int ct_meas = 0;
    for (const auto &pair : (*it).second->timestamps) {
      ct_meas += (int)(pair.second.size());
    }
    // If delete flag is set, then delete it
    if (ct_meas < 1) {
      features_idlookup.erase(it++);
    } else {
      it++;
    }
  }

  // Also clean up tracking stat
  tracking_stat.erase(timestamp);
}

double FeatureDatabase::get_oldest_timestamp() {
  std::lock_guard<std::mutex> lck(mtx);
  double oldest_time = -1;
  for (auto const &feat : features_idlookup) {
    for (auto const &camtimepair : feat.second->timestamps) {
      if (!camtimepair.second.empty() &&
          (oldest_time == -1 || oldest_time > camtimepair.second.at(0))) {
        oldest_time = camtimepair.second.at(0);
      }
    }
  }
  return oldest_time;
}

void FeatureDatabase::append_new_measurements(
    const std::shared_ptr<FeatureDatabase> &database) {
  std::lock_guard<std::mutex> lck(mtx);

  // Loop through the other database's internal database
  // int sizebefore = (int)features_idlookup.size();
  for (const auto &feat : database->get_internal_data()) {
    if (features_idlookup.find(feat.first) != features_idlookup.end()) {

      // For this feature, now try to append the new measurement data
      std::shared_ptr<Feature> temp = features_idlookup.at(feat.first);
      for (const auto &times : feat.second->timestamps) {
        // Append the whole camera vector is not seen
        // Otherwise need to loop through each and append
        size_t cam_id = times.first;
        if (temp->timestamps.find(cam_id) == temp->timestamps.end()) {
          temp->timestamps[cam_id] = feat.second->timestamps.at(cam_id);
          temp->uvs[cam_id] = feat.second->uvs.at(cam_id);
          temp->uvs_norm[cam_id] = feat.second->uvs_norm.at(cam_id);
        } else {
          auto temp_times = temp->timestamps.at(cam_id);
          for (size_t i = 0; i < feat.second->timestamps.at(cam_id).size();
               i++) {
            double time_to_find = feat.second->timestamps.at(cam_id).at(i);
            if (std::find(temp_times.begin(), temp_times.end(), time_to_find) ==
                temp_times.end()) {
              temp->timestamps.at(cam_id).push_back(
                  feat.second->timestamps.at(cam_id).at(i));
              temp->uvs.at(cam_id).push_back(feat.second->uvs.at(cam_id).at(i));
              temp->uvs_norm.at(cam_id).push_back(
                  feat.second->uvs_norm.at(cam_id).at(i));
            }
          }
        }
      }

    } else {

      // Else we have not found the feature, so lets make it be a new one!
      std::shared_ptr<Feature> temp = std::make_shared<Feature>();
      temp->featid = feat.second->featid;
      temp->timestamps = feat.second->timestamps;
      temp->uvs = feat.second->uvs;
      temp->uvs_norm = feat.second->uvs_norm;
      features_idlookup[feat.first] = temp;
    }
  }
  // PRINT_DEBUG("feat db = %d -> %d\n", sizebefore,
  // (int)features_idlookup.size() << std::endl;
}

/**
 * @brief This functions will compute the disparity over all features we have
 *
 * NOTE: this is on the RAW coordinates of the feature not the normalized ones.
 * NOTE: This computes the disparity over all cameras!
 *
 * @param db Feature database pointer
 * @param disp_mean Average raw disparity
 * @param disp_var Variance of the disparities
 * @param total_feats Total number of common features
 * @param newest_time Only compute disparity for ones older (-1 to disable)
 * @param oldest_time Only compute disparity for ones newer (-1 to disable)
 */
DataType FeatureDatabase::compute_disparity(double time0, double time1) {

  // Compute the disparity
  std::vector<DataType> disparities;
  Vec2 uv0;
  Vec2 uv1;
  DataType disp_mean, disp_var;
  for (auto &feat : get_internal_data()) {
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
        auto idx = std::distance(feat.second->timestamps.at(camid).begin(), it);
        uv0 = feat.second->uvs.at(camid).at(idx);
      }

      // Find time1
      it = std::find(feat.second->timestamps.at(camid).begin(),
                     feat.second->timestamps.at(camid).end(), time1);
      if (it == feat.second->timestamps.at(camid).end()) {
        continue;
      } else {
        auto idx = std::distance(feat.second->timestamps.at(camid).begin(), it);
        uv1 = feat.second->uvs.at(camid).at(idx);
      }

      disparities.push_back((uv1 - uv0).norm());
    }
  }

  // If no enought features, just return
  if (disparities.size() < 15) {
    disp_mean = -1;
    disp_var = -1;
    return disp_mean;
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
  return disp_mean;
}

void FeatureDatabase::update_disp_info() {
  std::set<double> all_timestamps;
  for (const auto &feat : features_idlookup) {
    for (const auto &camtimepair : feat.second->timestamps) {
      for (const auto &time : camtimepair.second) {
        all_timestamps.insert(time);
      }
    }
  }

  // Remove the no-exist timestamps
  auto it = map_disp.begin();
  while (it != map_disp.end()) {
    if (all_timestamps.find((*it).first) == all_timestamps.end()) {
      it = map_disp.erase(it);
    } else {
      it++;
    }
  }

  // Insert the new timestamps
  for (const auto &time0 : all_timestamps) {
    for (const auto &time1 : all_timestamps) {
      if (time0 == time1)
        continue;

      if (map_disp.find(time0) != map_disp.end() &&
          map_disp.at(time0).find(time1) != map_disp.at(time0).end()) {
        continue;
      }

      if (map_disp.find(time1) != map_disp.end() &&
          map_disp.at(time1).find(time0) != map_disp.at(time1).end()) {
        continue;
      }

      DataType avg_disp = compute_disparity(time0, time1);
      map_disp[time0][time1] = avg_disp;
      map_disp[time1][time0] = avg_disp;
    }
  }
}
