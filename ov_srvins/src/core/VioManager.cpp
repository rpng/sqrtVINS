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





#include "VioManager.h"

#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "feat/FeatureInitializer.h"
#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "track/TrackSIM.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "utils/DataType.h"
#include "utils/opencv_lambda_body.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

#include "initializer/InertialInitializer.h"

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "update/UpdaterZeroVelocity.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_srvins;

VioManager::VioManager(VioManagerOptions &params_)
    : thread_init_running(false), thread_init_success(false) {

  // Nice startup message
  PRINT_DEBUG("=======================================\n");
  PRINT_DEBUG("OPENVINS ON-MANIFOLD EKF IS STARTING\n");
  PRINT_DEBUG("=======================================\n");

  // Nice debug
  this->params = params_;
  params.print_and_load_estimator();
  params.print_and_load_noise();
  params.print_and_load_state();
  params.print_and_load_trackers();

  // This will globally set the thread count we will use
  // -1 will reset to the system default threading (usually the num of cores)
  cv::setNumThreads(params.num_opencv_threads);
  cv::setRNGSeed(0);

  // Create the state!!
  state = std::make_shared<State>(params.state_options, params.init_options);

  // Timeoffset from camera to IMU
  VecX temp_camimu_dt;
  temp_camimu_dt.resize(1);
  temp_camimu_dt(0) = params.calib_camimu_dt;
  state->calib_dt_CAMtoIMU->set_value(temp_camimu_dt);
  state->calib_dt_CAMtoIMU->set_fej(temp_camimu_dt);

  // Loop through and load each of the cameras
  state->cam_intrinsics_cameras = params.camera_intrinsics;
  for (int i = 0; i < state->options.num_cameras; i++) {
    state->cam_intrinsics.at(i)->set_value(
        params.camera_intrinsics.at(i)->get_value());
    state->cam_intrinsics.at(i)->set_fej(
        params.camera_intrinsics.at(i)->get_value());
    state->calib_IMUtoCAM.at(i)->set_value(params.camera_extrinsics.at(i));
    state->calib_IMUtoCAM.at(i)->set_fej(params.camera_extrinsics.at(i));
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // If we are recording statistics, then open our file
  if (params.record_timing_information) {
    // If the file exists, then delete it
    if (boost::filesystem::exists(params.record_timing_filepath)) {
      boost::filesystem::remove(params.record_timing_filepath);
      PRINT_INFO(YELLOW "[STATS]: found old file found, deleted...\n" RESET);
    }
    // Create the directory that we will open the file in
    boost::filesystem::path p(params.record_timing_filepath);
    boost::filesystem::create_directories(p.parent_path());
    // Open our statistics file!
    of_statistics.open(params.record_timing_filepath,
                       std::ofstream::out | std::ofstream::app);
    // Write the header information into it

    of_statistics
        << "# timestamp (sec),tracking,propagation,marg,msckf update,";
    if (state->options.max_slam_features > 0) {
      of_statistics << "slam update,slam delayed,";
    }
    of_statistics << "state init,qr,back sub,re-tri,total" << std::endl;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Let's make a feature extractor
  // NOTE: after we initialize we will increase the total number of feature
  // tracks NOTE: we will split the total number of features over all cameras
  // uniformly
  int init_max_features =
      std::floor((float)params.init_options.init_max_features /
                 (float)params.state_options.num_cameras);
  if (params.use_klt) {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackKLT(
        state->cam_intrinsics_cameras, init_max_features,
        state->options.max_aruco_features, params.use_stereo,
        params.histogram_method, params.fast_threshold, params.grid_x,
        params.grid_y, params.min_px_dist, params.ransac_th));
  } else {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackDescriptor(
        state->cam_intrinsics_cameras, init_max_features,
        state->options.max_aruco_features, params.use_stereo,
        params.histogram_method, params.fast_threshold, params.grid_x,
        params.grid_y, params.min_px_dist, params.knn_ratio));
  }

  // Initialize our aruco tag extractor
  if (params.use_aruco) {
    trackARUCO = std::shared_ptr<TrackBase>(new TrackAruco(
        state->cam_intrinsics_cameras, state->options.max_aruco_features,
        params.use_stereo, params.histogram_method, params.downsize_aruco));
  }

  // Initialize our state propagator
  propagator =
      std::make_shared<Propagator>(params.imu_noises, params.gravity_mag);

  // Make the updater!
  updaterMSCKF = std::make_shared<UpdaterMSCKF>(params.msckf_options,
                                                params.featinit_options);
  updaterSLAM = std::make_shared<UpdaterSLAM>(
      params.slam_options, params.aruco_options, params.featinit_options);

  // Our state initialize
  initializer = std::make_shared<ov_srvins::InertialInitializer>(
      params.init_options, trackFEATS->get_feature_database(), propagator,
      updaterMSCKF, updaterSLAM);

  // If we are using zero velocity updates, then create the updater
  if (params.try_zupt) {
    updaterZUPT = std::make_shared<UpdaterZeroVelocity>(
        params.zupt_options, params.imu_noises,
        trackFEATS->get_feature_database(), propagator, params.gravity_mag,
        params.zupt_max_velocity, params.zupt_noise_multiplier,
        params.zupt_max_disparity);
  }
}

void VioManager::feed_measurement_imu(const ov_core::ImuData &message) {

  // The oldest time we need IMU with is the last clone
  // We shouldn't really need the whole window, but if we go backwards in time
  // we will
  double oldest_time = state->margtimestep();
  if (oldest_time > state->timestamp) {
    oldest_time = -1;
  }
  if (!is_initialized_vio) {
    oldest_time = message.timestamp - params.init_options.init_window_time +
                  state->calib_dt_CAMtoIMU->value()(0) - 0.1;
  }
  propagator->feed_imu(message, oldest_time);

  // Push back to the zero velocity updater if it is enabled
  // No need to push back if we are just doing the zv-update at the begining and
  // we have moved
  if (is_initialized_vio && updaterZUPT != nullptr &&
      (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    updaterZUPT->feed_imu(message, oldest_time);
  }
}

void VioManager::track_image_and_update(
    const ov_core::CameraData &message_const) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Assert we have valid measurement data and ids
  assert(!message_const.sensor_ids.empty());
  assert(message_const.sensor_ids.size() == message_const.images.size());
  for (size_t i = 0; i < message_const.sensor_ids.size() - 1; i++) {
    assert(message_const.sensor_ids.at(i) !=
           message_const.sensor_ids.at(i + 1));
  }

  // Downsample if we are downsampling
  ov_core::CameraData message = message_const;
  for (size_t i = 0; i < message.sensor_ids.size() && params.downsample_cameras;
       i++) {
    cv::Mat img = message.images.at(i);
    cv::Mat mask = message.masks.at(i);
    cv::Mat img_temp, mask_temp;
    cv::pyrDown(img, img_temp, cv::Size(img.cols / 2.0, img.rows / 2.0));
    message.images.at(i) = img_temp;
    cv::pyrDown(mask, mask_temp, cv::Size(mask.cols / 2.0, mask.rows / 2.0));
    message.masks.at(i) = mask_temp;
  }

  // Perform our feature tracking!
  trackFEATS->feed_new_camera(message);

  // If the aruco tracker is available, the also pass to it
  // NOTE: binocular tracking for aruco doesn't make sense as we by default have
  // the ids NOTE: thus we just call the stereo tracking if we are doing
  // binocular!
  if (is_initialized_vio && trackARUCO != nullptr) {
    trackARUCO->feed_new_camera(message);
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization
  // phase If we have since moved, then we should never try to do a zero
  // velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr &&
      (!params.zupt_only_at_beginning || !has_moved_since_zupt) &&
      state->features_SLAM.empty()) {
    // If the same state time, use the previous timestep decision
    if (state->timestamp != message.timestamp) {
      state->setup_matrix_buffer();
      did_zupt_update = updaterZUPT->try_update(state, message.timestamp);
    }
    if (did_zupt_update) {
      assert(state->timestamp == message.timestamp);
      propagator->clean_old_imu_measurements(
          message.timestamp + state->calib_dt_CAMtoIMU->value()(0) - 0.10);
      updaterZUPT->clean_old_imu_measurements(
          message.timestamp + state->calib_dt_CAMtoIMU->value()(0) - 0.10);
      return;
    }
  }

  // If we do not have VIO initialization, then try to initialize
  // TODO: Or if we are trying to reset the system, then do that here!
  if (!is_initialized_vio) {
    is_initialized_vio = try_to_initialize(message);
    if (!is_initialized_vio) {
      double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
      PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
      return;
    }
  }

  // Call on our propagate and update function
  do_feature_propagate_update(message);
}

void VioManager::feed_measurement_simulation(
    double timestamp, const std::vector<int> &camids,
    const std::vector<std::vector<std::pair<size_t, Eigen::Vector2f>>> &feats) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Check if we actually have a simulated tracker
  // If not, recreate and re-cast the tracker to our simulation tracker
  std::shared_ptr<TrackSIM> trackSIM =
      std::dynamic_pointer_cast<TrackSIM>(trackFEATS);
  if (trackSIM == nullptr) {
    // Replace with the simulated tracker
    trackSIM = std::make_shared<TrackSIM>(state->cam_intrinsics_cameras,
                                          state->options.max_aruco_features);
    trackFEATS = trackSIM;
    // Need to also replace it in init and zv-upt since it points to the
    // trackFEATS db pointer
    initializer = std::make_shared<ov_srvins::InertialInitializer>(
        params.init_options, trackFEATS->get_feature_database(), propagator,
        updaterMSCKF, updaterSLAM);
    if (params.try_zupt) {
      updaterZUPT = std::make_shared<UpdaterZeroVelocity>(
          params.zupt_options, params.imu_noises,
          trackFEATS->get_feature_database(), propagator, params.gravity_mag,
          params.zupt_max_velocity, params.zupt_noise_multiplier,
          params.zupt_max_disparity);
    }
    PRINT_WARNING(RED
                  "[SIM]: casting our tracker to a TrackSIM object!\n" RESET);
  }

  // Feed our simulation tracker
  trackSIM->feed_measurement_simulation(timestamp, camids, feats);
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization
  // phase If we have since moved, then we should never try to do a zero
  // velocity update! If we have SLAM features, no need to use zero velocity
  // updates
  if (is_initialized_vio && updaterZUPT != nullptr &&
      (!params.zupt_only_at_beginning || !has_moved_since_zupt) &&
      state->features_SLAM.empty()) {
    // If the same state time, use the previous timestep decision
    if (state->timestamp != timestamp) {
      state->setup_matrix_buffer();
      did_zupt_update = updaterZUPT->try_update(state, timestamp);
    }
    if (did_zupt_update) {
      assert(state->timestamp == timestamp);
      propagator->clean_old_imu_measurements(
          timestamp + state->calib_dt_CAMtoIMU->value()(0) - 0.10);
      updaterZUPT->clean_old_imu_measurements(
          timestamp + state->calib_dt_CAMtoIMU->value()(0) - 0.10);
      return;
    }
  }

  // If we do not have VIO initialization, then return an error
  if (!is_initialized_vio) {
    PRINT_ERROR(RED "[SIM]: your vio system should already be initialized "
                    "before simulating features!!!\n" RESET);
    PRINT_ERROR(RED "[SIM]: initialize your system first before calling "
                    "feed_measurement_simulation()!!!!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Call on our propagate and update function
  // Simulation is either all sync, or single camera...
  ov_core::CameraData message;
  message.timestamp = timestamp;
  for (auto const &camid : camids) {
    int width = state->cam_intrinsics_cameras.at(camid)->w();
    int height = state->cam_intrinsics_cameras.at(camid)->h();
    message.sensor_ids.push_back(camid);
    message.images.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
    message.masks.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
  }
  do_feature_propagate_update(message);
}

void VioManager::do_feature_propagate_update(
    const ov_core::CameraData &message) {
  //===================================================================================
  // State propagation, and clone augmentation
  //===================================================================================

  // Return if the camera measurement is out of order
  if (state->timestamp > message.timestamp) {
    PRINT_WARNING(YELLOW "image received out of order, unable to do anything "
                         "(prop dt = %3f)\n" RESET,
                  (message.timestamp - state->timestamp));
    return;
  }

  // Propagate the state forward to the current update time
  // Also augment it with a new clone!
  // NOTE: if the state is already at the given time (can happen in sim)
  // NOTE: then no need to prop since we already are at the desired timestep
  if (state->timestamp != message.timestamp) {
    propagator->propagate_and_clone(state, message.timestamp);
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // If we have not reached max clones, we should just return...
  // This isn't super ideal, but it keeps the logic after this easier...
  // We can start processing things when we have at least 5 clones since we can
  // start triangulating things...
  if ((int)state->clones_IMU.size() <
          std::min(state->options.max_clone_size, 5) &&
      state->features_SLAM.empty()) {
    PRINT_DEBUG("waiting for enough clone states (%d of %d)....\n",
                (int)state->clones_IMU.size(),
                std::min(state->options.max_clone_size, 5));
    return;
  }

  // Return if we where unable to propagate
  if (state->timestamp != message.timestamp) {
    PRINT_WARNING(RED "[PROP]: Propagator unable to propagate the state "
                      "forward in time!\n" RESET);
    PRINT_WARNING(
        RED "[PROP]: It has been %.3f since last time we propagated\n" RESET,
        message.timestamp - state->timestamp);
    return;
  }
  has_moved_since_zupt = true;

  //===================================================================================
  // MSCKF features and KLT tracks that are SLAM features
  //===================================================================================
  // Cleanup any features older than the marginalization time
  // Add one here to match openvins logic (openvins always has 12 clones when
  // doing update when set num_clones:11)
  if ((int)state->clones_IMU.size() > state->options.max_clone_size + 1) {
    trackFEATS->get_feature_database()->cleanup_measurements(
        state->margtimestep());
    if (trackARUCO != nullptr) {
      trackARUCO->get_feature_database()->cleanup_measurements(
          state->margtimestep());
    }
  }

  // Lets marginalize out all old SLAM features here
  // These are ones that where not successfully tracked into the current frame
  // We do *NOT* marginalize out our aruco tags landmarks
  StateHelper::marginalize_slam(state);

  // First do anchor change if we are about to lose an anchor pose
  state->calculate_clone_poses();
  if (state->options.do_fej) {
    state->calculate_clone_poses_fej();
  }
  updaterSLAM->change_anchors(state);

  // Firstly marginalize the oldest clone if needed
  StateHelper::marginalize_old_clone(state);
  StateHelper::marginalize(state);
  rT4 = boost::posix_time::microsec_clock::local_time();

  // Now, lets get all features that should be used for an update that are lost
  // in the newest frame We explicitly request features that have not been
  // deleted (used) in another update step
  std::vector<std::shared_ptr<Feature>> feats_lost, feats_marg, feats_slam;
  feats_lost =
      trackFEATS->get_feature_database()->features_not_containing_newer(
          state->timestamp, false, true);

  // Don't need to get the oldest features until we reach our max number of
  // clones
  if ((int)state->clones_IMU.size() == state->options.max_clone_size + 1 ||
      (int)state->clones_IMU.size() > 5) {
    feats_marg = trackFEATS->get_feature_database()->features_containing(
        state->margtimestep(), false, true);
    if (trackARUCO != nullptr &&
        message.timestamp - startup_time >= params.dt_slam_delay) {
      feats_slam = trackARUCO->get_feature_database()->features_containing(
          state->margtimestep(), false, true);
    }
  }

  // Remove any lost features that were from other image streams
  // E.g: if we are cam1 and cam0 has not processed yet, we don't want to try to
  // use those in the update yet E.g: thus we wait until cam0 process its newest
  // image to remove features which were seen from that camera
  auto it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    bool found_current_message_camid = false;
    for (const auto &camuvpair : (*it1)->uvs) {
      if (std::find(message.sensor_ids.begin(), message.sensor_ids.end(),
                    camuvpair.first) != message.sensor_ids.end()) {
        found_current_message_camid = true;
        break;
      }
    }
    if (found_current_message_camid) {
      it1++;
    } else {
      it1 = feats_lost.erase(it1);
    }
  }

  // We also need to make sure that the max tracks does not contain any lost
  // features This could happen if the feature was lost in the last frame, but
  // has a measurement at the marg timestep
  it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    if (std::find(feats_marg.begin(), feats_marg.end(), (*it1)) !=
        feats_marg.end()) {
      // PRINT_WARNING(YELLOW "FOUND FEATURE THAT WAS IN BOTH feats_lost and
      // feats_marg!!!!!!\n" RESET);
      it1 = feats_lost.erase(it1);
    } else {
      it1++;
    }
  }

  // Find tracks that have reached max length, these can be made into SLAM
  // features
  std::vector<std::shared_ptr<Feature>> feats_maxtracks;
  auto it2 = feats_marg.begin();
  while (it2 != feats_marg.end()) {
    // See if any of our camera's reached max track
    bool reached_max = false;
    for (const auto &cams : (*it2)->timestamps) {
      // Add one here to match openvins logic
      if ((int)cams.second.size() > state->options.max_clone_size) {
        reached_max = true;
        break;
      }
    }
    // If max track, then add it to our possible slam feature list
    if (reached_max) {
      feats_maxtracks.push_back(*it2);
      it2 = feats_marg.erase(it2);
    } else {
      it2++;
    }
  }

  // Count how many aruco tags we have in our state
  int curr_aruco_tags = 0;
  auto it0 = state->features_SLAM.begin();
  while (it0 != state->features_SLAM.end()) {
    if ((int)(*it0).second->featid <= 4 * state->options.max_aruco_features)
      curr_aruco_tags++;
    it0++;
  }

  // Append a new SLAM feature if we have the room to do so
  // Also check that we have waited our delay amount (normally prevents bad
  // first set of slam points)
  if (state->options.max_slam_features > 0 &&
      message.timestamp - startup_time >= params.dt_slam_delay &&
      (int)state->features_SLAM.size() <
          state->options.max_slam_features + curr_aruco_tags) {
    // Get the total amount to add, then the max amount that we can add given
    // our marginalize feature array
    int amount_to_add = (state->options.max_slam_features + curr_aruco_tags) -
                        (int)state->features_SLAM.size();
    int valid_amount = (amount_to_add > (int)feats_maxtracks.size())
                           ? (int)feats_maxtracks.size()
                           : amount_to_add;
    // If we have at least 1 that we can add, lets add it!
    // Note: we remove them from the feat_marg array since we don't want to
    // reuse information...
    if (valid_amount > 0) {
      feats_slam.insert(feats_slam.end(), feats_maxtracks.end() - valid_amount,
                        feats_maxtracks.end());
      feats_maxtracks.erase(feats_maxtracks.end() - valid_amount,
                            feats_maxtracks.end());
    }
  }

  // Loop through current SLAM features, we have tracks of them, grab them for
  // this update! NOTE: if we have a slam feature that has lost tracking, then
  // we should marginalize it out NOTE: we only enforce this if the current
  // camera message is where the feature was seen from NOTE: if you do not use
  // FEJ, these types of slam features *degrade* the estimator performance....
  // NOTE: we will also marginalize SLAM features if they have failed their
  // update a couple times in a row
  for (std::pair<const size_t, std::shared_ptr<Landmark>> &landmark :
       state->features_SLAM) {
    if (trackARUCO != nullptr) {
      std::shared_ptr<Feature> feat1 =
          trackARUCO->get_feature_database()->get_feature(
              landmark.second->featid);
      if (feat1 != nullptr)
        feats_slam.push_back(feat1);
    }
    std::shared_ptr<Feature> feat2 =
        trackFEATS->get_feature_database()->get_feature(
            landmark.second->featid);
    if (feat2 != nullptr)
      feats_slam.push_back(feat2);
    assert(landmark.second->unique_camera_id != -1);
    bool current_unique_cam =
        std::find(message.sensor_ids.begin(), message.sensor_ids.end(),
                  landmark.second->unique_camera_id) !=
        message.sensor_ids.end();
    if (feat2 == nullptr && current_unique_cam)
      landmark.second->should_marg = true;
    if (landmark.second->update_fail_count > 1)
      landmark.second->should_marg = true;
  }

  // Separate our SLAM features into new ones, and old ones
  std::vector<std::shared_ptr<Feature>> feats_slam_DELAYED, feats_slam_UPDATE;
  for (size_t i = 0; i < feats_slam.size(); i++) {
    if (state->features_SLAM.find(feats_slam.at(i)->featid) !=
        state->features_SLAM.end()) {
      feats_slam_UPDATE.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: found old feature %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    } else {
      feats_slam_DELAYED.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: new feature ready %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    }
  }

  // Concatenate our MSCKF feature arrays (i.e., ones not being used for slam
  // updates)
  std::vector<std::shared_ptr<Feature>> featsup_MSCKF = feats_lost;
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(),
                       feats_marg.end());
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_maxtracks.begin(),
                       feats_maxtracks.end());

  //===================================================================================
  // Now that we have a list of features, lets do the EKF update for MSCKF and
  // SLAM!
  //===================================================================================

  // Sort based on track length
  // TODO: we should have better selection logic here (i.e. even feature
  // distribution in the FOV etc..)
  // TODO: right now features that are "lost" are at the front of this vector,
  // while ones at the end are long-tracks
  auto compare_feat = [](const std::shared_ptr<Feature> &a,
                         const std::shared_ptr<Feature> &b) -> bool {
    size_t asize = 0;
    size_t bsize = 0;
    for (const auto &pair : a->timestamps)
      asize += pair.second.size();
    for (const auto &pair : b->timestamps)
      bsize += pair.second.size();
    return asize < bsize;
  };
  std::sort(featsup_MSCKF.begin(), featsup_MSCKF.end(), compare_feat);
  // state->xk_minus_x0_ = VecX::Zero(state->get_state_size());

  // Pass them to our MSCKF updater
  // NOTE: if we have more then the max, we select the "best" ones (i.e. max
  // tracks) for this update NOTE: this should only really be used if you want
  // to track a lot of features, or have limited computational resources
  state->setup_matrix_buffer();
  if ((int)featsup_MSCKF.size() > state->options.max_msckf_in_update)
    featsup_MSCKF.erase(featsup_MSCKF.begin(),
                        featsup_MSCKF.end() -
                            state->options.max_msckf_in_update);
  updaterMSCKF->update(state, featsup_MSCKF);
  rT5 = boost::posix_time::microsec_clock::local_time();

  // Perform SLAM delay init and update
  // NOTE: that we provide the option here to do a *sequential* update
  // NOTE: this will be a lot faster but won't be as accurate.

  // Do the update
  updaterSLAM->update(state, feats_slam_UPDATE);

  rT6 = boost::posix_time::microsec_clock::local_time();
  updaterSLAM->delayed_init(state, feats_slam_DELAYED);
  rT7 = boost::posix_time::microsec_clock::local_time();
  StateHelper::initialize_slam_in_U(state);
  rT8 = boost::posix_time::microsec_clock::local_time();
  StateHelper::update_llt(state);
  state->clear(true);

  //===================================================================================
  // Update our visualization feature set, and clean up the old features
  //===================================================================================
  // Re-triangulate all current tracks in the current frame
  if (message.sensor_ids.at(0) == 0) {
    // Clear the MSCKF features only on the base camera
    // Thus we should be able to visualize the other unique camera stream
    // MSCKF features as they will also be appended to the vector
    good_features_MSCKF.clear();
  }

  // Save all the MSCKF features used in the update
  for (auto const &feat : featsup_MSCKF) {
    good_features_MSCKF.push_back(feat->p_FinG);
    feat->to_delete = true;
  }

  //===================================================================================
  // Cleanup, marginalize out what we don't need any more...
  //===================================================================================

  // Remove features that where used for the update from our extractors at the
  // last timestep This allows for measurements to be used in the future if they
  // failed to be used this time Note we need to do this before we feed a new
  // image, as we want all new measurements to NOT be deleted
  trackFEATS->get_feature_database()->cleanup();
  if (trackARUCO != nullptr) {
    trackARUCO->get_feature_database()->cleanup();
  }

  rT9 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  // Debug info, and stats tracking
  //===================================================================================

  // Get timing statitics information
  double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
  double time_prop = (rT3 - rT2).total_microseconds() * 1e-6;
  double time_marg = (rT4 - rT3).total_microseconds() * 1e-6;
  double time_msckf = (rT5 - rT4).total_microseconds() * 1e-6;
  double time_slam_update = (rT6 - rT5).total_microseconds() * 1e-6;
  double time_slam_delay = (rT7 - rT6).total_microseconds() * 1e-6;
  double time_init = (rT8 - rT7).total_microseconds() * 1e-6;
  double time_qr = (state->rT81 - rT8).total_microseconds() * 1e-6;
  double time_backsub = (state->rT82 - state->rT81).total_microseconds() * 1e-6;
  double time_retri = (rT9 - state->rT82).total_microseconds() * 1e-6;
  if (time_qr < 0) {
    time_qr = time_backsub = 0;
    time_retri = (rT9 - rT8).total_microseconds() * 1e-6;
  }

  double time_total = (rT9 - rT1).total_microseconds() * 1e-6;

  // Timing information
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for propagation\n" RESET, time_prop);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for marg (%d clones in state)\n" RESET,
              time_marg, (int)state->clones_IMU.size());
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for MSCKF update (%d feats)\n" RESET,
              time_msckf, (int)featsup_MSCKF.size());
  if (state->options.max_slam_features > 0) {
    PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for SLAM update (%d feats)\n" RESET,
                time_slam_update, (int)state->features_SLAM.size());
    PRINT_DEBUG(BLUE
                "[TIME]: %.4f seconds for SLAM delayed init (%d feats)\n" RESET,
                time_slam_delay, (int)feats_slam_DELAYED.size());
  }
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for state init\n" RESET, time_init);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for QR\n" RESET, time_qr);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for back sub\n" RESET, time_backsub);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for re-tri\n" RESET, time_retri);

  std::stringstream ss;
  ss << "[TIME]: " << std::setprecision(4) << time_total
     << " seconds for total (camera";
  for (const auto &id : message.sensor_ids) {
    ss << " " << id;
  }
  ss << ")" << std::endl;
  PRINT_DEBUG(BLUE "%s" RESET, ss.str().c_str());

  // Finally if we are saving stats to file, lets save it to file
  if (params.record_timing_information && of_statistics.is_open()) {
    // We want to publish in the IMU clock frame
    // The timestamp in the state will be the last camera time
    double t_ItoC = state->calib_dt_CAMtoIMU->value()(0);
    double timestamp_inI = state->timestamp + t_ItoC;
    // Append to the file
    of_statistics << std::fixed << std::setprecision(15) << timestamp_inI << ","
                  << std::fixed << std::setprecision(5) << time_track << ","
                  << time_prop << "," << time_marg << "," << time_msckf << ",";
    if (state->options.max_slam_features > 0) {
      of_statistics << time_slam_update << "," << time_slam_delay << ",";
    }
    of_statistics << time_init << "," << time_qr << "," << time_backsub << ","
                  << time_retri << "," << time_total << std::endl;
    of_statistics.flush();
  }

  // Update our distance traveled
  if (timelastupdate != -1 &&
      state->clones_IMU.find(timelastupdate) != state->clones_IMU.end()) {
    Eigen::Matrix<DataType, 3, 1> dx =
        state->imu->pos() - state->clones_IMU.at(timelastupdate)->pos();
    distance += dx.norm();
  }
  timelastupdate = message.timestamp;

  // Debug, print our current state
  PRINT_INFO("q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f | dist = "
             "%.2f (meters)\n",
             state->imu->quat()(0), state->imu->quat()(1),
             state->imu->quat()(2), state->imu->quat()(3), state->imu->pos()(0),
             state->imu->pos()(1), state->imu->pos()(2), distance);
  PRINT_INFO("bg = %.4f,%.4f,%.4f | ba = %.4f,%.4f,%.4f\n",
             state->imu->bias_g()(0), state->imu->bias_g()(1),
             state->imu->bias_g()(2), state->imu->bias_a()(0),
             state->imu->bias_a()(1), state->imu->bias_a()(2));

  // Debug for camera imu offset
  if (state->options.do_calib_camera_timeoffset) {
    PRINT_INFO("camera-imu timeoffset = %.5f\n",
               state->calib_dt_CAMtoIMU->value()(0));
  }

  // Debug for camera intrinsics
  if (state->options.do_calib_camera_intrinsics) {
    for (int i = 0; i < state->options.num_cameras; i++) {
      std::shared_ptr<Vec> calib = state->cam_intrinsics.at(i);
      PRINT_INFO(
          "cam%d intrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f\n",
          (int)i, calib->value()(0), calib->value()(1), calib->value()(2),
          calib->value()(3), calib->value()(4), calib->value()(5),
          calib->value()(6), calib->value()(7));
    }
  }

  // Debug for camera extrinsics
  if (state->options.do_calib_camera_pose) {
    for (int i = 0; i < state->options.num_cameras; i++) {
      std::shared_ptr<PoseJPL> calib = state->calib_IMUtoCAM.at(i);
      PRINT_INFO("cam%d extrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f\n",
                 (int)i, calib->quat()(0), calib->quat()(1), calib->quat()(2),
                 calib->quat()(3), calib->pos()(0), calib->pos()(1),
                 calib->pos()(2));
    }
  }
}

void VioManager::initialize_with_gt(Eigen::Matrix<double, 17, 1> imustate) {

  // Initialize the system
  state->imu->set_value(imustate.block(1, 0, 16, 1).cast<DataType>());
  state->imu->set_fej(imustate.block(1, 0, 16, 1).cast<DataType>());

  // Fix the global yaw and position gauge freedoms
  // TODO: Why does this break out simulation consistency metrics?
  std::vector<std::shared_ptr<ov_type::Type>> order = {state->imu};
  MatX diagonal = 0.02 * MatX::Identity(state->imu->size(), state->imu->size());
  diagonal.topRows<3>() = 0.017 * Vec3::Ones();    // q
  diagonal.middleRows<3>(3) = 0.05 * Vec3::Ones(); // p
  diagonal.middleRows<3>(6) = 0.01 * Vec3::Ones(); // v (static)
  StateHelper::set_initial_imu_square_root_covariance(state, diagonal);

  // Set the state time
  state->update_timestamp(imustate(0, 0));
  startup_time = imustate(0, 0);
  is_initialized_vio = true;

  // Cleanup any features older then the initialization time
  trackFEATS->get_feature_database()->cleanup_measurements(state->timestamp);
  if (trackARUCO != nullptr) {
    trackARUCO->get_feature_database()->cleanup_measurements(state->timestamp);
  }

  // Print what we init'ed with
  PRINT_DEBUG(GREEN "[INIT]: INITIALIZED FROM GROUNDTRUTH FILE!!!!!\n" RESET);
  PRINT_DEBUG(GREEN "[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\n" RESET,
              state->imu->quat()(0), state->imu->quat()(1),
              state->imu->quat()(2), state->imu->quat()(3));
  PRINT_DEBUG(GREEN "[INIT]: bias gyro = %.4f, %.4f, %.4f\n" RESET,
              state->imu->bias_g()(0), state->imu->bias_g()(1),
              state->imu->bias_g()(2));
  PRINT_DEBUG(GREEN "[INIT]: velocity = %.4f, %.4f, %.4f\n" RESET,
              state->imu->vel()(0), state->imu->vel()(1), state->imu->vel()(2));
  PRINT_DEBUG(GREEN "[INIT]: bias accel = %.4f, %.4f, %.4f\n" RESET,
              state->imu->bias_a()(0), state->imu->bias_a()(1),
              state->imu->bias_a()(2));
  PRINT_DEBUG(GREEN "[INIT]: position = %.4f, %.4f, %.4f\n" RESET,
              state->imu->pos()(0), state->imu->pos()(1), state->imu->pos()(2));
}

bool VioManager::try_to_initialize(const ov_core::CameraData &message) {

  // Directly return if the initialization thread is running
  // Note that we lock on the queue since we could have finished an update
  // And are using this queue to propagate the state forward. We should wait in
  // this case
  if (thread_init_running) {
    std::lock_guard<std::mutex> lck(camera_queue_init_mtx);
    camera_queue_init.push_back(message.timestamp);
    return false;
  }

  // If the thread was a success, then return success!
  if (thread_init_success) {
    return true;
  }

  // Run the initialization in a second thread so it can go as slow as it
  // desires
  thread_init_running = true;
  std::thread thread([&] {
    // Returns from our initializer
    Eigen::MatrixXd covariance;
    std::vector<std::shared_ptr<ov_type::Type>> order;
    auto init_rT1 = boost::posix_time::microsec_clock::local_time();

    // Try to initialize the system
    // We will wait for a jerk if we do not have the zero velocity update
    // enabled Otherwise we can initialize right away as the zero velocity will
    // handle the stationary case
    bool wait_for_jerk = (updaterZUPT == nullptr);
    // bool success = initializer->initialize(timestamp, covariance, order,
    // state->_imu, wait_for_jerk);
    bool success = initializer->initialize(state, wait_for_jerk);

    // If we have initialized successfully we will set the covariance and state
    // elements as needed
    if (success) {
      startup_time = state->timestamp;
      state->is_initialized = true;

      // Cleanup any features older than the initialization time
      // Also increase the number of features to the desired amount during
      // estimation NOTE: we will split the total number of features over all
      // cameras uniformly
      trackFEATS->get_feature_database()->cleanup_measurements(
          state->timestamp);
      trackFEATS->set_num_features(
          std::floor((DataType)params.num_pts /
                     (DataType)params.state_options.num_cameras));
      if (trackARUCO != nullptr) {
        trackARUCO->get_feature_database()->cleanup_measurements(
            state->timestamp);
      }

      // If we are moving then don't do zero velocity update
      if (state->imu->vel().norm() > params.zupt_max_velocity) {
        has_moved_since_zupt = true;
      }

      // Else we are good to go, print out our stats
      auto init_rT2 = boost::posix_time::microsec_clock::local_time();
      PRINT_INFO(GREEN
                 "[init]: successful initialization in %.4f seconds\n" RESET,
                 (init_rT2 - init_rT1).total_microseconds() * 1e-6);
      PRINT_INFO(GREEN "[init]: orientation = %.4f, %.4f, %.4f, %.4f\n" RESET,
                 state->imu->quat()(0), state->imu->quat()(1),
                 state->imu->quat()(2), state->imu->quat()(3));
      PRINT_INFO(GREEN "[init]: bias gyro = %.4f, %.4f, %.4f\n" RESET,
                 state->imu->bias_g()(0), state->imu->bias_g()(1),
                 state->imu->bias_g()(2));
      PRINT_INFO(GREEN "[init]: velocity = %.4f, %.4f, %.4f\n" RESET,
                 state->imu->vel()(0), state->imu->vel()(1),
                 state->imu->vel()(2));
      PRINT_INFO(GREEN "[init]: bias accel = %.4f, %.4f, %.4f\n" RESET,
                 state->imu->bias_a()(0), state->imu->bias_a()(1),
                 state->imu->bias_a()(2));
      PRINT_INFO(GREEN "[init]: position = %.4f, %.4f, %.4f\n" RESET,
                 state->imu->pos()(0), state->imu->pos()(1),
                 state->imu->pos()(2));

      // Remove any camera times that are order then the initialized time
      // This can happen if the initialization has taken a while to perform
      std::lock_guard<std::mutex> lck(camera_queue_init_mtx);
      std::vector<double> camera_timestamps_to_init;
      for (size_t i = 0; i < camera_queue_init.size(); i++) {
        if (camera_queue_init.at(i) > startup_time) {
          camera_timestamps_to_init.push_back(camera_queue_init.at(i));
        }
      }

      // Now we have initialized we will propagate the state to the current
      // timestep In general this should be ok as long as the initialization
      // didn't take too long to perform Propagating over multiple seconds will
      // become an issue if the initial biases are bad
      size_t clone_rate = (size_t)((float)camera_timestamps_to_init.size() /
                                   (float)params.state_options.max_clone_size) +
                          1;
      for (size_t i = 0; i < camera_timestamps_to_init.size();
           i += clone_rate) {
        propagator->propagate_and_clone(state, camera_timestamps_to_init.at(i));
        StateHelper::marginalize_old_clone(state);
        StateHelper::marginalize(state);
      }
      PRINT_DEBUG(YELLOW "[init]: moved the state forward %.2f seconds\n" RESET,
                  state->timestamp - startup_time);
      thread_init_success = true;
      camera_queue_init.clear();
    } else {
      auto init_rT2 = boost::posix_time::microsec_clock::local_time();
      PRINT_DEBUG(YELLOW
                  "[init]: failed initialization in %.4f seconds\n" RESET,
                  (init_rT2 - init_rT1).total_microseconds() * 1e-6);
      thread_init_success = false;
      std::lock_guard<std::mutex> lck(camera_queue_init_mtx);
      camera_queue_init.clear();
    }

    // Finally, mark that the thread has finished running
    thread_init_running = false;
  });

  // If we are single threaded, then run single threaded
  // Otherwise detach this thread so it runs in the background!
  if (!params.use_multi_threading_subs) {
    thread.join();
  } else {
    thread.detach();
  }
  return false;
}

cv::Mat VioManager::get_historical_viz_image() {

  // Return if not ready yet
  if (state == nullptr || trackFEATS == nullptr)
    return cv::Mat();

  // Build an id-list of what features we should highlight (i.e. SLAM)
  std::vector<size_t> highlighted_ids;
  for (const auto &feat : state->features_SLAM) {
    highlighted_ids.push_back(feat.first);
  }

  // Text we will overlay if needed
  std::string overlay = (did_zupt_update) ? "zvupt" : "";
  overlay = (!is_initialized_vio) ? "init" : overlay;

  // Get the current active tracks
  cv::Mat img_history;
  trackFEATS->display_history(img_history, 255, 255, 0, 255, 255, 255,
                              highlighted_ids, overlay);
  if (trackARUCO != nullptr) {
    trackARUCO->display_history(img_history, 0, 255, 255, 255, 255, 255,
                                highlighted_ids, overlay);
  }

  // Finally return the image
  return img_history;
}

std::vector<Vec3> VioManager::get_features_SLAM() {
  std::vector<Vec3> slam_feats;
  for (auto &f : state->features_SLAM) {
    if ((int)f.first <= 4 * state->options.max_aruco_features)
      continue;
    if (ov_type::LandmarkRepresentation::is_relative_representation(
            f.second->feat_representation)) {
      // Assert that we have an anchor pose for this feature
      assert(f.second->anchor_cam_id != -1);
      const auto anchor_pose = state->cam_pose_buffer.get_buffer_unsafe(
          f.second->anchor_cam_id, f.second->anchor_clone_timestamp);
      // Feature in the global frame
      slam_feats.push_back(anchor_pose.R_GtoC.transpose() *
                               f.second->get_xyz(false) +
                           anchor_pose.p_CinG);
    } else {
      slam_feats.push_back(f.second->get_xyz(false));
    }
  }
  return slam_feats;
}

std::vector<Vec3> VioManager::get_features_ARUCO() {
  std::vector<Vec3> aruco_feats;
  for (auto &f : state->features_SLAM) {
    if ((int)f.first > 4 * state->options.max_aruco_features)
      continue;
    if (ov_type::LandmarkRepresentation::is_relative_representation(
            f.second->feat_representation)) {
      // Assert that we have an anchor pose for this feature
      assert(f.second->anchor_cam_id != -1);
      const auto anchor_pose = state->cam_pose_buffer.get_buffer_unsafe(
          f.second->anchor_cam_id, f.second->anchor_clone_timestamp);
      // Feature in the global frame
      aruco_feats.push_back(anchor_pose.R_GtoC.transpose() *
                                f.second->get_xyz(false) +
                            anchor_pose.p_CinG);
    } else {
      aruco_feats.push_back(f.second->get_xyz(false));
    }
  }
  return aruco_feats;
}
