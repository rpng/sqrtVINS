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





#ifndef OV_SRVINS_VIOMANAGEROPTIONS_H
#define OV_SRVINS_VIOMANAGEROPTIONS_H

#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "state/State.h"

#include "state/StateOptions.h"
#include "update/UpdaterOptions.h"
#include "utils/NoiseManager.h"

#include "initializer/InertialInitializerOptions.h"

#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"
#include "feat/FeatureInitializerOptions.h"
#include "track/TrackBase.h"
#include "utils/DataType.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

namespace ov_srvins {

/**
 * @brief Struct which stores all options needed for state estimation.
 *
 * This is broken into a few different parts: estimator, trackers, and
 * simulation. If you are going to add a parameter here you will need to add it
 * to the parsers. You will also need to add it to the print statement at the
 * bottom of each.
 */
struct VioManagerOptions {
  /**
   * @brief This function will load the non-simulation parameters of the system
   * and print.
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load(std::shared_ptr<ov_core::YamlParser> parser = nullptr);

  /**
   * @brief This function will load print out all estimator settings loaded.
   * This allows for visual checking that everything was loaded properly from
   * ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_estimator(
      std::shared_ptr<ov_core::YamlParser> parser = nullptr);

  /**
   * @brief This function will load print out all noise parameters loaded.
   * This allows for visual checking that everything was loaded properly from
   * ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void
  print_and_load_noise(std::shared_ptr<ov_core::YamlParser> parser = nullptr);

  /**
   * @brief This function will load and print all state parameters (e.g. sensor
   * extrinsics) This allows for visual checking that everything was loaded
   * properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void
  print_and_load_state(std::shared_ptr<ov_core::YamlParser> parser = nullptr);

  /**
   * @brief This function will load print out all parameters related to visual
   * tracking This allows for visual checking that everything was loaded
   * properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_trackers(
      std::shared_ptr<ov_core::YamlParser> parser = nullptr);

  /**
   * @brief This function will load print out all simulated parameters.
   * This allows for visual checking that everything was loaded properly from
   * ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_simulation(
      std::shared_ptr<ov_core::YamlParser> parser = nullptr);

  // ESTIMATOR ===============================
  /// Core state options (e.g. number of cameras, use fej, stereo, what
  /// calibration to enable etc)
  StateOptions state_options;

  /// Our state initialization options (e.g. window size, num features, if we
  /// should get the calibration)
  ov_srvins::InertialInitializerOptions init_options;

  /// Delay, in seconds, that we should wait from init before we start
  /// estimating SLAM features
  DataType dt_slam_delay = 2.0;

  /// If we should try to use zero velocity update
  bool try_zupt = false;

  /// Max velocity we will consider to try to do a zupt (i.e. if above this,
  /// don't do zupt)
  DataType zupt_max_velocity = 1.0;

  /// Multiplier of our zupt measurement IMU noise matrix (default should
  /// be 1.0)
  DataType zupt_noise_multiplier = 1.0;

  /// Max disparity we will consider to try to do a zupt (i.e. if above this,
  /// don't do zupt)
  DataType zupt_max_disparity = 1.0;

  /// If we should only use the zupt at the very beginning static initialization
  /// phase
  bool zupt_only_at_beginning = false;

  /// If we should record the timing performance to file
  bool record_timing_information = false;

  /// The path to the file we will record the timing information into
  std::string record_timing_filepath = "OV_SRVINS_timing.txt";

  // NOISE / CHI2 ============================

  /// IMU noise (gyroscope and accelerometer)
  NoiseManager imu_noises;

  /// Update options for MSCKF features (pixel noise and chi2 multiplier)
  UpdaterOptions msckf_options;

  /// Update options for SLAM features (pixel noise and chi2 multiplier)
  UpdaterOptions slam_options;

  /// Update options for ARUCO features (pixel noise and chi2 multiplier)
  UpdaterOptions aruco_options;

  /// Update options for zero velocity (chi2 multiplier)
  UpdaterOptions zupt_options;

  // STATE DEFAULTS ==========================

  /// Gravity magnitude in the global frame (i.e. should be 9.81 typically)
  DataType gravity_mag = 9.81;

  /// Time offset between camera and IMU.
  DataType calib_camimu_dt = 0.0;

  /// Map between camid and camera intrinsics (fx, fy, cx, cy, d1...d4, cam_w,
  /// cam_h)
  std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>>
      camera_intrinsics;

  /// Map between camid and camera extrinsics (q_ItoC, p_IinC).
  std::map<size_t, VecX> camera_extrinsics;

  /// If we should try to load a mask and use it to reject invalid features
  bool use_mask = false;

  /// Mask images for each camera
  std::map<size_t, cv::Mat> masks;

  // TRACKERS ===============================

  /// If we should process two cameras are being stereo or binocular. If
  /// binocular, we do monocular feature tracking on each image.
  bool use_stereo = true;

  /// If we should use KLT tracking, or descriptor matcher
  bool use_klt = true;

  /// If should extract aruco tags and estimate them
  bool use_aruco = true;

  /// Will half the resolution of the aruco tag image (will be faster)
  bool downsize_aruco = true;

  /// Will half the resolution all tracking image (aruco will be 1/4 instead of
  /// halved if dowsize_aruoc also enabled)
  bool downsample_cameras = false;

  /// Threads our front-end should try to use (opencv uses this also)
  int num_opencv_threads = 4;

  /// If our ROS image publisher should be async (if sim this should be no!)
  bool use_multi_threading_pubs = true;

  /// If our ROS subscriber callbacks should be async (if sim and serial then
  /// this should be no!)
  bool use_multi_threading_subs = false;

  /// The number of points we should extract and track in *each* image frame.
  /// This highly effects the computation required for tracking.
  int num_pts = 150;

  /// Fast extraction threshold
  int fast_threshold = 20;

  /// Number of grids we should split column-wise to do feature extraction in
  int grid_x = 5;

  /// Number of grids we should split row-wise to do feature extraction in
  int grid_y = 5;

  /// Will check after doing KLT track and remove any features closer than this
  int min_px_dist = 10;

  /// RANSAC threshold for our KLT tracker
  double ransac_th = 1.0;

  /// What type of pre-processing histogram method should be applied to images
  ov_core::TrackBase::HistogramMethod histogram_method =
      ov_core::TrackBase::HistogramMethod::HISTOGRAM;

  /// KNN ration between top two descriptor matcher which is required to be a
  /// good match
  DataType knn_ratio = 0.85;

  /// Frequency we want to track images at (higher freq ones will be dropped)
  DataType track_frequency = 20.0;

  /// Parameters used by our feature initialize / triangulator
  ov_core::FeatureInitializerOptions featinit_options;

  // SIMULATOR ===============================

  /// Seed for initial states (i.e. random feature 3d positions in the generated
  /// map)
  int sim_seed_state_init = 0;

  /// Seed for calibration perturbations. Change this to perturb by different
  /// random values if perturbations are enabled.
  int sim_seed_preturb = 0;

  /// Measurement noise seed. This should be incremented for each run in the
  /// Monte-Carlo simulation to generate the same true measurements, but
  /// diffferent noise values.
  int sim_seed_measurements = 0;

  /// If we should perturb the calibration that the estimator starts with
  bool sim_do_perturbation = false;

  /// Path to the trajectory we will b-spline and simulate on. Should be
  /// time(s),pos(xyz),ori(xyzw) format.
  std::string sim_traj_path = "src/open_vins/ov_data/sim/udel_gore.txt";

  /// We will start simulating after we have moved this much along the b-spline.
  /// This prevents static starts as we init from groundtruth in simulation.
  DataType sim_distance_threshold = 1.2;

  /// Frequency (Hz) that we will simulate our cameras
  DataType sim_freq_cam = 10.0;

  /// Frequency (Hz) that we will simulate our inertial measurement unit
  DataType sim_freq_imu = 400.0;

  /// Feature distance we generate features from (minimum)
  DataType sim_min_feature_gen_distance = 5;

  /// Feature distance we generate features from (maximum)
  DataType sim_max_feature_gen_distance = 10;
};

} // namespace ov_srvins

#endif // OV_SRVINS_VIOMANAGEROPTIONS_H