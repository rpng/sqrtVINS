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





#ifndef OV_SRVINS_INERTIALINITIALIZEROPTIONS_H
#define OV_SRVINS_INERTIALINITIALIZEROPTIONS_H

#include <Eigen/Eigen>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"
#include "feat/FeatureInitializerOptions.h"
#include "track/TrackBase.h"
#include "types/LandmarkRepresentation.h"
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
struct InertialInitializerOptions {

  // INITIALIZATION ============================
  /// Amount of time we will initialize over (seconds)
  double init_window_time = 1.0;

  /// Max disparity we will consider the unit to be stationary
  double init_max_disparity = 1.0;

  /// Number of features we should try to track
  int init_max_features = 50;

  /// If we should perform dynamic initialization
  bool init_dyn_use = false;

  /// Max number of MLE iterations for dynamic initialization
  int init_dyn_mle_max_iter = 20;

  /// Number of poses to use during initialization (max should be cam freq *
  /// window)
  int init_dyn_num_pose = 5;

  /// Minimum degrees we need to rotate before we try to init (sum of norm)
  double init_dyn_min_deg = 45.0;

  /// Minimum reciprocal condition number acceptable for our covariance recovery
  /// (min_sigma / max_sigma < sqrt(min_reciprocal_condition_number))
  double init_dyn_min_rec_cond = 1e-15;

  /// Initial IMU gyroscope bias values for dynamic initialization (will be
  /// optimized)
  Vec3 init_dyn_bias_g = Vec3::Zero();

  /// Initial IMU accelerometer bias values for dynamic initialization (will be
  /// optimized)
  Vec3 init_dyn_bias_a = Vec3::Zero();

  /// To account for unstable timestamps, and default openvins clean measuremnt
  /// logic, which clean the timestamp exact at the oldest time
  double init_window_offset = 0.001;

  /// Gravity refiner params
  // Maximal number of iteration for gravity refinement
  int init_grav_opt_max_iter = 20;
  // LM optimizauiton initial lambda
  double init_grav_opt_init_lambda = 1.0;
  // Convergence threshold for optimization
  double init_grav_opt_converge_thres = 1e-6;
  // Decay ratio for lambda in LM optimization
  double init_grav_opt_lambda_decay = 0.5;

  /// Initialization prior covariance
  double init_prior_q = 0.1;
  double init_prior_p = 0.0;
  double init_prior_v = 0.5;
  double init_prior_bg = 0.1;
  double init_prior_ba = 0.1;

  double init_prior_t = 0.001;
  double init_prior_qc = 0.02;
  double init_prior_pc = 0.01;
  double init_prior_fc = 1.0;
  double init_prior_dc1 = 0.01; // distortion
  double init_prior_dc2 = 1e-5; // distortion

  /// Initilazation refienemtn parameters
  // Maxmial allow reprojection error
  double init_max_reproj = 1.0;
  // Convergence check for the dx/x
  double init_ba_dx_converge_thres = 1e-6;
  // Convergence check for the dr/r
  double init_ba_res_converge_thres = 1e-6;
  // Minimal number of features in refinement
  int init_min_feat = 30;
  // Maximal number of features in refinement
  int init_max_feat = 50;
  // Huber loss threshold in initialization
  double init_ba_huber_th = 2.0;

  // Maximal number of SLAM features to initialize after initialization
  int init_max_slam = 50;

  /// Logger
  // Record intialization poses
  bool record_init_pose = true;
  // Record intialization time
  bool record_init_timing = true;
  std::string init_poses_log_file_path = "/tmp/init_window_poses.txt";
  std::string init_metadata_log_file_path = "/tmp/init_window_timing.txt";

  // NOISE / CHI2 ============================

  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w = 1.6968e-04;
  double sigma_w2 = sigma_w * sigma_w;

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb = 1.9393e-05;
  double sigma_wb2 = sigma_wb * sigma_wb;

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a = 2.0000e-3;
  double sigma_a2 = sigma_a * sigma_a;

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab = 3.0000e-03;
  double sigma_ab2 = sigma_ab * sigma_ab;

  /// Noise sigma for our raw pixel measurements
  double sigma_pix = 1;

  // STATE DEFAULTS ==========================
  /// Gravity magnitude in the global frame (i.e. should be 9.81 typically)
  double gravity_mag = 9.81;

  /// Number of distinct cameras that we will observe features in
  int num_cameras = 1;

  /// If we should process two cameras are being stereo or binocular. If
  /// binocular, we do monocular feature tracking on each image.
  bool use_stereo = true;

  /// Will half the resolution all tracking image (aruco will be 1/4 instead of
  /// halved if dowsize_aruoc also enabled)
  bool downsample_cameras = false;

  /// Time offset between camera and IMU (t_imu = t_cam + t_off)
  double calib_camimu_dt = 0.0;

  // Recover initial bg during intialization
  bool use_bg_estimator = false;

  // Debug output flags
  bool optimizer_debug = false;
  bool eigenvector_debug = false;
  bool residual_debug = false;
  bool ba_debug = false;

  /// Map between camid and camera intrinsics (fx, fy, cx, cy, d1...d4, cam_w,
  /// cam_h)
  std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>>
      camera_intrinsics;

  /// Map between camid and camera extrinsics (q_ItoC, p_IinC).
  std::map<size_t, VecX> camera_extrinsics;

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
  std::string sim_traj_path = "../ov_data/sim/udel_gore.txt";

  /// We will start simulating after we have moved this much along the b-spline.
  /// This prevents static starts as we init from groundtruth in simulation.
  double sim_distance_threshold = 1.2;

  /// Frequency (Hz) that we will simulate our cameras
  double sim_freq_cam = 10.0;

  /// Frequency (Hz) that we will simulate our inertial measurement unit
  double sim_freq_imu = 400.0;

  /// Feature distance we generate features from (minimum)
  double sim_min_feature_gen_distance = 5;

  /// Feature distance we generate features from (maximum)
  double sim_max_feature_gen_distance = 10;

  // large pertubation error
  double sim_fisheye_min2center = 20;

  // large distort and undistort error, and make the simulation only has
  // features at the edge (too large fov!)
  double sim_fisheye_max2center = 200;

  /**
   * @brief This function will load the non-simulation parameters of the system
   * and print.
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load(std::shared_ptr<ov_core::YamlParser> parser = nullptr);

  /**
   * @brief This function will load print out all initializer settings loaded.
   * This allows for visual checking that everything was loaded properly from
   * ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_initializer(
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
   * @brief This function will load print out all simulated parameters.
   * This allows for visual checking that everything was loaded properly from
   * ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_simulation(
      std::shared_ptr<ov_core::YamlParser> parser = nullptr);

  /**
   * @brief This function will load and print all state parameters (e.g. sensor
   * extrinsics) This allows for visual checking that everything was loaded
   * properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void
  print_and_load_state(std::shared_ptr<ov_core::YamlParser> parser = nullptr);
};

} // namespace ov_srvins

#endif // OV_SRVINS_INERTIALINITIALIZEROPTIONS_H