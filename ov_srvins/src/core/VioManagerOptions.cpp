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





#include "VioManagerOptions.h"

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

void VioManagerOptions::print_and_load(
    std::shared_ptr<ov_core::YamlParser> parser) {
  print_and_load_estimator(parser);
  print_and_load_noise(parser);
  print_and_load_state(parser);
  print_and_load_trackers(parser);
}

void VioManagerOptions::print_and_load_estimator(
    std::shared_ptr<ov_core::YamlParser> parser) {
  PRINT_DEBUG("ESTIMATOR PARAMETERS:\n");
  state_options.print(parser);
  init_options.print_and_load(parser);
  if (parser != nullptr) {
    parser->parse_config("dt_slam_delay", dt_slam_delay);
    parser->parse_config("try_zupt", try_zupt);
    parser->parse_config("zupt_max_velocity", zupt_max_velocity);
    parser->parse_config("zupt_noise_multiplier", zupt_noise_multiplier);
    parser->parse_config("zupt_max_disparity", zupt_max_disparity);
    parser->parse_config("zupt_only_at_beginning", zupt_only_at_beginning);
    parser->parse_config("record_timing_information",
                         record_timing_information);
    parser->parse_config("record_timing_filepath", record_timing_filepath);
  }
  PRINT_DEBUG("  - dt_slam_delay: %.1f\n", dt_slam_delay);
  PRINT_DEBUG("  - zero_velocity_update: %d\n", try_zupt);
  PRINT_DEBUG("  - zupt_max_velocity: %.2f\n", zupt_max_velocity);
  PRINT_DEBUG("  - zupt_noise_multiplier: %.2f\n", zupt_noise_multiplier);
  PRINT_DEBUG("  - zupt_max_disparity: %.4f\n", zupt_max_disparity);
  PRINT_DEBUG("  - zupt_only_at_beginning?: %d\n", zupt_only_at_beginning);
  PRINT_DEBUG("  - record timing?: %d\n", (int)record_timing_information);
  PRINT_DEBUG("  - record timing filepath: %s\n",
              record_timing_filepath.c_str());
}

void VioManagerOptions::print_and_load_noise(
    std::shared_ptr<ov_core::YamlParser> parser) {
  PRINT_DEBUG("NOISE PARAMETERS:\n");
  if (parser != nullptr) {
    parser->parse_external("relative_config_imu", "imu0",
                           "gyroscope_noise_density", imu_noises.sigma_w);
    parser->parse_external("relative_config_imu", "imu0",
                           "gyroscope_random_walk", imu_noises.sigma_wb);
    parser->parse_external("relative_config_imu", "imu0",
                           "accelerometer_noise_density", imu_noises.sigma_a);
    parser->parse_external("relative_config_imu", "imu0",
                           "accelerometer_random_walk", imu_noises.sigma_ab);
    imu_noises.sigma_w_2 = std::pow(imu_noises.sigma_w, 2);
    imu_noises.sigma_wb_2 = std::pow(imu_noises.sigma_wb, 2);
    imu_noises.sigma_a_2 = std::pow(imu_noises.sigma_a, 2);
    imu_noises.sigma_ab_2 = std::pow(imu_noises.sigma_ab, 2);
  }
  imu_noises.print();
  if (parser != nullptr) {
    DataType sigma_pix, chi2_mult;
    // MSCKF
    parser->parse_config("up_msckf_sigma_px", sigma_pix);
    parser->parse_config("up_msckf_chi2_multipler", chi2_mult);
    msckf_options = UpdaterOptions(chi2_mult, sigma_pix);
    // SLAM
    parser->parse_config("up_slam_sigma_px", sigma_pix);
    parser->parse_config("up_slam_chi2_multipler", chi2_mult);
    slam_options = UpdaterOptions(chi2_mult, sigma_pix);
    // ARUCO
    parser->parse_config("up_aruco_sigma_px", aruco_options.sigma_pix);
    parser->parse_config("up_aruco_chi2_multipler",
                         aruco_options.chi2_multipler);
    aruco_options = UpdaterOptions(chi2_mult, sigma_pix);
    // ZUPT
    parser->parse_config("zupt_chi2_multipler", zupt_options.chi2_multipler);
  }
  PRINT_DEBUG("  Updater MSCKF Feats:\n");
  msckf_options.print();
  PRINT_DEBUG("  Updater SLAM Feats:\n");
  slam_options.print();
  PRINT_DEBUG("  Updater ARUCO Tags:\n");
  aruco_options.print();
  PRINT_DEBUG("  Updater ZUPT:\n");
  zupt_options.print();
}

void VioManagerOptions::print_and_load_state(
    std::shared_ptr<ov_core::YamlParser> parser) {
  if (parser != nullptr) {
    parser->parse_config("gravity_mag", gravity_mag);
    parser->parse_config("max_cameras",
                         state_options.num_cameras); // might be redundant
    parser->parse_config("downsample_cameras",
                         downsample_cameras); // might be redundant
    for (int i = 0; i < state_options.num_cameras; i++) {

      // Time offset (use the first one)
      // TODO: support multiple time offsets between cameras
      if (i == 0) {
        parser->parse_external("relative_config_imucam",
                               "cam" + std::to_string(i), "timeshift_cam_imu",
                               calib_camimu_dt, false);
      }

      // Distortion model
      std::string dist_model = "radtan";
      parser->parse_external("relative_config_imucam",
                             "cam" + std::to_string(i), "distortion_model",
                             dist_model);

      // Distortion parameters
      std::vector<double> cam_calib1 = {1, 1, 0, 0};
      std::vector<double> cam_calib2 = {0, 0, 0, 0};
      parser->parse_external("relative_config_imucam",
                             "cam" + std::to_string(i), "intrinsics",
                             cam_calib1);
      parser->parse_external("relative_config_imucam",
                             "cam" + std::to_string(i), "distortion_coeffs",
                             cam_calib2);
      VecX cam_calib = VecX::Zero(8);
      cam_calib << cam_calib1.at(0), cam_calib1.at(1), cam_calib1.at(2),
          cam_calib1.at(3), cam_calib2.at(0), cam_calib2.at(1),
          cam_calib2.at(2), cam_calib2.at(3);
      cam_calib(0) /= (downsample_cameras) ? 2.0 : 1.0;
      cam_calib(1) /= (downsample_cameras) ? 2.0 : 1.0;
      cam_calib(2) /= (downsample_cameras) ? 2.0 : 1.0;
      cam_calib(3) /= (downsample_cameras) ? 2.0 : 1.0;

      // FOV / resolution
      std::vector<int> matrix_wh = {1, 1};
      parser->parse_external("relative_config_imucam",
                             "cam" + std::to_string(i), "resolution",
                             matrix_wh);
      matrix_wh.at(0) /= (downsample_cameras) ? 2.0 : 1.0;
      matrix_wh.at(1) /= (downsample_cameras) ? 2.0 : 1.0;
      std::pair<int, int> wh(matrix_wh.at(0), matrix_wh.at(1));

      // Extrinsics
      Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
      parser->parse_external("relative_config_imucam",
                             "cam" + std::to_string(i), "T_imu_cam", T_CtoI);

      // Load these into our state
      Eigen::Matrix<double, 7, 1> cam_eigen;
      cam_eigen.block(0, 0, 4, 1) =
          ov_core::rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
      cam_eigen.block(4, 0, 3, 1) =
          -(T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1));

      // Create intrinsics model
      if (dist_model == "equidistant") {
        camera_intrinsics.insert({i, std::make_shared<ov_core::CamEqui>(
                                         matrix_wh.at(0), matrix_wh.at(1))});
        camera_intrinsics.at(i)->set_value(cam_calib);
      } else {
        camera_intrinsics.insert({i, std::make_shared<ov_core::CamRadtan>(
                                         matrix_wh.at(0), matrix_wh.at(1))});
        camera_intrinsics.at(i)->set_value(cam_calib);
      }
      camera_extrinsics.insert({i, cam_eigen.cast<DataType>()});
    }
    parser->parse_config("use_mask", use_mask);
    if (use_mask) {
      for (int i = 0; i < state_options.num_cameras; i++) {
        std::string mask_path;
        std::string mask_node = "mask" + std::to_string(i);
        parser->parse_config(mask_node, mask_path);
        std::string total_mask_path = parser->get_config_folder() + mask_path;
        if (!boost::filesystem::exists(total_mask_path)) {
          PRINT_ERROR(RED "VioManager(): invalid mask path:\n" RESET);
          PRINT_ERROR(RED "\t- mask%d - %s\n" RESET, i,
                      total_mask_path.c_str());
          std::exit(EXIT_FAILURE);
        }
        masks.insert({i, cv::imread(total_mask_path, cv::IMREAD_GRAYSCALE)});
      }
    }
  }
  PRINT_DEBUG("STATE PARAMETERS:\n");
  PRINT_DEBUG("  - gravity_mag: %.4f\n", gravity_mag);
  PRINT_DEBUG("  - gravity: %.3f, %.3f, %.3f\n", 0.0, 0.0, gravity_mag);
  PRINT_DEBUG("  - camera masks?: %d\n", use_mask);
  if (state_options.num_cameras != (int)camera_intrinsics.size() ||
      state_options.num_cameras != (int)camera_extrinsics.size()) {
    PRINT_ERROR(
        RED "[SIM]: camera calib size does not match max cameras...\n" RESET);
    PRINT_ERROR(RED "[SIM]: got %d but expected %d max cameras "
                    "(camera_intrinsics)\n" RESET,
                (int)camera_intrinsics.size(), state_options.num_cameras);
    PRINT_ERROR(RED "[SIM]: got %d but expected %d max cameras "
                    "(camera_extrinsics)\n" RESET,
                (int)camera_extrinsics.size(), state_options.num_cameras);
    std::exit(EXIT_FAILURE);
  }
  PRINT_DEBUG("  - calib_camimu_dt: %.4f\n", calib_camimu_dt);
  for (int n = 0; n < state_options.num_cameras; n++) {
    std::stringstream ss;
    ss << "cam_" << n << "_fisheye:"
       << (std::dynamic_pointer_cast<ov_core::CamEqui>(
               camera_intrinsics.at(n)) != nullptr)
       << std::endl;
    ss << "cam_" << n << "_wh:" << std::endl
       << camera_intrinsics.at(n)->w() << " x " << camera_intrinsics.at(n)->h()
       << std::endl;
    ss << "cam_" << n << "_intrinsic(0:3):" << std::endl
       << camera_intrinsics.at(n)->get_value().block(0, 0, 4, 1).transpose()
       << std::endl;
    ss << "cam_" << n << "_intrinsic(4:7):" << std::endl
       << camera_intrinsics.at(n)->get_value().block(4, 0, 4, 1).transpose()
       << std::endl;
    ss << "cam_" << n << "_extrinsic(0:3):" << std::endl
       << camera_extrinsics.at(n).block(0, 0, 4, 1).transpose() << std::endl;
    ss << "cam_" << n << "_extrinsic(4:6):" << std::endl
       << camera_extrinsics.at(n).block(4, 0, 3, 1).transpose() << std::endl;
    Mat4 T_CtoI = Mat4::Identity();
    T_CtoI.block(0, 0, 3, 3) =
        ov_core::quat_2_Rot(camera_extrinsics.at(n).block(0, 0, 4, 1))
            .transpose();
    T_CtoI.block(0, 3, 3, 1) =
        -T_CtoI.block(0, 0, 3, 3) * camera_extrinsics.at(n).block(4, 0, 3, 1);
    ss << "T_C" << n << "toI:" << std::endl << T_CtoI << std::endl << std::endl;
    PRINT_DEBUG(ss.str().c_str());
  }
}

void VioManagerOptions::print_and_load_trackers(
    std::shared_ptr<ov_core::YamlParser> parser) {
  if (parser != nullptr) {
    parser->parse_config("use_stereo", use_stereo);
    parser->parse_config("use_klt", use_klt);
    parser->parse_config("use_aruco", use_aruco);
    parser->parse_config("downsize_aruco", downsize_aruco);
    parser->parse_config("downsample_cameras", downsample_cameras);
    parser->parse_config("num_opencv_threads", num_opencv_threads);
    parser->parse_config("multi_threading_pubs", use_multi_threading_pubs,
                         false);
    parser->parse_config("multi_threading_subs", use_multi_threading_subs,
                         false);
    parser->parse_config("num_pts", num_pts);
    parser->parse_config("fast_threshold", fast_threshold);
    parser->parse_config("grid_x", grid_x);
    parser->parse_config("grid_y", grid_y);
    parser->parse_config("min_px_dist", min_px_dist);
    parser->parse_config("ransac_th", ransac_th, false);
    std::string histogram_method_str = "HISTOGRAM";
    parser->parse_config("histogram_method", histogram_method_str);
    if (histogram_method_str == "NONE") {
      histogram_method = ov_core::TrackBase::NONE;
    } else if (histogram_method_str == "HISTOGRAM") {
      histogram_method = ov_core::TrackBase::HISTOGRAM;
    } else if (histogram_method_str == "CLAHE") {
      histogram_method = ov_core::TrackBase::CLAHE;
    } else {
      printf(RED "VioManager(): invalid feature histogram specified:\n" RESET);
      printf(RED "\t- NONE\n" RESET);
      printf(RED "\t- HISTOGRAM\n" RESET);
      printf(RED "\t- CLAHE\n" RESET);
      std::exit(EXIT_FAILURE);
    }
    parser->parse_config("knn_ratio", knn_ratio);
    parser->parse_config("track_frequency", track_frequency);
  }
  PRINT_DEBUG("FEATURE TRACKING PARAMETERS:\n");
  PRINT_DEBUG("  - use_stereo: %d\n", use_stereo);
  PRINT_DEBUG("  - use_klt: %d\n", use_klt);
  PRINT_DEBUG("  - use_aruco: %d\n", use_aruco);
  PRINT_DEBUG("  - downsize aruco: %d\n", downsize_aruco);
  PRINT_DEBUG("  - downsize cameras: %d\n", downsample_cameras);
  PRINT_DEBUG("  - num opencv threads: %d\n", num_opencv_threads);
  PRINT_DEBUG("  - use multi-threading pubs: %d\n", use_multi_threading_pubs);
  PRINT_DEBUG("  - use multi-threading subs: %d\n", use_multi_threading_subs);
  PRINT_DEBUG("  - num_pts: %d\n", num_pts);
  PRINT_DEBUG("  - fast threshold: %d\n", fast_threshold);
  PRINT_DEBUG("  - grid X by Y: %d by %d\n", grid_x, grid_y);
  PRINT_DEBUG("  - min px dist: %d\n", min_px_dist);
  PRINT_DEBUG("  - ransac th: %.2f\n", ransac_th);
  PRINT_DEBUG("  - hist method: %d\n", (int)histogram_method);
  PRINT_DEBUG("  - knn ratio: %.3f\n", knn_ratio);
  PRINT_DEBUG("  - track frequency: %.1f\n", track_frequency);
  featinit_options.print(parser);
}

void VioManagerOptions::print_and_load_simulation(
    std::shared_ptr<ov_core::YamlParser> parser) {
  if (parser != nullptr) {
    parser->parse_config("sim_seed_state_init", sim_seed_state_init);
    parser->parse_config("sim_seed_preturb", sim_seed_preturb);
    parser->parse_config("sim_seed_measurements", sim_seed_measurements);
    parser->parse_config("sim_do_perturbation", sim_do_perturbation);
    parser->parse_config("sim_traj_path", sim_traj_path);
    parser->parse_config("sim_distance_threshold", sim_distance_threshold);
    parser->parse_config("sim_freq_cam", sim_freq_cam);
    parser->parse_config("sim_freq_imu", sim_freq_imu);
    parser->parse_config("sim_min_feature_gen_dist",
                         sim_min_feature_gen_distance);
    parser->parse_config("sim_max_feature_gen_dist",
                         sim_max_feature_gen_distance);
  }
  PRINT_DEBUG("SIMULATION PARAMETERS:\n");
  PRINT_WARNING(BOLDRED "  - state init seed: %d \n" RESET,
                sim_seed_state_init);
  PRINT_WARNING(BOLDRED "  - perturb seed: %d \n" RESET, sim_seed_preturb);
  PRINT_WARNING(BOLDRED "  - measurement seed: %d \n" RESET,
                sim_seed_measurements);
  PRINT_WARNING(BOLDRED "  - do perturb?: %d\n" RESET, sim_do_perturbation);
  PRINT_DEBUG("  - traj path: %s\n", sim_traj_path.c_str());
  PRINT_DEBUG("  - dist thresh: %.2f\n", sim_distance_threshold);
  PRINT_DEBUG("  - cam feq: %.2f\n", sim_freq_cam);
  PRINT_DEBUG("  - imu feq: %.2f\n", sim_freq_imu);
  PRINT_DEBUG("  - min feat dist: %.2f\n", sim_min_feature_gen_distance);
  PRINT_DEBUG("  - max feat dist: %.2f\n", sim_max_feature_gen_distance);
}
}; // namespace ov_srvins
