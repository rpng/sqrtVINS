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




#include "initializer/InertialInitializerOptions.h"

namespace ov_srvins {
void InertialInitializerOptions::print_and_load(
    std::shared_ptr<ov_core::YamlParser> parser) {
  print_and_load_initializer(parser);
  print_and_load_noise(parser);
  print_and_load_state(parser);
}

void InertialInitializerOptions::print_and_load_initializer(
    std::shared_ptr<ov_core::YamlParser> parser) {
  PRINT_DEBUG("INITIALIZATION SETTINGS:\n");
  if (parser != nullptr) {
    parser->parse_config("init_window_time", init_window_time, false);

    parser->parse_config("init_max_disparity", init_max_disparity, false);
    parser->parse_config("init_max_features", init_max_features, false);
    parser->parse_config("init_max_slam", init_max_slam, false);
    parser->parse_config("init_dyn_use", init_dyn_use, false);
    parser->parse_config("init_dyn_mle_max_iter", init_dyn_mle_max_iter, false);

    parser->parse_config("init_dyn_num_pose", init_dyn_num_pose, false);
    parser->parse_config("init_dyn_min_deg", init_dyn_min_deg, false);

    parser->parse_config("init_window_offset", init_window_offset, false);

    parser->parse_config("init_grav_opt_max_iter", init_grav_opt_max_iter,
                         false);
    parser->parse_config("init_grav_opt_init_lambda", init_grav_opt_init_lambda,
                         false);
    parser->parse_config("init_grav_opt_converge_thres",
                         init_grav_opt_converge_thres, false);
    parser->parse_config("init_grav_opt_lambda_decay",
                         init_grav_opt_lambda_decay, false);

    parser->parse_config("init_prior_q", init_prior_q, false);
    parser->parse_config("init_prior_p", init_prior_p, false);
    parser->parse_config("init_prior_v", init_prior_v, false);
    parser->parse_config("init_prior_bg", init_prior_bg, false);
    parser->parse_config("init_prior_ba", init_prior_ba, false);

    parser->parse_config("init_prior_t", init_prior_t, false);
    parser->parse_config("init_prior_qc", init_prior_qc, false);
    parser->parse_config("init_prior_pc", init_prior_pc, false);
    parser->parse_config("init_prior_fc", init_prior_fc, false);
    parser->parse_config("init_prior_dc1", init_prior_dc1, false);
    parser->parse_config("init_prior_dc2", init_prior_dc2, false);

    parser->parse_config("init_max_reproj", init_max_reproj, false);
    parser->parse_config("init_ba_dx_converge_thres", init_ba_dx_converge_thres,
                         false);
    parser->parse_config("init_ba_res_converge_thres",
                         init_ba_res_converge_thres, false);
    parser->parse_config("init_min_feat", init_min_feat, false);
    parser->parse_config("init_max_feat", init_max_feat, false);
    parser->parse_config("init_ba_huber_th", init_ba_huber_th, false);

    parser->parse_config("record_init_pose", record_init_pose, false);
    parser->parse_config("record_init_timing", record_init_timing, false);
    parser->parse_config("init_poses_log_file_path", init_poses_log_file_path,
                         false);
    parser->parse_config("init_metadata_log_file_path",
                         init_metadata_log_file_path, false);

    std::vector<double> bias_g = {0, 0, 0};
    std::vector<double> bias_a = {0, 0, 0};
    parser->parse_config("init_dyn_bias_g", bias_g);
    parser->parse_config("init_dyn_bias_a", bias_a);
    init_dyn_bias_g << bias_g.at(0), bias_g.at(1), bias_g.at(2);
    init_dyn_bias_a << bias_a.at(0), bias_a.at(1), bias_a.at(2);
  }
  PRINT_DEBUG("  - init_window_time: %.2f\n", init_window_time);
  PRINT_DEBUG("  - init_max_disparity: %.2f\n", init_max_disparity);
  PRINT_DEBUG("  - init_max_features: %.2f\n", init_max_features);
  if (init_max_features < 15) {
    PRINT_ERROR(
        RED "number of requested feature tracks to init not enough!!\n" RESET);
    PRINT_ERROR(RED "  init_max_features = %d\n" RESET, init_max_features);
    std::exit(EXIT_FAILURE);
  }

  if (init_max_disparity <= 0.0 && !init_dyn_use) {
    PRINT_ERROR(RED "need to have an DISPARITY threshold for static "
                    "initialization!\n" RESET);
    PRINT_ERROR(RED "  init_max_disparity = %.3f\n" RESET, init_max_disparity);
    PRINT_ERROR(RED "  init_dyn_use = %d\n" RESET, init_dyn_use);
    std::exit(EXIT_FAILURE);
  }
  PRINT_DEBUG("  - init_dyn_use: %d\n", init_dyn_use);
  PRINT_DEBUG("  - init_dyn_mle_max_iter: %d\n", init_dyn_mle_max_iter);
  PRINT_DEBUG("  - init_dyn_num_pose: %d\n", init_dyn_num_pose);
  PRINT_DEBUG("  - init_dyn_min_deg: %.2f\n", init_dyn_min_deg);
  // if (init_dyn_num_pose < 4) {
  //   PRINT_ERROR(RED "number of requested frames to init not enough!!\n"
  //   RESET); PRINT_ERROR(RED "  init_dyn_num_pose = %d (4 min)\n" RESET,
  //   init_dyn_num_pose); std::exit(EXIT_FAILURE);
  // }
  PRINT_DEBUG("  - init_dyn_bias_g: %.2f, %.2f, %.2f\n", init_dyn_bias_g(0),
              init_dyn_bias_g(1), init_dyn_bias_g(2));
  PRINT_DEBUG("  - init_dyn_bias_a: %.2f, %.2f, %.2f\n", init_dyn_bias_a(0),
              init_dyn_bias_a(1), init_dyn_bias_a(2));
}

void InertialInitializerOptions::print_and_load_noise(
    std::shared_ptr<ov_core::YamlParser> parser) {
  PRINT_DEBUG("NOISE PARAMETERS:\n");
  if (parser != nullptr) {
    parser->parse_external("relative_config_imu", "imu0",
                           "gyroscope_noise_density", sigma_w);
    parser->parse_external("relative_config_imu", "imu0",
                           "gyroscope_random_walk", sigma_wb);
    parser->parse_external("relative_config_imu", "imu0",
                           "accelerometer_noise_density", sigma_a);
    parser->parse_external("relative_config_imu", "imu0",
                           "accelerometer_random_walk", sigma_ab);
    parser->parse_config("up_slam_sigma_px", sigma_pix);
  }
  PRINT_DEBUG("  - gyroscope_noise_density: %.6f\n", sigma_w);
  PRINT_DEBUG("  - accelerometer_noise_density: %.5f\n", sigma_a);
  PRINT_DEBUG("  - gyroscope_random_walk: %.7f\n", sigma_wb);
  PRINT_DEBUG("  - accelerometer_random_walk: %.6f\n", sigma_ab);
  PRINT_DEBUG("  - sigma_pix: %.2f\n", sigma_pix);

  // Update noise squares
  sigma_w2 = sigma_w * sigma_w;
  sigma_wb2 = sigma_wb * sigma_wb;
  sigma_a2 = sigma_a * sigma_a;
  sigma_ab2 = sigma_ab * sigma_ab;
}

void InertialInitializerOptions::print_and_load_state(
    std::shared_ptr<ov_core::YamlParser> parser) {
  if (parser != nullptr) {
    parser->parse_config("gravity_mag", gravity_mag);
    parser->parse_config("max_cameras", num_cameras); // might be redundant
    parser->parse_config("use_stereo", use_stereo);
    parser->parse_config("downsample_cameras", downsample_cameras);
    parser->parse_config("use_bg_estimator", use_bg_estimator, false);
    parser->parse_config("optimizer_debug", optimizer_debug, false);
    parser->parse_config("eigenvector_debug", eigenvector_debug, false);
    parser->parse_config("residual_debug", residual_debug, false);
    parser->parse_config("ba_debug", ba_debug, false);
    for (int i = 0; i < num_cameras; i++) {

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

      // Extrinsics
      Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
      parser->parse_external("relative_config_imucam",
                             "cam" + std::to_string(i), "T_imu_cam", T_CtoI);

      // Load these into our state
      Eigen::Matrix<double, 7, 1> cam_eigen;
      cam_eigen.block(0, 0, 4, 1) =
          ov_core::rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
      cam_eigen.block(4, 0, 3, 1) =
          -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);

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
  }
  PRINT_DEBUG("STATE PARAMETERS:\n");
  PRINT_DEBUG("  - gravity_mag: %.4f\n", gravity_mag);
  PRINT_DEBUG("  - gravity: %.3f, %.3f, %.3f\n", 0.0, 0.0, gravity_mag);
  PRINT_DEBUG("  - num_cameras: %d\n", num_cameras);
  PRINT_DEBUG("  - use_stereo: %d\n", use_stereo);
  PRINT_DEBUG("  - downsize cameras: %d\n", downsample_cameras);
  if (num_cameras != (int)camera_intrinsics.size() ||
      num_cameras != (int)camera_extrinsics.size()) {
    PRINT_ERROR(
        RED "[SIM]: camera calib size does not match max cameras...\n" RESET);
    PRINT_ERROR(RED "[SIM]: got %d but expected %d max cameras "
                    "(camera_intrinsics)\n" RESET,
                (int)camera_intrinsics.size(), num_cameras);
    PRINT_ERROR(RED "[SIM]: got %d but expected %d max cameras "
                    "(camera_extrinsics)\n" RESET,
                (int)camera_extrinsics.size(), num_cameras);
    std::exit(EXIT_FAILURE);
  }
  PRINT_DEBUG("  - calib_camimu_dt: %.4f\n", calib_camimu_dt);
  for (int n = 0; n < num_cameras; n++) {
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
    Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
    T_CtoI.block(0, 0, 3, 3) =
        ov_core::quat_2_Rot(
            camera_extrinsics.at(n).block(0, 0, 4, 1).cast<double>())
            .transpose();
    T_CtoI.block(0, 3, 3, 1) =
        -T_CtoI.block(0, 0, 3, 3) *
        camera_extrinsics.at(n).block(4, 0, 3, 1).cast<double>();
    ss << "T_C" << n << "toI:" << std::endl << T_CtoI << std::endl << std::endl;
    PRINT_DEBUG(ss.str().c_str());
  }
}

void InertialInitializerOptions::print_and_load_simulation(
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
    parser->parse_config("sim_fisheye_min2center", sim_fisheye_min2center,
                         false);
    parser->parse_config("sim_fisheye_max2center", sim_fisheye_max2center,
                         false);
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
  PRINT_DEBUG("  - fisheye min2center: %.2f\n", sim_fisheye_min2center);
  PRINT_DEBUG("  - fisheye max2center: %.2f\n", sim_fisheye_max2center);
}

} // namespace ov_srvins