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





#include "ROSVisualizerHelper.h"

#include "core/VioManager.h"
#include "sim/Simulator.h"
#include "state/State.h"
#include "state/StateHelper.h"

#include "types/PoseJPL.h"

using namespace ov_srvins;
using namespace std;

#if ROS_AVAILABLE == 1
sensor_msgs::PointCloud2
ROSVisualizerHelper::get_ros_pointcloud(const std::vector<Vec3> &feats) {

  // Declare message and sizes
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "global";
  cloud.header.stamp = ros::Time::now();
  cloud.width = 3 * feats.size();
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false; // there may be invalid points

  // Setup pointcloud fields
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(3 * feats.size());

  // Iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

  // Fill our iterators
  for (const auto &pt : feats) {
    *out_x = (float)pt(0);
    ++out_x;
    *out_y = (float)pt(1);
    ++out_y;
    *out_z = (float)pt(2);
    ++out_z;
  }

  return cloud;
}

tf::StampedTransform ROSVisualizerHelper::get_stamped_transform_from_pose(
    const std::shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans) {

  // Need to flip the transform to the IMU frame
  Vec4 q_ItoC = pose->quat();
  Vec3 p_CinI = pose->pos();
  if (flip_trans) {
    p_CinI = -pose->Rot().transpose() * pose->pos();
  }

  // publish our transform on TF
  // NOTE: since we use JPL we have an implicit conversion to Hamilton when we
  // publish NOTE: a rotation from ItoC in JPL has the same xyzw as a CtoI
  // Hamilton rotation
  tf::StampedTransform trans;
  trans.stamp_ = ros::Time::now();
  tf::Quaternion quat(q_ItoC(0), q_ItoC(1), q_ItoC(2), q_ItoC(3));
  trans.setRotation(quat);
  tf::Vector3 orig(p_CinI(0), p_CinI(1), p_CinI(2));
  trans.setOrigin(orig);
  return trans;
}
#endif

#if ROS_AVAILABLE == 2
sensor_msgs::msg::PointCloud2
ROSVisualizerHelper::get_ros_pointcloud(std::shared_ptr<rclcpp::Node> node,
                                        const std::vector<Vec3> &feats) {

  // Declare message and sizes
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "global";
  cloud.header.stamp = node->now();
  cloud.width = 3 * feats.size();
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false; // there may be invalid points

  // Setup pointcloud fields
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(3 * feats.size());

  // Iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

  // Fill our iterators
  for (const auto &pt : feats) {
    *out_x = (float)pt(0);
    ++out_x;
    *out_y = (float)pt(1);
    ++out_y;
    *out_z = (float)pt(2);
    ++out_z;
  }

  return cloud;
}

geometry_msgs::msg::TransformStamped
ROSVisualizerHelper::get_stamped_transform_from_pose(
    std::shared_ptr<rclcpp::Node> node,
    const std::shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans) {

  // Need to flip the transform to the IMU frame
  Vec4 q_ItoC = pose->quat();
  Vec3 p_CinI = pose->pos();
  if (flip_trans) {
    p_CinI = -pose->Rot().transpose() * pose->pos();
  }

  // publish our transform on TF
  // NOTE: since we use JPL we have an implicit conversion to Hamilton when we
  // publish NOTE: a rotation from ItoC in JPL has the same xyzw as a CtoI
  // Hamilton rotation
  geometry_msgs::msg::TransformStamped trans;
  trans.header.stamp = node->now();
  trans.transform.rotation.x = q_ItoC(0);
  trans.transform.rotation.y = q_ItoC(1);
  trans.transform.rotation.z = q_ItoC(2);
  trans.transform.rotation.w = q_ItoC(3);
  trans.transform.translation.x = p_CinI(0);
  trans.transform.translation.y = p_CinI(1);
  trans.transform.translation.z = p_CinI(2);
  return trans;
}
#endif

void ROSVisualizerHelper::sim_save_total_state_to_file(
    std::shared_ptr<State> state, std::shared_ptr<Simulator> sim,
    std::ofstream &of_state_est, std::ofstream &of_state_std,
    std::ofstream &of_state_gt) {

  // We want to publish in the IMU clock frame
  // The timestamp in the state will be the last camera time
  double t_ItoC = state->calib_dt_CAMtoIMU->value()(0);
  double timestamp_inI = state->timestamp + t_ItoC;

  // If we have our simulator, then save it to our groundtruth file
  if (sim != nullptr) {

    // Note that we get the true time in the IMU clock frame
    // NOTE: we record both the estimate and groundtruth with the same "true"
    // timestamp if we are doing simulation
    Eigen::Matrix<double, 17, 1> state_gt;
    timestamp_inI =
        state->timestamp + sim->get_true_parameters().calib_camimu_dt;
    if (sim->get_state(timestamp_inI, state_gt)) {
      // STATE: write current true state
      of_state_gt.precision(5);
      of_state_gt.setf(std::ios::fixed, std::ios::floatfield);
      of_state_gt << state_gt(0) << " ";
      of_state_gt.precision(6);
      of_state_gt << state_gt(1) << " " << state_gt(2) << " " << state_gt(3)
                  << " " << state_gt(4) << " ";
      of_state_gt << state_gt(5) << " " << state_gt(6) << " " << state_gt(7)
                  << " ";
      of_state_gt << state_gt(8) << " " << state_gt(9) << " " << state_gt(10)
                  << " ";
      of_state_gt << state_gt(11) << " " << state_gt(12) << " " << state_gt(13)
                  << " ";
      of_state_gt << state_gt(14) << " " << state_gt(15) << " " << state_gt(16)
                  << " ";

      // TIMEOFF: Get the current true time offset
      of_state_gt.precision(7);
      of_state_gt << sim->get_true_parameters().calib_camimu_dt << " ";
      of_state_gt.precision(0);
      of_state_gt << state->options.num_cameras << " ";
      of_state_gt.precision(6);

      // CALIBRATION: Write the camera values to file
      assert(state->options.num_cameras ==
             sim->get_true_parameters().state_options.num_cameras);
      for (int i = 0; i < state->options.num_cameras; i++) {
        // Intrinsics values
        of_state_gt
            << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(
                   0)
            << " "
            << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(
                   1)
            << " "
            << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(
                   2)
            << " "
            << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(
                   3)
            << " ";
        of_state_gt
            << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(
                   4)
            << " "
            << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(
                   5)
            << " "
            << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(
                   6)
            << " "
            << sim->get_true_parameters().camera_intrinsics.at(i)->get_value()(
                   7)
            << " ";
        // Rotation and position
        of_state_gt << sim->get_true_parameters().camera_extrinsics.at(i)(0)
                    << " "
                    << sim->get_true_parameters().camera_extrinsics.at(i)(1)
                    << " "
                    << sim->get_true_parameters().camera_extrinsics.at(i)(2)
                    << " "
                    << sim->get_true_parameters().camera_extrinsics.at(i)(3)
                    << " ";
        of_state_gt << sim->get_true_parameters().camera_extrinsics.at(i)(4)
                    << " "
                    << sim->get_true_parameters().camera_extrinsics.at(i)(5)
                    << " "
                    << sim->get_true_parameters().camera_extrinsics.at(i)(6)
                    << " ";
      }

      // New line
      of_state_gt << endl;
    }
  }

  //==========================================================================
  //==========================================================================
  //==========================================================================

  // STATE: Write the current state to file
  of_state_est.precision(5);
  of_state_est.setf(std::ios::fixed, std::ios::floatfield);
  of_state_est << timestamp_inI << " ";
  of_state_est.precision(6);
  of_state_est << state->imu->quat()(0) << " " << state->imu->quat()(1) << " "
               << state->imu->quat()(2) << " " << state->imu->quat()(3) << " ";
  of_state_est << state->imu->pos()(0) << " " << state->imu->pos()(1) << " "
               << state->imu->pos()(2) << " ";
  of_state_est << state->imu->vel()(0) << " " << state->imu->vel()(1) << " "
               << state->imu->vel()(2) << " ";
  of_state_est << state->imu->bias_g()(0) << " " << state->imu->bias_g()(1)
               << " " << state->imu->bias_g()(2) << " ";
  of_state_est << state->imu->bias_a()(0) << " " << state->imu->bias_a()(1)
               << " " << state->imu->bias_a()(2) << " ";

  // STATE: Write current uncertainty to file
  of_state_std.precision(5);
  of_state_std.setf(std::ios::fixed, std::ios::floatfield);
  of_state_std << timestamp_inI << " ";
  of_state_std.precision(6);
  MatX cov_imu = StateHelper::get_marginal_covariance(state, {state->imu});
  of_state_std << std::sqrt(cov_imu(0, 0)) << " " << std::sqrt(cov_imu(1, 1))
               << " " << std::sqrt(cov_imu(2, 2)) << " ";
  of_state_std << std::sqrt(cov_imu(0, 0)) << " " << std::sqrt(cov_imu(1, 1))
               << " " << std::sqrt(cov_imu(2, 2)) << " ";
  of_state_std << std::sqrt(cov_imu(0, 0)) << " " << std::sqrt(cov_imu(1, 1))
               << " " << std::sqrt(cov_imu(2, 2)) << " ";
  of_state_std << std::sqrt(cov_imu(0, 0)) << " " << std::sqrt(cov_imu(1, 1))
               << " " << std::sqrt(cov_imu(2, 2)) << " ";
  of_state_std << std::sqrt(cov_imu(0, 0)) << " " << std::sqrt(cov_imu(1, 1))
               << " " << std::sqrt(cov_imu(2, 2)) << " ";

  // TIMEOFF: Get the current estimate time offset
  of_state_est.precision(7);
  of_state_est << state->calib_dt_CAMtoIMU->value()(0) << " ";
  of_state_est.precision(0);
  of_state_est << state->options.num_cameras << " ";
  of_state_est.precision(6);

  // TIMEOFF: Get the current std values
  if (state->options.do_calib_camera_timeoffset) {
    MatX cov_timeoffset =
        StateHelper::get_marginal_covariance(state, {state->calib_dt_CAMtoIMU});

    of_state_std << std::sqrt(cov_timeoffset(0, 0)) << " ";
  } else {
    of_state_std << 0.0 << " ";
  }
  of_state_std.precision(0);
  of_state_std << state->options.num_cameras << " ";
  of_state_std.precision(6);

  // CALIBRATION: Write the camera values to file
  for (int i = 0; i < state->options.num_cameras; i++) {
    // Intrinsics values
    of_state_est << state->cam_intrinsics.at(i)->value()(0) << " "
                 << state->cam_intrinsics.at(i)->value()(1) << " "
                 << state->cam_intrinsics.at(i)->value()(2) << " "
                 << state->cam_intrinsics.at(i)->value()(3) << " ";
    of_state_est << state->cam_intrinsics.at(i)->value()(4) << " "
                 << state->cam_intrinsics.at(i)->value()(5) << " "
                 << state->cam_intrinsics.at(i)->value()(6) << " "
                 << state->cam_intrinsics.at(i)->value()(7) << " ";
    // Rotation and position
    of_state_est << state->calib_IMUtoCAM.at(i)->value()(0) << " "
                 << state->calib_IMUtoCAM.at(i)->value()(1) << " "
                 << state->calib_IMUtoCAM.at(i)->value()(2) << " "
                 << state->calib_IMUtoCAM.at(i)->value()(3) << " ";
    of_state_est << state->calib_IMUtoCAM.at(i)->value()(4) << " "
                 << state->calib_IMUtoCAM.at(i)->value()(5) << " "
                 << state->calib_IMUtoCAM.at(i)->value()(6) << " ";
    // Covariance
    if (state->options.do_calib_camera_intrinsics) {
      MatX cov = StateHelper::get_marginal_covariance(
          state, {state->cam_intrinsics.at(i)});
      of_state_std << std::sqrt(cov(0, 0)) << " " << std::sqrt(cov(1, 1)) << " "
                   << std::sqrt(cov(2, 2)) << " " << std::sqrt(cov(3, 3))
                   << " ";
      of_state_std << std::sqrt(cov(4, 4)) << " " << std::sqrt(cov(5, 5)) << " "
                   << std::sqrt(cov(6, 6)) << " " << std::sqrt(cov(7, 7))
                   << " ";
    } else {
      of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " ";
      of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " ";
    }
    if (state->options.do_calib_camera_pose) {
      MatX cov = StateHelper::get_marginal_covariance(
          state, {state->calib_IMUtoCAM.at(i)});
      of_state_std << std::sqrt(cov(0, 0)) << " " << std::sqrt(cov(1, 1)) << " "
                   << std::sqrt(cov(2, 2)) << " ";
      of_state_std << std::sqrt(cov(3, 3)) << " " << std::sqrt(cov(4, 4)) << " "
                   << std::sqrt(cov(5, 5)) << " ";
    } else {
      of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " ";
      of_state_std << 0.0 << " " << 0.0 << " " << 0.0 << " ";
    }
  }

  // Done with the estimates!
  of_state_est << endl;
  of_state_std << endl;
}
