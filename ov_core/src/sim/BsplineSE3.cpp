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





#include "BsplineSE3.h"

using namespace ov_core;

void BsplineSE3::feed_trajectory(std::vector<Eigen::VectorXd> traj_points) {

  // Find the average frequency to use as our uniform timesteps
  double sumdt = 0;
  for (size_t i = 0; i < traj_points.size() - 1; i++) {
    sumdt += traj_points.at(i + 1)(0) - traj_points.at(i)(0);
  }
  dt = sumdt / (traj_points.size() - 1);
  dt = (dt < 0.05) ? 0.05 : dt;
  PRINT_DEBUG("[B-SPLINE]: control point dt = %.3f (original dt of %.3f)\n", dt,
              sumdt / (traj_points.size() - 1));

  // convert all our trajectory points into SE(3) matrices
  // we are given [timestamp, p_IinG, q_GtoI]
  AlignedEigenMat4d trajectory_points;
  for (size_t i = 0; i < traj_points.size() - 1; i++) {
    Mat4 T_IinG = Mat4::Identity();
    T_IinG.block(0, 0, 3, 3) =
        quat_2_Rot(traj_points.at(i).block(4, 0, 4, 1).cast<DataType>())
            .transpose();
    T_IinG.block(0, 3, 3, 1) =
        traj_points.at(i).block(1, 0, 3, 1).cast<DataType>();
    trajectory_points.insert({traj_points.at(i)(0), T_IinG});
  }

  // Get the oldest timestamp
  double timestamp_min = INFINITY;
  double timestamp_max = -INFINITY;
  for (const auto &pose : trajectory_points) {
    if (pose.first <= timestamp_min) {
      timestamp_min = pose.first;
    }
    if (pose.first >= timestamp_max) {
      timestamp_max = pose.first;
    }
  }
  PRINT_DEBUG("[B-SPLINE]: trajectory start time = %.6f\n", timestamp_min);
  PRINT_DEBUG("[B-SPLINE]: trajectory end time = %.6f\n", timestamp_max);

  // then create spline control points
  double timestamp_curr = timestamp_min;
  while (true) {

    // Get bounding posed for the current time
    double t0, t1;
    Mat4 pose0, pose1;
    bool success = find_bounding_poses(timestamp_curr, trajectory_points, t0,
                                       pose0, t1, pose1);
    // PRINT_DEBUG("[SIM]: time curr = %.6f | lambda = %.3f | dt = %.3f | dtmeas
    // =
    // %.3f\n",timestamp_curr,(timestamp_curr-t0)/(t1-t0),dt,(t1-t0));

    // If we didn't find a bounding pose, then that means we are at the end of
    // the dataset Thus break out of this loop since we have created our max
    // number of control points
    if (!success)
      break;

    // Linear interpolation and append to our control points
    DataType lambda = (timestamp_curr - t0) / (t1 - t0);
    Mat4 pose_interp =
        exp_se3(lambda * log_se3(pose1 * Inv_se3(pose0))) * pose0;
    control_points.insert({timestamp_curr, pose_interp});
    timestamp_curr += dt;
    // std::stringstream ss;
    // ss << pose_interp(0,3) << "," << pose_interp(1,3) << "," <<
    // pose_interp(2,3) << std::endl; PRINT_DEBUG(ss.str().c_str());
  }

  // The start time of the system is two dt in since we need at least two older
  // control points
  timestamp_start = timestamp_min + 2 * dt;
  PRINT_DEBUG("[B-SPLINE]: start trajectory time of %.6f\n", timestamp_start);
}

bool BsplineSE3::get_pose(double timestamp, Mat3 &R_GtoI, Vec3 &p_IinG) {

  // Get the bounding poses for the desired timestamp
  double t0, t1, t2, t3;
  Mat4 pose0, pose1, pose2, pose3;
  bool success = find_bounding_control_points(
      timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);
  // PRINT_DEBUG("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f
  // | dt4 = %.3f | success =
  // %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

  // Return failure if we can't get bounding poses
  if (!success) {
    R_GtoI.setIdentity();
    p_IinG.setZero();
    return false;
  }

  // Our De Boor-Cox matrix scalars
  DataType DT = (t2 - t1);
  DataType u = (timestamp - t1) / DT;
  DataType b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  DataType b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  DataType b2 = 1.0 / 6.0 * (u * u * u);

  // Calculate interpolated poses
  Mat4 A0 = exp_se3(b0 * log_se3(Inv_se3(pose0) * pose1));
  Mat4 A1 = exp_se3(b1 * log_se3(Inv_se3(pose1) * pose2));
  Mat4 A2 = exp_se3(b2 * log_se3(Inv_se3(pose2) * pose3));

  // Finally get the interpolated pose
  Mat4 pose_interp = pose0 * A0 * A1 * A2;
  R_GtoI = pose_interp.block(0, 0, 3, 3).transpose();
  p_IinG = pose_interp.block(0, 3, 3, 1);
  return true;
}

bool BsplineSE3::get_velocity(double timestamp, Mat3 &R_GtoI, Vec3 &p_IinG,
                              Vec3 &w_IinI, Vec3 &v_IinG) {

  // Get the bounding poses for the desired timestamp
  double t0, t1, t2, t3;
  Mat4 pose0, pose1, pose2, pose3;
  bool success = find_bounding_control_points(
      timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);
  // PRINT_DEBUG("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f
  // | dt4 = %.3f | success =
  // %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

  // Return failure if we can't get bounding poses
  if (!success) {
    w_IinI.setZero();
    v_IinG.setZero();
    return false;
  }

  // Our De Boor-Cox matrix scalars
  DataType DT = (t2 - t1);
  DataType u = (timestamp - t1) / DT;
  DataType b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  DataType b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  DataType b2 = 1.0 / 6.0 * (u * u * u);
  DataType b0dot = 1.0 / (6.0 * DT) * (3 - 6 * u + 3 * u * u);
  DataType b1dot = 1.0 / (6.0 * DT) * (3 + 6 * u - 6 * u * u);
  DataType b2dot = 1.0 / (6.0 * DT) * (3 * u * u);

  // Cache some values we use alot
  Eigen::Matrix<DataType, 6, 1> omega_10 = log_se3(Inv_se3(pose0) * pose1);
  Eigen::Matrix<DataType, 6, 1> omega_21 = log_se3(Inv_se3(pose1) * pose2);
  Eigen::Matrix<DataType, 6, 1> omega_32 = log_se3(Inv_se3(pose2) * pose3);

  // Calculate interpolated poses
  Mat4 A0 = exp_se3(b0 * omega_10);
  Mat4 A1 = exp_se3(b1 * omega_21);
  Mat4 A2 = exp_se3(b2 * omega_32);
  Mat4 A0dot = b0dot * hat_se3(omega_10) * A0;
  Mat4 A1dot = b1dot * hat_se3(omega_21) * A1;
  Mat4 A2dot = b2dot * hat_se3(omega_32) * A2;

  // Get the interpolated pose
  Mat4 pose_interp = pose0 * A0 * A1 * A2;
  R_GtoI = pose_interp.block(0, 0, 3, 3).transpose();
  p_IinG = pose_interp.block(0, 3, 3, 1);

  // Finally get the interpolated velocities
  // NOTE: Rdot = R*skew(omega) => R^T*Rdot = skew(omega)
  Mat4 vel_interp =
      pose0 * (A0dot * A1 * A2 + A0 * A1dot * A2 + A0 * A1 * A2dot);
  w_IinI = vee(pose_interp.block(0, 0, 3, 3).transpose() *
               vel_interp.block(0, 0, 3, 3));
  v_IinG = vel_interp.block(0, 3, 3, 1);
  return true;
}

bool BsplineSE3::get_acceleration(double timestamp, Mat3 &R_GtoI, Vec3 &p_IinG,
                                  Vec3 &w_IinI, Vec3 &v_IinG, Vec3 &alpha_IinI,
                                  Vec3 &a_IinG) {

  // Get the bounding poses for the desired timestamp
  double t0, t1, t2, t3;
  Mat4 pose0, pose1, pose2, pose3;
  bool success = find_bounding_control_points(
      timestamp, control_points, t0, pose0, t1, pose1, t2, pose2, t3, pose3);

  // Return failure if we can't get bounding poses
  if (!success) {
    alpha_IinI.setZero();
    a_IinG.setZero();
    return false;
  }

  // Our De Boor-Cox matrix scalars
  DataType DT = (t2 - t1);
  DataType u = (timestamp - t1) / DT;
  DataType b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
  DataType b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
  DataType b2 = 1.0 / 6.0 * (u * u * u);
  DataType b0dot = 1.0 / (6.0 * DT) * (3 - 6 * u + 3 * u * u);
  DataType b1dot = 1.0 / (6.0 * DT) * (3 + 6 * u - 6 * u * u);
  DataType b2dot = 1.0 / (6.0 * DT) * (3 * u * u);
  DataType b0dotdot = 1.0 / (6.0 * DT * DT) * (-6 + 6 * u);
  DataType b1dotdot = 1.0 / (6.0 * DT * DT) * (6 - 12 * u);
  DataType b2dotdot = 1.0 / (6.0 * DT * DT) * (6 * u);

  // Cache some values we use alot
  Eigen::Matrix<DataType, 6, 1> omega_10 = log_se3(Inv_se3(pose0) * pose1);
  Eigen::Matrix<DataType, 6, 1> omega_21 = log_se3(Inv_se3(pose1) * pose2);
  Eigen::Matrix<DataType, 6, 1> omega_32 = log_se3(Inv_se3(pose2) * pose3);
  Mat4 omega_10_hat = hat_se3(omega_10);
  Mat4 omega_21_hat = hat_se3(omega_21);
  Mat4 omega_32_hat = hat_se3(omega_32);

  // Calculate interpolated poses
  Mat4 A0 = exp_se3(b0 * omega_10);
  Mat4 A1 = exp_se3(b1 * omega_21);
  Mat4 A2 = exp_se3(b2 * omega_32);
  Mat4 A0dot = b0dot * omega_10_hat * A0;
  Mat4 A1dot = b1dot * omega_21_hat * A1;
  Mat4 A2dot = b2dot * omega_32_hat * A2;
  Mat4 A0dotdot = b0dot * omega_10_hat * A0dot + b0dotdot * omega_10_hat * A0;
  Mat4 A1dotdot = b1dot * omega_21_hat * A1dot + b1dotdot * omega_21_hat * A1;
  Mat4 A2dotdot = b2dot * omega_32_hat * A2dot + b2dotdot * omega_32_hat * A2;

  // Get the interpolated pose
  Mat4 pose_interp = pose0 * A0 * A1 * A2;
  R_GtoI = pose_interp.block(0, 0, 3, 3).transpose();
  p_IinG = pose_interp.block(0, 3, 3, 1);

  // Get the interpolated velocities
  // NOTE: Rdot = R*skew(omega) => R^T*Rdot = skew(omega)
  Mat4 vel_interp =
      pose0 * (A0dot * A1 * A2 + A0 * A1dot * A2 + A0 * A1 * A2dot);
  w_IinI = vee(pose_interp.block(0, 0, 3, 3).transpose() *
               vel_interp.block(0, 0, 3, 3));
  v_IinG = vel_interp.block(0, 3, 3, 1);

  // Finally get the interpolated velocities
  // NOTE: Rdot = R*skew(omega)
  // NOTE: Rdotdot = Rdot*skew(omega) + R*skew(alpha) =>
  // R^T*(Rdotdot-Rdot*skew(omega))=skew(alpha)
  Mat4 acc_interp = pose0 * (A0dotdot * A1 * A2 + A0 * A1dotdot * A2 +
                             A0 * A1 * A2dotdot + 2 * A0dot * A1dot * A2 +
                             2 * A0 * A1dot * A2dot + 2 * A0dot * A1 * A2dot);
  Mat3 omegaskew =
      pose_interp.block(0, 0, 3, 3).transpose() * vel_interp.block(0, 0, 3, 3);
  alpha_IinI = vee(pose_interp.block(0, 0, 3, 3).transpose() *
                   (acc_interp.block(0, 0, 3, 3) -
                    vel_interp.block(0, 0, 3, 3) * omegaskew));
  a_IinG = acc_interp.block(0, 3, 3, 1);
  return true;
}

bool BsplineSE3::find_bounding_poses(const double timestamp,
                                     const AlignedEigenMat4d &poses, double &t0,
                                     Mat4 &pose0, double &t1, Mat4 &pose1) {

  // Set the default values
  t0 = -1;
  t1 = -1;
  pose0 = Mat4::Identity();
  pose1 = Mat4::Identity();

  // Find the bounding poses
  bool found_older = false;
  bool found_newer = false;

  // Find the bounding poses for interpolation.
  auto lower_bound = poses.lower_bound(
      timestamp); // Finds timestamp or next(timestamp) if not available
  auto upper_bound = poses.upper_bound(timestamp); // Finds next(timestamp)

  if (lower_bound != poses.end()) {
    // Check that the lower bound is the timestamp.
    // If not then we move iterator to previous timestamp so that the timestamp
    // is bounded
    if (lower_bound->first == timestamp) {
      found_older = true;
    } else if (lower_bound != poses.begin()) {
      --lower_bound;
      found_older = true;
    }
  }

  if (upper_bound != poses.end()) {
    found_newer = true;
  }

  // If we found the older one, set it
  if (found_older) {
    t0 = lower_bound->first;
    pose0 = lower_bound->second;
  }

  // If we found the newest one, set it
  if (found_newer) {
    t1 = upper_bound->first;
    pose1 = upper_bound->second;
  }

  // Assert the timestamps
  if (found_older && found_newer)
    assert(t0 < t1);

  // Return true if we found both bounds
  return (found_older && found_newer);
}

bool BsplineSE3::find_bounding_control_points(const double timestamp,
                                              const AlignedEigenMat4d &poses,
                                              double &t0, Mat4 &pose0,
                                              double &t1, Mat4 &pose1,
                                              double &t2, Mat4 &pose2,
                                              double &t3, Mat4 &pose3) {

  // Set the default values
  t0 = -1;
  t1 = -1;
  t2 = -1;
  t3 = -1;
  pose0 = Mat4::Identity();
  pose1 = Mat4::Identity();
  pose2 = Mat4::Identity();
  pose3 = Mat4::Identity();

  // Get the two bounding poses
  bool success = find_bounding_poses(timestamp, poses, t1, pose1, t2, pose2);

  // Return false if this was a failure
  if (!success)
    return false;

  // Now find the poses that are below and above
  auto iter_t1 = poses.find(t1);
  auto iter_t2 = poses.find(t2);

  // Check that t1 is not the first timestamp
  if (iter_t1 == poses.begin()) {
    return false;
  }

  // Move the older pose backwards in time
  // Move the newer one forwards in time
  auto iter_t0 = --iter_t1;
  auto iter_t3 = ++iter_t2;

  // Check that it is valid
  if (iter_t3 == poses.end()) {
    return false;
  }

  // Set the oldest one
  t0 = iter_t0->first;
  pose0 = iter_t0->second;

  // Set the newest one
  t3 = iter_t3->first;
  pose3 = iter_t3->second;

  // Assert the timestamps
  if (success) {
    assert(t0 < t1);
    assert(t1 < t2);
    assert(t2 < t3);
  }

  // Return true if we found all four bounding poses
  return success;
}
