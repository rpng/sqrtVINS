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



#ifndef OV_CORE_CAM_BASE_H
#define OV_CORE_CAM_BASE_H

#include <Eigen/Eigen>
#include <unordered_map>

#include "utils/DataType.h"
#include <opencv2/opencv.hpp>

namespace ov_core {

/**
 * @brief Base pinhole camera model class
 *
 * This is the base class for all our camera models.
 * All these models are pinhole cameras, thus just have standard reprojection
 * logic. See each derived class for detailed examples of each model.
 */
class CamBase {

public:
  /**
   * @brief Default constructor
   * @param width Width of the camera (raw pixels)
   * @param height Height of the camera (raw pixels)
   */
  CamBase(int width, int height) : width_(width), height_(height) {}

  virtual ~CamBase() {}

  /**
   * @brief This will set and update the camera calibration values.
   * This should be called on startup for each camera and after update!
   * @param calib Camera calibration information (f_x & f_y & c_x & c_y & k_1 &
   * k_2 & k_3 & k_4)
   */
  virtual void set_value(const VecX &calib) {

    // Assert we are of size eight
    assert(calib.rows() == 8);
    camera_values_ = calib;

    // Camera matrix
    cv::Matx33d tempK;
    tempK(0, 0) = calib(0);
    tempK(0, 1) = 0;
    tempK(0, 2) = calib(2);
    tempK(1, 0) = 0;
    tempK(1, 1) = calib(1);
    tempK(1, 2) = calib(3);
    tempK(2, 0) = 0;
    tempK(2, 1) = 0;
    tempK(2, 2) = 1;
    camera_k_OPENCV_ = tempK;

    // Distortion parameters
    cv::Vec4d tempD;
    tempD(0) = calib(4);
    tempD(1) = calib(5);
    tempD(2) = calib(6);
    tempD(3) = calib(7);
    camera_d_OPENCV_ = tempD;
  }

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera
   * matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  virtual Vec2 undistort(const Vec2 &uv_dist) = 0;

  // /**
  //  * @brief Given a raw uv point, this will undistort it based on the camera
  //  * matrices into normalized camera coords.
  //  * @param uv_dist Raw uv coordinate we wish to undistort
  //  * @return 2d vector of normalized coordinates
  //  */
  // Eigen::Vector2d undistort_d(const Eigen::Vector2d &uv_dist) {
  //   Eigen::Vector2f ept1, ept2;
  //   ept1 = uv_dist.cast<float>();
  //   ept2 = undistort_f(ept1);
  //   return ept2.cast<double>();
  // }

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera
   * matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  virtual cv::Point2f undistort(const cv::Point2f &uv_dist) {
    Vec2 uv_undist_vec = undistort(Vec2(uv_dist.x, uv_dist.y));
    return cv::Point2f(uv_undist_vec(0), uv_undist_vec(1));
  }

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw
   * image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  virtual Vec2 distort(const Vec2 &uv_undistort) = 0;

  // /**
  //  * @brief Given a normalized uv coordinate this will distort it to the raw
  //  * image plane
  //  * @param uv_norm Normalized coordinates we wish to distort
  //  * @return 2d vector of raw uv coordinate
  //  */
  // Eigen::Vector2d distort_d(const Eigen::Vector2d &uv_norm) {
  //   Eigen::Vector2f ept1, ept2;
  //   ept1 = uv_norm.cast<float>();
  //   ept2 = distort_f(ept1);
  //   return ept2.cast<double>();
  // }

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw
   * image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  virtual cv::Point2f distort(const cv::Point2f &uv_norm) {
    Vec2 uv_dist_vec = distort(Vec2(uv_norm.x, uv_norm.y));
    return cv::Point2f(uv_dist_vec(0), uv_dist_vec(1));
  }

  // cv::Point2f distort_cv(const cv::Point2f &uv_norm) {
  //   Eigen::Vector2f ept1, ept2;
  //   ept1 << uv_norm.x, uv_norm.y;
  //   ept2 = distort_f(ept1);
  //   cv::Point2f pt_out;
  //   pt_out.x = ept2(0);
  //   pt_out.y = ept2(1);
  //   return pt_out;
  // }

  /**
   * @brief Computes the derivative of raw distorted to normalized coordinate.
   * @param uv_norm Normalized coordinates we wish to distort
   * @param H_dz_dzn Derivative of measurement z in respect to normalized
   * @param H_dz_dzeta Derivative of measurement z in respect to intrinic
   * parameters
   */
  virtual void
  compute_distort_jacobian(const Vec2 &uv_norm, Mat2 &H_dz_dzn,
                           Eigen::Matrix<DataType, 2, 8> &H_dz_dzeta,
                           bool do_calib = true) = 0;

  /// Gets the complete intrinsic vector
  VecX get_value() { return camera_values_; }

  /// Gets the camera matrix
  cv::Matx33d get_K() { return camera_k_OPENCV_; }

  /// Gets the camera distortion
  cv::Vec4d get_D() { return camera_d_OPENCV_; }

  /// Gets the width of the camera images
  int w() { return width_; }

  /// Gets the height of the camera images
  int h() { return height_; }

  std::string _type = "pinhole";

protected:
  // Cannot construct the base camera class, needs a distortion model
  CamBase() = default;

  /// Raw set of camera intrinic values (f_x & f_y & c_x & c_y & k_1 & k_2 & k_3
  /// & k_4)
  VecX camera_values_;

  /// Camera intrinsics in OpenCV format
  cv::Matx33d camera_k_OPENCV_;

  /// Camera distortion in OpenCV format
  cv::Vec4d camera_d_OPENCV_;

  /// Width of the camera (raw pixels)
  int width_;

  /// Height of the camera (raw pixels)
  int height_;
};

} // namespace ov_core

#endif /* OV_CORE_CAM_BASE_H */