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





#ifndef OV_CORE_CAM_RADTAN_H
#define OV_CORE_CAM_RADTAN_H

#include "CamBase.h"

namespace ov_core {

/**
 * @brief Radial-tangential / Brown–Conrady model pinhole camera model class
 *
 * To calibrate camera intrinsics, we need to know how to map our normalized
 * coordinates into the raw pixel coordinates on the image plane. We first
 * employ the radial distortion as in [OpenCV
 * model](https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#details):
 *
 * \f{align*}{
 * \begin{bmatrix} u \\ v \end{bmatrix}:= \mathbf{z}_k &= \mathbf
 * h_d(\mathbf{z}_{n,k}, ~\boldsymbol\zeta)
 * = \begin{bmatrix}  f_x * x + c_x \\
 * f_y * y + c_y \end{bmatrix}\\[1em]
 * \empty
 * {\rm where}~~
 * x &= x_n (1 + k_1 r^2 + k_2 r^4) + 2 p_1 x_n y_n + p_2(r^2 + 2 x_n^2) \\\
 * y &= y_n (1 + k_1 r^2 + k_2 r^4) + p_1 (r^2 + 2 y_n^2) + 2 p_2 x_n y_n
 * \\[1em] r^2 &= x_n^2 + y_n^2 \f}
 *
 * where \f$ \mathbf{z}_{n,k} = [ x_n ~ y_n ]^\top\f$  are the normalized
 * coordinates of the 3D feature and u and v are the distorted image coordinates
 * on the image plane. The following distortion and camera intrinsic (focal
 * length and image center) parameters are involved in the above distortion
 * model, which can be estimated online:
 *
 * \f{align*}{
 * \boldsymbol\zeta = \begin{bmatrix} f_x & f_y & c_x & c_y & k_1 & k_2 & p_1 &
 * p_2 \end{bmatrix}^\top \f}
 *
 * Note that we do not estimate the higher order (i.e., higher than fourth
 * order) terms as in most offline calibration methods such as
 * [Kalibr](https://github.com/ethz-asl/kalibr). To estimate these intrinsic
 * parameters (including the distortation parameters), the following Jacobian
 * for these parameters is needed:
 *
 * \f{align*}{
 * \frac{\partial \mathbf h_d(\cdot)}{\partial \boldsymbol\zeta} =
 * \begin{bmatrix}
 * x & 0  & 1 & 0 & f_x*(x_nr^2) & f_x*(x_nr^4) & f_x*(2x_ny_n) &
 * f_x*(r^2+2x_n^2)  \\[5pt] 0  & y & 0 & 1 & f_y*(y_nr^2) & f_y*(y_nr^4) &
 * f_y*(r^2+2y_n^2) & f_y*(2x_ny_n) \end{bmatrix} \f}
 *
 * Similarly, the Jacobian with respect to the normalized coordinates can be
 * obtained as follows:
 *
 * \f{align*}{
 * \frac{\partial \mathbf h_d (\cdot)}{\partial \mathbf{z}_{n,k}} =
 * \begin{bmatrix}
 * f_x*((1+k_1r^2+k_2r^4)+(2k_1x_n^2+4k_2x_n^2(x_n^2+y_n^2))+2p_1y_n+(2p_2x_n+4p_2x_n))
 * &
 * f_x*(2k_1x_ny_n+4k_2x_ny_n(x_n^2+y_n^2)+2p_1x_n+2p_2y_n)    \\
 * f_y*(2k_1x_ny_n+4k_2x_ny_n(x_n^2+y_n^2)+2p_1x_n+2p_2y_n)  &
 * f_y*((1+k_1r^2+k_2r^4)+(2k_1y_n^2+4k_2y_n^2(x_n^2+y_n^2))+(2p_1y_n+4p_1y_n)+2p_2x_n)
 * \end{bmatrix}
 * \f}
 *
 * To equate this camera class to Kalibr's models, this is what you would use
 * for `pinhole-radtan`.
 *
 */
class CamRadtan : public CamBase {

public:
  /**
   * @brief Default constructor
   * @param width Width of the camera (raw pixels)
   * @param height Height of the camera (raw pixels)
   */
  CamRadtan(int width, int height) : CamBase(width, height) {}

  ~CamRadtan() {}

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera
   * matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  Vec2 undistort(const Vec2 &uv_dist) override {
    // Convert to opencv format
    cv::Mat mat(1, 2, CV_32F);
    mat.at<float>(0, 0) = uv_dist(0);
    mat.at<float>(0, 1) = uv_dist(1);
    mat = mat.reshape(2); // Nx1, 2-channel

    // Undistort it!
    cv::undistortPoints(mat, mat, camera_k_OPENCV_, camera_d_OPENCV_);
    return Vec2(mat.at<float>(0, 0), mat.at<float>(0, 1));
  }

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw
   * image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  Vec2 distort(const Vec2 &uv_norm) override {
    // Calculate distorted coordinates for radial
    DataType r = uv_norm.norm();
    DataType x = uv_norm(0);
    DataType y = uv_norm(1);
    DataType r_2 = r * r;
    DataType r_4 = r_2 * r_2;
    DataType alpha = 1 + camera_values_(4) * r_2 + camera_values_(5) * r_4;
    DataType x1 = x * alpha + 2 * camera_values_(6) * x * y +
                  camera_values_(7) * (r_2 + 2 * x * x);
    DataType y1 = y * alpha + camera_values_(6) * (r_2 + 2 * y * y) +
                  2 * camera_values_(7) * x * y;

    // Return the distorted point
    Vec2 uv_dist;
    uv_dist(0) = camera_values_(0) * x1 + camera_values_(2);
    uv_dist(1) = camera_values_(1) * y1 + camera_values_(3);
    return uv_dist;
  }

  /**
   * @brief Computes the derivative of raw distorted to normalized coordinate.
   * @param uv_norm Normalized coordinates we wish to distort
   * @param H_dz_dzn Derivative of measurement z in respect to normalized
   * @param H_dz_dzeta Derivative of measurement z in respect to intrinic
   * parameters
   */
  void compute_distort_jacobian(const Vec2 &uv_norm, Mat2 &H_dz_dzn,
                                Eigen::Matrix<DataType, 2, 8> &H_dz_dzeta,
                                bool do_calib = true) override {
    // Calculate distorted coordinates for radial
    DataType r = uv_norm.norm();
    DataType r_2 = r * r;
    DataType r_4 = r_2 * r_2;

    // Jacobian of distorted pixel to normalized pixel
    // H_dz_dzn = MatX::Zero(2, 2);
    H_dz_dzn.setZero();
    DataType x = uv_norm(0);
    DataType y = uv_norm(1);
    DataType x_2 = x * x;
    DataType y_2 = y * y;
    DataType x_y = x * y;
    DataType alpha = 1 + camera_values_(4) * r_2 + camera_values_(5) * r_4;
    DataType beta = 2 * camera_values_(4) * x_y +
                    4 * camera_values_(5) * x_y * r_2 +
                    2 * camera_values_(6) * x + 2 * camera_values_(7) * y;
    H_dz_dzn(0, 0) =
        camera_values_(0) *
        (alpha +
         (2 * camera_values_(4) * x_2 + 4 * camera_values_(5) * x_2 * r_2) +
         2 * camera_values_(6) * y + 6 * camera_values_(7) * x);
    H_dz_dzn(0, 1) = camera_values_(0) * beta;
    H_dz_dzn(1, 0) = camera_values_(1) * beta;
    H_dz_dzn(1, 1) =
        camera_values_(1) *
        (alpha +
         (2 * camera_values_(4) * y_2 + 4 * camera_values_(5) * y_2 * r_2) +
         2 * camera_values_(7) * x + 6 * camera_values_(6) * y);

    // Compute the Jacobian in respect to the intrinsics
    if (do_calib) {
      // Calculate distorted coordinates for radtan
      DataType x1 = x * alpha + 2 * camera_values_(6) * x_y +
                    camera_values_(7) * (r_2 + 2 * x_2);
      DataType y1 = y * alpha + camera_values_(6) * (r_2 + 2 * y_2) +
                    2 * camera_values_(7) * x_y;
      // H_dz_dzeta = MatX::Zero(2, 8);
      H_dz_dzeta.setZero();
      H_dz_dzeta(0, 0) = x1;
      H_dz_dzeta(0, 2) = 1;
      H_dz_dzeta(0, 4) = camera_values_(0) * x * r_2;
      H_dz_dzeta(0, 5) = camera_values_(0) * x * r_4;
      H_dz_dzeta(0, 6) = 2 * camera_values_(0) * x_y;
      H_dz_dzeta(0, 7) = camera_values_(0) * (r_2 + 2 * x_2);
      H_dz_dzeta(1, 1) = y1;
      H_dz_dzeta(1, 3) = 1;
      H_dz_dzeta(1, 4) = camera_values_(1) * y * r_2;
      H_dz_dzeta(1, 5) = camera_values_(1) * y * r_4;
      H_dz_dzeta(1, 6) = camera_values_(1) * (r_2 + 2 * y_2);
      H_dz_dzeta(1, 7) = 2 * camera_values_(1) * x_y;
    }
  }
};

} // namespace ov_core

#endif /* OV_CORE_CAM_RADTAN_H */