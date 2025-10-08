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





#ifndef OV_CORE_CAM_EQUI_H
#define OV_CORE_CAM_EQUI_H

#include "CamBase.h"

namespace ov_core {

/**
 * @brief Fisheye / equadistant model pinhole camera model class
 *
 * As fisheye or wide-angle lenses are widely used in practice, we here provide
 * mathematical derivations of such distortion model as in [OpenCV
 * fisheye](https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html#details).
 *
 * \f{align*}{
 * \begin{bmatrix} u \\ v \end{bmatrix}:= \mathbf{z}_k &= \mathbf
 * h_d(\mathbf{z}_{n,k}, ~\boldsymbol\zeta)
 * = \begin{bmatrix}  f_x * x + c_x \\
 * f_y * y + c_y \end{bmatrix}\\[1em]
 * \empty
 * {\rm where}~~
 * x &= \frac{x_n}{r} * \theta_d \\
 * y &= \frac{y_n}{r} * \theta_d \\
 * \theta_d &= \theta (1 + k_1 \theta^2 + k_2 \theta^4 + k_3 \theta^6 + k_4
 * \theta^8) \\
 * \quad r^2 &= x_n^2 + y_n^2 \\
 * \theta &= atan(r)
 * \f}
 *
 * where \f$ \mathbf{z}_{n,k} = [ x_n ~ y_n ]^\top\f$ are the normalized
 * coordinates of the 3D feature and u and v are the distorted image coordinates
 * on the image plane. Clearly, the following distortion intrinsic parameters
 * are used in the above model:
 *
 * \f{align*}{
 * \boldsymbol\zeta = \begin{bmatrix} f_x & f_y & c_x & c_y & k_1 & k_2 & k_3 &
 * k_4 \end{bmatrix}^\top \f}
 *
 * In analogy to the previous radial distortion (see @ref ov_core::CamRadtan)
 * case, the following Jacobian for these parameters is needed for intrinsic
 * calibration: \f{align*}{ \frac{\partial \mathbf h_d (\cdot)}{\partial
 * \boldsymbol\zeta} = \begin{bmatrix} x_n & 0  & 1 & 0 &
 * f_x*(\frac{x_n}{r}\theta^3) & f_x*(\frac{x_n}{r}\theta^5) &
 * f_x*(\frac{x_n}{r}\theta^7) & f_x*(\frac{x_n}{r}\theta^9)
 * \\[5pt] 0  & y_n & 0 & 1 & f_y*(\frac{y_n}{r}\theta^3) &
 * f_y*(\frac{y_n}{r}\theta^5) & f_y*(\frac{y_n}{r}\theta^7) &
 * f_y*(\frac{y_n}{r}\theta^9) \end{bmatrix} \f}
 *
 * Similarly, with the chain rule of differentiation,
 * we can compute the following Jacobian with respect to the normalized
 * coordinates:
 *
 * \f{align*}{
 * \frac{\partial \mathbf h_d(\cdot)}{\partial \mathbf{z}_{n,k}}
 * &=
 * \frac{\partial uv}{\partial xy}\frac{\partial xy}{\partial x_ny_n}+
 * \frac{\partial uv}{\partial xy}\frac{\partial xy}{\partial r}\frac{\partial
 * r}{\partial x_ny_n}+ \frac{\partial uv}{\partial xy}\frac{\partial
 * xy}{\partial \theta_d}\frac{\partial \theta_d}{\partial \theta}\frac{\partial
 * \theta}{\partial r}\frac{\partial r}{\partial x_ny_n} \\[1em] \empty
 * {\rm where}~~~~
 * \frac{\partial uv}{\partial xy} &= \begin{bmatrix} f_x & 0 \\ 0 & f_y
 * \end{bmatrix} \\ \empty
 * \frac{\partial xy}{\partial x_ny_n} &= \begin{bmatrix} \theta_d/r & 0 \\ 0 &
 * \theta_d/r \end{bmatrix} \\ \empty
 * \frac{\partial xy}{\partial r} &= \begin{bmatrix} -\frac{x_n}{r^2}\theta_d \\
 * -\frac{y_n}{r^2}\theta_d \end{bmatrix} \\ \empty
 * \frac{\partial r}{\partial x_ny_n} &= \begin{bmatrix} \frac{x_n}{r} &
 * \frac{y_n}{r} \end{bmatrix} \\ \empty
 * \frac{\partial xy}{\partial \theta_d} &= \begin{bmatrix} \frac{x_n}{r} \\
 * \frac{y_n}{r} \end{bmatrix} \\ \empty \frac{\partial \theta_d}{\partial
 * \theta} &= \begin{bmatrix} 1 + 3k_1 \theta^2 + 5k_2 \theta^4 + 7k_3 \theta^6
 * + 9k_4 \theta^8\end{bmatrix} \\ \empty \frac{\partial \theta}{\partial r} &=
 * \begin{bmatrix} \frac{1}{r^2+1} \end{bmatrix} \f}
 *
 * To equate this to one of Kalibr's models, this is what you would use for
 * `pinhole-equi`.
 */
class CamEqui : public CamBase {

public:
  /**
   * @brief Default constructor
   * @param width Width of the camera (raw pixels)
   * @param height Height of the camera (raw pixels)
   */
  CamEqui(int width, int height) : CamBase(width, height) { _type = "fisheye"; }

  ~CamEqui() {}

  /**
   * @brief Given a raw uv point, this will undistort it based on the camera
   * matrices into normalized camera coords.
   * @param uv_dist Raw uv coordinate we wish to undistort
   * @return 2d vector of normalized coordinates
   */
  Vec2 undistort(const Vec2 &uv_dist) override {

    cv::Mat mat(1, 2, CV_32F);
    mat.at<float>(0, 0) = uv_dist(0);
    mat.at<float>(0, 1) = uv_dist(1);
    mat = mat.reshape(2); // Nx1, 2-channel
    cv::fisheye::undistortPoints(mat, mat, camera_k_OPENCV_, camera_d_OPENCV_);
    return Vec2(mat.at<float>(0, 0), mat.at<float>(0, 1));
  }

  /**
   * @brief Given a normalized uv coordinate this will distort it to the raw
   * image plane
   * @param uv_norm Normalized coordinates we wish to distort
   * @return 2d vector of raw uv coordinate
   */
  Vec2 distort(const Vec2 &uv_norm) override {
    // Calculate distorted coordinates for fisheye
    DataType r = uv_norm.norm();
    DataType theta = std::atan(r);
    DataType theta_d = theta + camera_values_(4) * std::pow(theta, 3) +
                       camera_values_(5) * std::pow(theta, 5) +
                       camera_values_(6) * std::pow(theta, 7) +
                       camera_values_(7) * std::pow(theta, 9);

    // Handle when r is small (meaning our xy is near the camera center)
    DataType inv_r = (r > 1e-8) ? 1.0 / r : 1.0;
    DataType cdist = (r > 1e-8) ? theta_d * inv_r : 1.0;

    // Calculate distorted coordinates for fisheye
    Vec2 uv_dist;
    DataType x1 = uv_norm(0) * cdist;
    DataType y1 = uv_norm(1) * cdist;
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
    // Calculate distorted coordinates for fisheye
    DataType r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
    DataType theta = std::atan(r);
    DataType theta_d = theta + camera_values_(4) * std::pow(theta, 3) +
                       camera_values_(5) * std::pow(theta, 5) +
                       camera_values_(6) * std::pow(theta, 7) +
                       camera_values_(7) * std::pow(theta, 9);

    // Handle when r is small (meaning our xy is near the camera center)
    DataType inv_r = (r > 1e-8) ? 1.0 / r : 1.0;
    DataType cdist = (r > 1e-8) ? theta_d * inv_r : 1.0;

    // Jacobian of distorted pixel to "normalized" pixel
    Mat2 duv_dxy = Mat2::Zero();
    duv_dxy << camera_values_(0), 0, 0, camera_values_(1);

    // Jacobian of "normalized" pixel to normalized pixel
    Mat2 dxy_dxyn = Mat2::Zero();
    dxy_dxyn << theta_d * inv_r, 0, 0, theta_d * inv_r;

    // Jacobian of "normalized" pixel to r
    Vec2 dxy_dr = Vec2::Zero();
    dxy_dr << -uv_norm(0) * theta_d * inv_r * inv_r,
        -uv_norm(1) * theta_d * inv_r * inv_r;

    // Jacobian of r pixel to normalized xy
    Eigen::Matrix<DataType, 1, 2> dr_dxyn =
        Eigen::Matrix<DataType, 1, 2>::Zero();
    dr_dxyn << uv_norm(0) * inv_r, uv_norm(1) * inv_r;

    // Jacobian of "normalized" pixel to theta_d
    Vec2 dxy_dthd = Vec2::Zero();
    dxy_dthd << uv_norm(0) * inv_r, uv_norm(1) * inv_r;

    // Jacobian of theta_d to theta
    DataType dthd_dth = 1 + 3 * camera_values_(4) * std::pow(theta, 2) +
                        5 * camera_values_(5) * std::pow(theta, 4) +
                        7 * camera_values_(6) * std::pow(theta, 6) +
                        9 * camera_values_(7) * std::pow(theta, 8);

    // Jacobian of theta to r
    DataType dth_dr = 1 / (r * r + 1);

    // Total Jacobian wrt normalized pixel coordinates
    H_dz_dzn.setZero();
    H_dz_dzn = duv_dxy *
               (dxy_dxyn + (dxy_dr + dxy_dthd * dthd_dth * dth_dr) * dr_dxyn);

    // Calculate distorted coordinates for fisheye
    DataType x1 = uv_norm(0) * cdist;
    DataType y1 = uv_norm(1) * cdist;

    // Compute the Jacobian in respect to the intrinsics
    if (do_calib) {
      H_dz_dzeta.setZero();
      H_dz_dzeta(0, 0) = x1;
      H_dz_dzeta(0, 2) = 1;
      H_dz_dzeta(0, 4) =
          camera_values_(0) * uv_norm(0) * inv_r * std::pow(theta, 3);
      H_dz_dzeta(0, 5) =
          camera_values_(0) * uv_norm(0) * inv_r * std::pow(theta, 5);
      H_dz_dzeta(0, 6) =
          camera_values_(0) * uv_norm(0) * inv_r * std::pow(theta, 7);
      H_dz_dzeta(0, 7) =
          camera_values_(0) * uv_norm(0) * inv_r * std::pow(theta, 9);
      H_dz_dzeta(1, 1) = y1;
      H_dz_dzeta(1, 3) = 1;
      H_dz_dzeta(1, 4) =
          camera_values_(1) * uv_norm(1) * inv_r * std::pow(theta, 3);
      H_dz_dzeta(1, 5) =
          camera_values_(1) * uv_norm(1) * inv_r * std::pow(theta, 5);
      H_dz_dzeta(1, 6) =
          camera_values_(1) * uv_norm(1) * inv_r * std::pow(theta, 7);
      H_dz_dzeta(1, 7) =
          camera_values_(1) * uv_norm(1) * inv_r * std::pow(theta, 9);
    }
  }
};

} // namespace ov_core

#endif /* OV_CORE_CAM_EQUI_H */