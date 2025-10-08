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




#ifndef OV_SRVINS_NOISEMANAGER_H
#define OV_SRVINS_NOISEMANAGER_H

#include "math.h"
#include "utils/DataType.h"

namespace ov_srvins {

/**
 * @brief Struct of our imu noise parameters
 */
struct NoiseManager {

  /// Gyroscope white noise (rad/s/sqrt(hz))
  DataType sigma_w = 1.6968e-04;

  /// Gyroscope white noise covariance
  DataType sigma_w_2 = pow(1.6968e-04, 2);

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  DataType sigma_wb = 1.9393e-05;

  /// Gyroscope random walk covariance
  DataType sigma_wb_2 = pow(1.9393e-05, 2);

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  DataType sigma_a = 2.0000e-3;

  /// Accelerometer white noise covariance
  DataType sigma_a_2 = pow(2.0000e-3, 2);

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  DataType sigma_ab = 3.0000e-03;

  /// Accelerometer random walk covariance
  DataType sigma_ab_2 = pow(3.0000e-03, 2);

  /// Nice print function of what parameters we have loaded
  void print();
};

} // namespace ov_srvins

#endif // OV_SRVINS_NOISEMANAGER_H