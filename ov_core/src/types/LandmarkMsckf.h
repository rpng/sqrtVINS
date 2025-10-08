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




#ifndef OV_TYPE_TYPE_MSCKF_H
#define OV_TYPE_TYPE_MSCKF_H

#include "Landmark.h"
#include "utils/DataType.h"
namespace ov_type {

class LandmarkMsckf : public Landmark {

public:
  /// Default constructor (feature is a Vec of size 3 or Vec of size 1)
  LandmarkMsckf(int dim) : Landmark(dim) {}

  // Store the msckf jacobians and residual for this feature and its order in
  // the state vector
  void store_msckf_jacobians(
      const Mat3 &Hf, const MatX &Hx, const Vec3 &res,
      const std::vector<std::shared_ptr<ov_type::Type>> &x_order) {
    Hf_msckf = Hf;
    Hx_msckf = Hx;
    res_msckf = res;
    x_order_msckf = x_order;
  }

  // Record the top part msckf jacobians after qr decompostion and its order
  Mat3 Hf_msckf;
  Vec3 res_msckf = Vec3::Zero();
  MatX Hx_msckf;
  std::vector<std::shared_ptr<ov_type::Type>> x_order_msckf;

  // Dx from last update
  Vec3 feat_dx;
};
} // namespace ov_type

#endif // OV_TYPE_TYPE_MSCKF_H
