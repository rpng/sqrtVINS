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




#pragma once
#include "utils/DataType.h"
#include "utils/quat_ops.h"
#include <Eigen/Core>
namespace ov_srvins {

Vec3 Rot2Cayley(const Mat3 &R);

Mat3 Cayley2RotReduced(const Vec3 &cayley);

Mat3 ComposeMwithJacobians(const Mat3 &xxF, const Mat3 &yyF, const Mat3 &zzF,
                           const Mat3 &xyF, const Mat3 &yzF, const Mat3 &zxF,
                           const Vec3 &cayley, Mat3 &M_jac1, Mat3 &M_jac2,
                           Mat3 &M_jac3);

DataType GetSmallestEVwithJacobian(const Mat3 &xxF, const Mat3 &yyF,
                                   const Mat3 &zzF, const Mat3 &xyF,
                                   const Mat3 &yzF, const Mat3 &zxF,
                                   const Vec3 &cayley,
                                   Eigen::Matrix<DataType, 1, 3> &jacobian);

Mat3 GetNumericalJacobians(const Mat3 &R);

} // namespace ov_srvins
