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
#include "Eigen/Eigen"

#if USE_FLOAT
typedef float DataType;
#else
typedef double DataType;
#endif
typedef Eigen::Matrix<DataType, 2, 1> Vec2;
typedef Eigen::Matrix<DataType, 3, 1> Vec3;
typedef Eigen::Matrix<DataType, 4, 1> Vec4;
typedef Eigen::Matrix<DataType, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<DataType, 2, 2> Mat2;
typedef Eigen::Matrix<DataType, 3, 3> Mat3;
typedef Eigen::Matrix<DataType, 4, 4> Mat4;
typedef Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> MatX;
