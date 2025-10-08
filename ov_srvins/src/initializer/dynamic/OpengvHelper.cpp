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




#include "utils/DataType.h"
#include "utils/quat_ops.h"
#include <Eigen/Core>

namespace ov_srvins {
Vec3 Rot2Cayley(const Mat3 &R) {
  Mat3 C1;
  Mat3 C2;
  Mat3 C;
  C1 = R - Mat3::Identity();
  C2 = R + Mat3::Identity();
  C = C1 * C2.inverse();

  Vec3 cayley;
  cayley[0] = -C(1, 2);
  cayley[1] = C(0, 2);
  cayley[2] = -C(0, 1);

  return cayley;
}

Mat3 Cayley2RotReduced(const Vec3 &cayley) {
  Mat3 R;

  R(0, 0) = 1 + pow(cayley[0], 2) - pow(cayley[1], 2) - pow(cayley[2], 2);
  R(0, 1) = 2 * (cayley[0] * cayley[1] - cayley[2]);
  R(0, 2) = 2 * (cayley[0] * cayley[2] + cayley[1]);
  R(1, 0) = 2 * (cayley[0] * cayley[1] + cayley[2]);
  R(1, 1) = 1 - pow(cayley[0], 2) + pow(cayley[1], 2) - pow(cayley[2], 2);
  R(1, 2) = 2 * (cayley[1] * cayley[2] - cayley[0]);
  R(2, 0) = 2 * (cayley[0] * cayley[2] - cayley[1]);
  R(2, 1) = 2 * (cayley[1] * cayley[2] + cayley[0]);
  R(2, 2) = 1 - pow(cayley[0], 2) - pow(cayley[1], 2) + pow(cayley[2], 2);

  return R;
}

Mat3 ComposeMwithJacobians(const Mat3 &xxF, const Mat3 &yyF, const Mat3 &zzF,
                           const Mat3 &xyF, const Mat3 &yzF, const Mat3 &zxF,
                           const Vec3 &cayley, Mat3 &M_jac1, Mat3 &M_jac2,
                           Mat3 &M_jac3) {

  Mat3 R = Cayley2RotReduced(cayley);
  Mat3 R_jac1;
  Mat3 R_jac2;
  Mat3 R_jac3;

  R_jac1(0, 0) = 2 * cayley[0];
  R_jac1(0, 1) = 2 * cayley[1];
  R_jac1(0, 2) = 2 * cayley[2];
  R_jac1(1, 0) = 2 * cayley[1];
  R_jac1(1, 1) = -2 * cayley[0];
  R_jac1(1, 2) = -2;
  R_jac1(2, 0) = 2 * cayley[2];
  R_jac1(2, 1) = 2;
  R_jac1(2, 2) = -2 * cayley[0];
  R_jac2(0, 0) = -2 * cayley[1];
  R_jac2(0, 1) = 2 * cayley[0];
  R_jac2(0, 2) = 2;
  R_jac2(1, 0) = 2 * cayley[0];
  R_jac2(1, 1) = 2 * cayley[1];
  R_jac2(1, 2) = 2 * cayley[2];
  R_jac2(2, 0) = -2;
  R_jac2(2, 1) = 2 * cayley[2];
  R_jac2(2, 2) = -2 * cayley[1];
  R_jac3(0, 0) = -2 * cayley[2];
  R_jac3(0, 1) = -2;
  R_jac3(0, 2) = 2 * cayley[0];
  R_jac3(1, 0) = 2;
  R_jac3(1, 1) = -2 * cayley[2];
  R_jac3(1, 2) = 2 * cayley[1];
  R_jac3(2, 0) = 2 * cayley[0];
  R_jac3(2, 1) = 2 * cayley[1];
  R_jac3(2, 2) = 2 * cayley[2];

  // Fill the matrix M using the precomputed summation terms. Plus Jacobian.
  Mat3 M;
  Eigen::Matrix<DataType, 1, 1> temp;
  temp = R.row(2) * yyF * R.row(2).transpose();
  M(0, 0) = temp(0, 0);
  temp = -2.0 * R.row(2) * yzF * R.row(1).transpose();
  M(0, 0) += temp(0, 0);
  temp = R.row(1) * zzF * R.row(1).transpose();
  M(0, 0) += temp(0, 0);
  temp = 2.0 * R_jac1.row(2) * yyF * R.row(2).transpose();
  M_jac1(0, 0) = temp(0, 0);
  temp = -2.0 * R_jac1.row(2) * yzF * R.row(1).transpose();
  M_jac1(0, 0) += temp(0, 0);
  temp = -2.0 * R.row(2) * yzF * R_jac1.row(1).transpose();
  M_jac1(0, 0) += temp(0, 0);
  temp = 2.0 * R_jac1.row(1) * zzF * R.row(1).transpose();
  M_jac1(0, 0) += temp(0, 0);
  temp = 2.0 * R_jac2.row(2) * yyF * R.row(2).transpose();
  M_jac2(0, 0) = temp(0, 0);
  temp = -2.0 * R_jac2.row(2) * yzF * R.row(1).transpose();
  M_jac2(0, 0) += temp(0, 0);
  temp = -2.0 * R.row(2) * yzF * R_jac2.row(1).transpose();
  M_jac2(0, 0) += temp(0, 0);
  temp = 2.0 * R_jac2.row(1) * zzF * R.row(1).transpose();
  M_jac2(0, 0) += temp(0, 0);
  temp = 2.0 * R_jac3.row(2) * yyF * R.row(2).transpose();
  M_jac3(0, 0) = temp(0, 0);
  temp = -2.0 * R_jac3.row(2) * yzF * R.row(1).transpose();
  M_jac3(0, 0) += temp(0, 0);
  temp = -2.0 * R.row(2) * yzF * R_jac3.row(1).transpose();
  M_jac3(0, 0) += temp(0, 0);
  temp = 2.0 * R_jac3.row(1) * zzF * R.row(1).transpose();
  M_jac3(0, 0) += temp(0, 0);

  temp = R.row(2) * yzF * R.row(0).transpose();
  M(0, 1) = temp(0, 0);
  temp = -1.0 * R.row(2) * xyF * R.row(2).transpose();
  M(0, 1) += temp(0, 0);
  temp = -1.0 * R.row(1) * zzF * R.row(0).transpose();
  M(0, 1) += temp(0, 0);
  temp = R.row(1) * zxF * R.row(2).transpose();
  M(0, 1) += temp(0, 0);
  temp = R_jac1.row(2) * yzF * R.row(0).transpose();
  M_jac1(0, 1) = temp(0, 0);
  temp = R.row(2) * yzF * R_jac1.row(0).transpose();
  M_jac1(0, 1) += temp(0, 0);
  temp = -2.0 * R_jac1.row(2) * xyF * R.row(2).transpose();
  M_jac1(0, 1) += temp(0, 0);
  temp = -R_jac1.row(1) * zzF * R.row(0).transpose();
  M_jac1(0, 1) += temp(0, 0);
  temp = -R.row(1) * zzF * R_jac1.row(0).transpose();
  M_jac1(0, 1) += temp(0, 0);
  temp = R_jac1.row(1) * zxF * R.row(2).transpose();
  M_jac1(0, 1) += temp(0, 0);
  temp = R.row(1) * zxF * R_jac1.row(2).transpose();
  M_jac1(0, 1) += temp(0, 0);
  temp = R_jac2.row(2) * yzF * R.row(0).transpose();
  M_jac2(0, 1) = temp(0, 0);
  temp = R.row(2) * yzF * R_jac2.row(0).transpose();
  M_jac2(0, 1) += temp(0, 0);
  temp = -2.0 * R_jac2.row(2) * xyF * R.row(2).transpose();
  M_jac2(0, 1) += temp(0, 0);
  temp = -R_jac2.row(1) * zzF * R.row(0).transpose();
  M_jac2(0, 1) += temp(0, 0);
  temp = -R.row(1) * zzF * R_jac2.row(0).transpose();
  M_jac2(0, 1) += temp(0, 0);
  temp = R_jac2.row(1) * zxF * R.row(2).transpose();
  M_jac2(0, 1) += temp(0, 0);
  temp = R.row(1) * zxF * R_jac2.row(2).transpose();
  M_jac2(0, 1) += temp(0, 0);
  temp = R_jac3.row(2) * yzF * R.row(0).transpose();
  M_jac3(0, 1) = temp(0, 0);
  temp = R.row(2) * yzF * R_jac3.row(0).transpose();
  M_jac3(0, 1) += temp(0, 0);
  temp = -2.0 * R_jac3.row(2) * xyF * R.row(2).transpose();
  M_jac3(0, 1) += temp(0, 0);
  temp = -R_jac3.row(1) * zzF * R.row(0).transpose();
  M_jac3(0, 1) += temp(0, 0);
  temp = -R.row(1) * zzF * R_jac3.row(0).transpose();
  M_jac3(0, 1) += temp(0, 0);
  temp = R_jac3.row(1) * zxF * R.row(2).transpose();
  M_jac3(0, 1) += temp(0, 0);
  temp = R.row(1) * zxF * R_jac3.row(2).transpose();
  M_jac3(0, 1) += temp(0, 0);

  temp = R.row(2) * xyF * R.row(1).transpose();
  M(0, 2) = temp(0, 0);
  temp = -1.0 * R.row(2) * yyF * R.row(0).transpose();
  M(0, 2) += temp(0, 0);
  temp = -1.0 * R.row(1) * zxF * R.row(1).transpose();
  M(0, 2) += temp(0, 0);
  temp = R.row(1) * yzF * R.row(0).transpose();
  M(0, 2) += temp(0, 0);
  temp = R_jac1.row(2) * xyF * R.row(1).transpose();

  M_jac1(0, 2) = temp(0, 0);
  temp = R.row(2) * xyF * R_jac1.row(1).transpose();
  M_jac1(0, 2) += temp(0, 0);
  temp = -R_jac1.row(2) * yyF * R.row(0).transpose();
  M_jac1(0, 2) += temp(0, 0);
  temp = -R.row(2) * yyF * R_jac1.row(0).transpose();
  M_jac1(0, 2) += temp(0, 0);
  temp = -2.0 * R_jac1.row(1) * zxF * R.row(1).transpose();
  M_jac1(0, 2) += temp(0, 0);
  temp = R_jac1.row(1) * yzF * R.row(0).transpose();
  M_jac1(0, 2) += temp(0, 0);
  temp = R.row(1) * yzF * R_jac1.row(0).transpose();
  M_jac1(0, 2) += temp(0, 0);
  temp = R_jac2.row(2) * xyF * R.row(1).transpose();
  M_jac2(0, 2) = temp(0, 0);
  temp = R.row(2) * xyF * R_jac2.row(1).transpose();
  M_jac2(0, 2) += temp(0, 0);
  temp = -R_jac2.row(2) * yyF * R.row(0).transpose();
  M_jac2(0, 2) += temp(0, 0);
  temp = -R.row(2) * yyF * R_jac2.row(0).transpose();
  M_jac2(0, 2) += temp(0, 0);
  temp = -2.0 * R_jac2.row(1) * zxF * R.row(1).transpose();
  M_jac2(0, 2) += temp(0, 0);
  temp = R_jac2.row(1) * yzF * R.row(0).transpose();
  M_jac2(0, 2) += temp(0, 0);
  temp = R.row(1) * yzF * R_jac2.row(0).transpose();
  M_jac2(0, 2) += temp(0, 0);
  temp = R_jac3.row(2) * xyF * R.row(1).transpose();
  M_jac3(0, 2) = temp(0, 0);
  temp = R.row(2) * xyF * R_jac3.row(1).transpose();
  M_jac3(0, 2) += temp(0, 0);
  temp = -R_jac3.row(2) * yyF * R.row(0).transpose();
  M_jac3(0, 2) += temp(0, 0);
  temp = -R.row(2) * yyF * R_jac3.row(0).transpose();
  M_jac3(0, 2) += temp(0, 0);
  temp = -2.0 * R_jac3.row(1) * zxF * R.row(1).transpose();
  M_jac3(0, 2) += temp(0, 0);
  temp = R_jac3.row(1) * yzF * R.row(0).transpose();
  M_jac3(0, 2) += temp(0, 0);
  temp = R.row(1) * yzF * R_jac3.row(0).transpose();
  M_jac3(0, 2) += temp(0, 0);

  temp = R.row(0) * zzF * R.row(0).transpose();
  M(1, 1) = temp(0, 0);
  temp = -2.0 * R.row(0) * zxF * R.row(2).transpose();
  M(1, 1) += temp(0, 0);
  temp = R.row(2) * xxF * R.row(2).transpose();
  M(1, 1) += temp(0, 0);
  temp = 2.0 * R_jac1.row(0) * zzF * R.row(0).transpose();
  M_jac1(1, 1) = temp(0, 0);
  temp = -2.0 * R_jac1.row(0) * zxF * R.row(2).transpose();
  M_jac1(1, 1) += temp(0, 0);
  temp = -2.0 * R.row(0) * zxF * R_jac1.row(2).transpose();
  M_jac1(1, 1) += temp(0, 0);
  temp = 2.0 * R_jac1.row(2) * xxF * R.row(2).transpose();
  M_jac1(1, 1) += temp(0, 0);
  temp = 2.0 * R_jac2.row(0) * zzF * R.row(0).transpose();
  M_jac2(1, 1) = temp(0, 0);
  temp = -2.0 * R_jac2.row(0) * zxF * R.row(2).transpose();
  M_jac2(1, 1) += temp(0, 0);
  temp = -2.0 * R.row(0) * zxF * R_jac2.row(2).transpose();
  M_jac2(1, 1) += temp(0, 0);
  temp = 2.0 * R_jac2.row(2) * xxF * R.row(2).transpose();
  M_jac2(1, 1) += temp(0, 0);
  temp = 2.0 * R_jac3.row(0) * zzF * R.row(0).transpose();
  M_jac3(1, 1) = temp(0, 0);
  temp = -2.0 * R_jac3.row(0) * zxF * R.row(2).transpose();
  M_jac3(1, 1) += temp(0, 0);
  temp = -2.0 * R.row(0) * zxF * R_jac3.row(2).transpose();
  M_jac3(1, 1) += temp(0, 0);
  temp = 2.0 * R_jac3.row(2) * xxF * R.row(2).transpose();
  M_jac3(1, 1) += temp(0, 0);

  temp = R.row(0) * zxF * R.row(1).transpose();
  M(1, 2) = temp(0, 0);
  temp = -1.0 * R.row(0) * yzF * R.row(0).transpose();
  M(1, 2) += temp(0, 0);
  temp = -1.0 * R.row(2) * xxF * R.row(1).transpose();
  M(1, 2) += temp(0, 0);
  temp = R.row(2) * xyF * R.row(0).transpose();
  M(1, 2) += temp(0, 0);
  temp = R_jac1.row(0) * zxF * R.row(1).transpose();
  M_jac1(1, 2) = temp(0, 0);
  temp = R.row(0) * zxF * R_jac1.row(1).transpose();
  M_jac1(1, 2) += temp(0, 0);
  temp = -2.0 * R_jac1.row(0) * yzF * R.row(0).transpose();
  M_jac1(1, 2) += temp(0, 0);
  temp = -R_jac1.row(2) * xxF * R.row(1).transpose();
  M_jac1(1, 2) += temp(0, 0);
  temp = -R.row(2) * xxF * R_jac1.row(1).transpose();
  M_jac1(1, 2) += temp(0, 0);
  temp = R_jac1.row(2) * xyF * R.row(0).transpose();
  M_jac1(1, 2) += temp(0, 0);
  temp = R.row(2) * xyF * R_jac1.row(0).transpose();
  M_jac1(1, 2) += temp(0, 0);
  temp = R_jac2.row(0) * zxF * R.row(1).transpose();
  M_jac2(1, 2) = temp(0, 0);
  temp = R.row(0) * zxF * R_jac2.row(1).transpose();
  M_jac2(1, 2) += temp(0, 0);
  temp = -2.0 * R_jac2.row(0) * yzF * R.row(0).transpose();
  M_jac2(1, 2) += temp(0, 0);
  temp = -R_jac2.row(2) * xxF * R.row(1).transpose();
  M_jac2(1, 2) += temp(0, 0);
  temp = -R.row(2) * xxF * R_jac2.row(1).transpose();
  M_jac2(1, 2) += temp(0, 0);
  temp = R_jac2.row(2) * xyF * R.row(0).transpose();
  M_jac2(1, 2) += temp(0, 0);
  temp = R.row(2) * xyF * R_jac2.row(0).transpose();
  M_jac2(1, 2) += temp(0, 0);
  temp = R_jac3.row(0) * zxF * R.row(1).transpose();
  M_jac3(1, 2) = temp(0, 0);
  temp = R.row(0) * zxF * R_jac3.row(1).transpose();
  M_jac3(1, 2) += temp(0, 0);
  temp = -2.0 * R_jac3.row(0) * yzF * R.row(0).transpose();
  M_jac3(1, 2) += temp(0, 0);
  temp = -R_jac3.row(2) * xxF * R.row(1).transpose();
  M_jac3(1, 2) += temp(0, 0);
  temp = -R.row(2) * xxF * R_jac3.row(1).transpose();
  M_jac3(1, 2) += temp(0, 0);
  temp = R_jac3.row(2) * xyF * R.row(0).transpose();
  M_jac3(1, 2) += temp(0, 0);
  temp = R.row(2) * xyF * R_jac3.row(0).transpose();
  M_jac3(1, 2) += temp(0, 0);

  temp = R.row(1) * xxF * R.row(1).transpose();
  M(2, 2) = temp(0, 0);
  temp = -2.0 * R.row(0) * xyF * R.row(1).transpose();
  M(2, 2) += temp(0, 0);
  temp = R.row(0) * yyF * R.row(0).transpose();
  M(2, 2) += temp(0, 0);
  temp = 2.0 * R_jac1.row(1) * xxF * R.row(1).transpose();
  M_jac1(2, 2) = temp(0, 0);
  temp = -2.0 * R_jac1.row(0) * xyF * R.row(1).transpose();
  M_jac1(2, 2) += temp(0, 0);
  temp = -2.0 * R.row(0) * xyF * R_jac1.row(1).transpose();
  M_jac1(2, 2) += temp(0, 0);
  temp = 2.0 * R_jac1.row(0) * yyF * R.row(0).transpose();
  M_jac1(2, 2) += temp(0, 0);
  temp = 2.0 * R_jac2.row(1) * xxF * R.row(1).transpose();
  M_jac2(2, 2) = temp(0, 0);
  temp = -2.0 * R_jac2.row(0) * xyF * R.row(1).transpose();
  M_jac2(2, 2) += temp(0, 0);
  temp = -2.0 * R.row(0) * xyF * R_jac2.row(1).transpose();
  M_jac2(2, 2) += temp(0, 0);
  temp = 2.0 * R_jac2.row(0) * yyF * R.row(0).transpose();
  M_jac2(2, 2) += temp(0, 0);
  temp = 2.0 * R_jac3.row(1) * xxF * R.row(1).transpose();
  M_jac3(2, 2) = temp(0, 0);
  temp = -2.0 * R_jac3.row(0) * xyF * R.row(1).transpose();
  M_jac3(2, 2) += temp(0, 0);
  temp = -2.0 * R.row(0) * xyF * R_jac3.row(1).transpose();
  M_jac3(2, 2) += temp(0, 0);
  temp = 2.0 * R_jac3.row(0) * yyF * R.row(0).transpose();
  M_jac3(2, 2) += temp(0, 0);

  M(1, 0) = M(0, 1);
  M(2, 0) = M(0, 2);
  M(2, 1) = M(1, 2);
  M_jac1(1, 0) = M_jac1(0, 1);
  M_jac1(2, 0) = M_jac1(0, 2);
  M_jac1(2, 1) = M_jac1(1, 2);
  M_jac2(1, 0) = M_jac2(0, 1);
  M_jac2(2, 0) = M_jac2(0, 2);
  M_jac2(2, 1) = M_jac2(1, 2);
  M_jac3(1, 0) = M_jac3(0, 1);
  M_jac3(2, 0) = M_jac3(0, 2);
  M_jac3(2, 1) = M_jac3(1, 2);

  return M;
}

DataType GetSmallestEVwithJacobian(const Mat3 &xxF, const Mat3 &yyF,
                                   const Mat3 &zzF, const Mat3 &xyF,
                                   const Mat3 &yzF, const Mat3 &zxF,
                                   const Vec3 &cayley,
                                   Eigen::Matrix<DataType, 1, 3> &jacobian) {

  Mat3 M_jac1 = Mat3::Zero();
  Mat3 M_jac2 = Mat3::Zero();
  Mat3 M_jac3 = Mat3::Zero();

  Mat3 M = ComposeMwithJacobians(xxF, yyF, zzF, xyF, yzF, zxF, cayley, M_jac1,
                                 M_jac2, M_jac3);

  // Retrieve the smallest Eigenvalue by the following closed form solution.
  // Plus Jacobian.
  DataType b = -M(0, 0) - M(1, 1) - M(2, 2);
  DataType b_jac1 = -M_jac1(0, 0) - M_jac1(1, 1) - M_jac1(2, 2);
  DataType b_jac2 = -M_jac2(0, 0) - M_jac2(1, 1) - M_jac2(2, 2);
  DataType b_jac3 = -M_jac3(0, 0) - M_jac3(1, 1) - M_jac3(2, 2);
  DataType c = -std::pow(M(0, 2), 2) - std::pow(M(1, 2), 2) -
               std::pow(M(0, 1), 2) + M(0, 0) * M(1, 1) + M(0, 0) * M(2, 2) +
               M(1, 1) * M(2, 2);
  DataType c_jac1 =
      -2.0 * M(0, 2) * M_jac1(0, 2) - 2.0 * M(1, 2) * M_jac1(1, 2) -
      2.0 * M(0, 1) * M_jac1(0, 1) + M_jac1(0, 0) * M(1, 1) +
      M(0, 0) * M_jac1(1, 1) + M_jac1(0, 0) * M(2, 2) + M(0, 0) * M_jac1(2, 2) +
      M_jac1(1, 1) * M(2, 2) + M(1, 1) * M_jac1(2, 2);
  DataType c_jac2 =
      -2.0 * M(0, 2) * M_jac2(0, 2) - 2.0 * M(1, 2) * M_jac2(1, 2) -
      2.0 * M(0, 1) * M_jac2(0, 1) + M_jac2(0, 0) * M(1, 1) +
      M(0, 0) * M_jac2(1, 1) + M_jac2(0, 0) * M(2, 2) + M(0, 0) * M_jac2(2, 2) +
      M_jac2(1, 1) * M(2, 2) + M(1, 1) * M_jac2(2, 2);
  DataType c_jac3 =
      -2.0 * M(0, 2) * M_jac3(0, 2) - 2.0 * M(1, 2) * M_jac3(1, 2) -
      2.0 * M(0, 1) * M_jac3(0, 1) + M_jac3(0, 0) * M(1, 1) +
      M(0, 0) * M_jac3(1, 1) + M_jac3(0, 0) * M(2, 2) + M(0, 0) * M_jac3(2, 2) +
      M_jac3(1, 1) * M(2, 2) + M(1, 1) * M_jac3(2, 2);
  DataType d = M(1, 1) * std::pow(M(0, 2), 2) + M(0, 0) * std::pow(M(1, 2), 2) +
               M(2, 2) * std::pow(M(0, 1), 2) - M(0, 0) * M(1, 1) * M(2, 2) -
               2.0 * M(0, 1) * M(1, 2) * M(0, 2);
  DataType d_jac1 =
      M_jac1(1, 1) * std::pow(M(0, 2), 2) +
      M(1, 1) * 2.0 * M(0, 2) * M_jac1(0, 2) +
      M_jac1(0, 0) * std::pow(M(1, 2), 2) +
      M(0, 0) * 2.0 * M(1, 2) * M_jac1(1, 2) +
      M_jac1(2, 2) * std::pow(M(0, 1), 2) +
      M(2, 2) * 2.0 * M(0, 1) * M_jac1(0, 1) -
      M_jac1(0, 0) * M(1, 1) * M(2, 2) - M(0, 0) * M_jac1(1, 1) * M(2, 2) -
      M(0, 0) * M(1, 1) * M_jac1(2, 2) -
      2.0 *
          (M_jac1(0, 1) * M(1, 2) * M(0, 2) + M(0, 1) * M_jac1(1, 2) * M(0, 2) +
           M(0, 1) * M(1, 2) * M_jac1(0, 2));
  DataType d_jac2 =
      M_jac2(1, 1) * std::pow(M(0, 2), 2) +
      M(1, 1) * 2.0 * M(0, 2) * M_jac2(0, 2) +
      M_jac2(0, 0) * std::pow(M(1, 2), 2) +
      M(0, 0) * 2.0 * M(1, 2) * M_jac2(1, 2) +
      M_jac2(2, 2) * std::pow(M(0, 1), 2) +
      M(2, 2) * 2.0 * M(0, 1) * M_jac2(0, 1) -
      M_jac2(0, 0) * M(1, 1) * M(2, 2) - M(0, 0) * M_jac2(1, 1) * M(2, 2) -
      M(0, 0) * M(1, 1) * M_jac2(2, 2) -
      2.0 *
          (M_jac2(0, 1) * M(1, 2) * M(0, 2) + M(0, 1) * M_jac2(1, 2) * M(0, 2) +
           M(0, 1) * M(1, 2) * M_jac2(0, 2));
  DataType d_jac3 =
      M_jac3(1, 1) * std::pow(M(0, 2), 2) +
      M(1, 1) * 2.0 * M(0, 2) * M_jac3(0, 2) +
      M_jac3(0, 0) * std::pow(M(1, 2), 2) +
      M(0, 0) * 2.0 * M(1, 2) * M_jac3(1, 2) +
      M_jac3(2, 2) * std::pow(M(0, 1), 2) +
      M(2, 2) * 2.0 * M(0, 1) * M_jac3(0, 1) -
      M_jac3(0, 0) * M(1, 1) * M(2, 2) - M(0, 0) * M_jac3(1, 1) * M(2, 2) -
      M(0, 0) * M(1, 1) * M_jac3(2, 2) -
      2.0 *
          (M_jac3(0, 1) * M(1, 2) * M(0, 2) + M(0, 1) * M_jac3(1, 2) * M(0, 2) +
           M(0, 1) * M(1, 2) * M_jac3(0, 2));

  DataType s = 2 * std::pow(b, 3) - 9 * b * c + 27 * d;
  DataType t = 4 * std::pow((std::pow(b, 2) - 3 * c), 3);

  DataType s_jac1 = 2.0 * 3.0 * std::pow(b, 2) * b_jac1 - 9.0 * b_jac1 * c -
                    9.0 * b * c_jac1 + 27.0 * d_jac1;
  DataType s_jac2 = 2.0 * 3.0 * std::pow(b, 2) * b_jac2 - 9.0 * b_jac2 * c -
                    9.0 * b * c_jac2 + 27.0 * d_jac2;
  DataType s_jac3 = 2.0 * 3.0 * std::pow(b, 2) * b_jac3 - 9.0 * b_jac3 * c -
                    9.0 * b * c_jac3 + 27.0 * d_jac3;
  DataType t_jac1 = 4.0 * 3.0 * std::pow((std::pow(b, 2) - 3.0 * c), 2) *
                    (2.0 * b * b_jac1 - 3.0 * c_jac1);
  DataType t_jac2 = 4.0 * 3.0 * std::pow((std::pow(b, 2) - 3.0 * c), 2) *
                    (2.0 * b * b_jac2 - 3.0 * c_jac2);
  DataType t_jac3 = 4.0 * 3.0 * std::pow((std::pow(b, 2) - 3.0 * c), 2) *
                    (2.0 * b * b_jac3 - 3.0 * c_jac3);

  DataType alpha = std::acos(s / std::sqrt(t));
  DataType alpha_jac1 =
      -1 / std::sqrt(1.0 - (std::pow(s, 2) / t)) *
      (s_jac1 * std::sqrt(t) - s * 0.5 * std::pow(t, -0.5) * t_jac1) / t;
  DataType alpha_jac2 =
      -1 / std::sqrt(1.0 - (std::pow(s, 2) / t)) *
      (s_jac2 * std::sqrt(t) - s * 0.5 * std::pow(t, -0.5) * t_jac2) / t;
  DataType alpha_jac3 =
      -1 / std::sqrt(1.0 - (std::pow(s, 2) / t)) *
      (s_jac3 * std::sqrt(t) - s * 0.5 * std::pow(t, -0.5) * t_jac3) / t;
  DataType beta = alpha / 3.0;
  DataType beta_jac1 = alpha_jac1 / 3.0;
  DataType beta_jac2 = alpha_jac2 / 3.0;
  DataType beta_jac3 = alpha_jac3 / 3.0;
  DataType y = std::cos(beta);
  DataType y_jac1 = -std::sin(beta) * beta_jac1;
  DataType y_jac2 = -std::sin(beta) * beta_jac2;
  DataType y_jac3 = -std::sin(beta) * beta_jac3;

  DataType r = 0.5 * std::sqrt(t);
  DataType r_jac1 = 0.25 * std::pow(t, -0.5) * t_jac1;
  DataType r_jac2 = 0.25 * std::pow(t, -0.5) * t_jac2;
  DataType r_jac3 = 0.25 * std::pow(t, -0.5) * t_jac3;
  DataType w = std::pow(r, (1.0 / 3.0));
  DataType w_jac1 = (1.0 / 3.0) * std::pow(r, -2.0 / 3.0) * r_jac1;
  DataType w_jac2 = (1.0 / 3.0) * std::pow(r, -2.0 / 3.0) * r_jac2;
  DataType w_jac3 = (1.0 / 3.0) * std::pow(r, -2.0 / 3.0) * r_jac3;

  DataType k = w * y;
  DataType k_jac1 = w_jac1 * y + w * y_jac1;
  DataType k_jac2 = w_jac2 * y + w * y_jac2;
  DataType k_jac3 = w_jac3 * y + w * y_jac3;
  DataType smallestEV = (-b - 2.0 * k) / 3.0;

  DataType smallestEV_jac1 = (-b_jac1 - 2.0 * k_jac1) / 3.0;
  DataType smallestEV_jac2 = (-b_jac2 - 2.0 * k_jac2) / 3.0;
  DataType smallestEV_jac3 = (-b_jac3 - 2.0 * k_jac3) / 3.0;

  jacobian(0, 0) = smallestEV_jac1;
  jacobian(0, 1) = smallestEV_jac2;
  jacobian(0, 2) = smallestEV_jac3;
  return smallestEV;
}

Mat3 GetNumericalJacobians(const Mat3 &R) {
  Vec3 dtheta;
  Vec3 cayley = Rot2Cayley(R);
  Vec3 cayley_perturb;
  Mat3 R_perturb;

  Mat3 jacobian;
  DataType perturb = 1e-5;

  // Get jacobian 1
  for (size_t i = 0; i < 3; i++) {
    dtheta << 0, 0, 0;
    dtheta(i) = perturb;
    R_perturb = ov_core::exp_so3(-dtheta) * R;
    cayley_perturb = Rot2Cayley(R_perturb);
    jacobian.col(i) = (cayley_perturb - cayley) / perturb;
  }
  return jacobian;
}

} // namespace ov_srvins
