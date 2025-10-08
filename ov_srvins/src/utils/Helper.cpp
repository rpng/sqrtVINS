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




#include "utils/Helper.h"
#include "feat/FeatureDatabase.h"
#include "feat/FeatureHelper.h"
#include "utils/DataType.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

namespace ov_srvins {
bool disparity_check(double oldest_time_allowed, double current_time,
                     std::shared_ptr<ov_core::FeatureDatabase> db,
                     DataType max_disparity, bool &is_move) {
  // Check if the disparity check success
  bool check_success = false;

  // Get the disparity statistics from current_time image to all the previous
  // time
  DataType avg_disp = 0.0;
  DataType max_disp = 0.0;
  double exist_oldest_time = INFINITY;
  if (db->map_disp.find(current_time) == db->map_disp.end()) {
    PRINT_INFO(
        YELLOW
        "[Disparity Check]: check failed! No disparity information\n" RESET);
    return false;
  }
  for (auto const &stat : db->map_disp.at(current_time)) {
    double prev_time = stat.first;

    // Start checking from the oldest time allowed
    if (prev_time < oldest_time_allowed && prev_time >= current_time) {
      continue;
    }

    if (prev_time < exist_oldest_time) {
      exist_oldest_time = prev_time;
    }

    avg_disp = stat.second;
    if (avg_disp > max_disp) {
      max_disp = avg_disp;
    }

    // Return if we can't compute the disparity
    if (avg_disp != -1) {
      check_success = true;
      is_move = avg_disp > max_disparity;
      if (is_move) {
        break;
      }
    } else {
      continue;
    }
  }

  if (check_success) {
    PRINT_INFO(YELLOW "[Disparity Check]: window time: %.3f sec, uv disparity "
                      "is %.3f (%.2f static thres)\n" RESET,
               (current_time - oldest_time_allowed), max_disp, max_disparity);
  } else {
    PRINT_INFO(YELLOW
               "[Disparity Check]: check failed! No features to check\n" RESET);
  }

  return check_success;
}

bool imu_jerk_check(double old_time, double new_time,
                    std::shared_ptr<std::vector<ov_core::ImuData>> imu_data,
                    DataType max_accel, bool &is_move, const int win_size,
                    const DataType gravity_mag) {
  is_move = false;

  // Return if we don't have any measurements
  if (imu_data->size() < 2) {
    PRINT_INFO(
        YELLOW
        "[IMU Jerk Check]: check failed! No enough imu to check\n" RESET);
    return false;
  }

  // First lets collect a window of IMU readings from the newest measurement to
  // the oldest
  std::vector<ov_core::ImuData> window_1to0, window_2to1;
  VecX accels = VecX::Zero(win_size);
  DataType accel_avg = 0;
  int win_count = 0;
  for (const ov_core::ImuData &data : *imu_data) {
    if (win_count == win_size) {
      accel_avg = accels.mean();
      if (accel_avg > max_accel) {
        is_move = true;
        break;
      }
      win_count = 0;
    }

    // Skip before the old time
    if (data.timestamp < old_time) {
      continue;
    }
    // Stop if passing the new time
    if (data.timestamp > new_time) {
      break;
    }
    accels(win_count) = abs(data.am.norm() - gravity_mag);
    win_count++;
  }

  // Deal with last window
  if (win_count == win_size && is_move == false) {
    accel_avg = accels.mean();
    if (accel_avg > max_accel) {
      is_move = true;
    }
  }

  // If it is below the threshold and we want to wait till we detect a jerk
  if (accel_avg < max_accel) {
    PRINT_INFO(YELLOW "[IMU Jerk Check] No IMU excitation, below threshold "
                      "%.3f < %.3f\n" RESET,
               accel_avg, max_accel);
  }

  // We should also check that the old state was below the threshold!
  // This is the case when we have started up moving, and thus we need to wait
  // for a period of stationary motion
  if (accel_avg > max_accel) {
    PRINT_INFO(YELLOW "[IMU Jerk Check] Enough IMU excitation, above threshold "
                      "%.3f > %.3f\n" RESET,
               accel_avg, max_accel);
  }
  return true;
}

void gram_schmidt(const Vec3 &gravity_inI, Mat3 &R_GtoI) {
  // This will find an orthogonal vector to gravity which is our local z-axis
  // We need to ensure we normalize after each one such that we obtain unit
  // vectors
  Vec3 z_axis = gravity_inI / gravity_inI.norm();
  Vec3 x_axis, y_axis;
  Vec3 e_1(1.0, 0.0, 0.0);
  Vec3 e_2(0.0, 1.0, 0.0);
  DataType inner1 = e_1.dot(z_axis) / z_axis.norm();
  DataType inner2 = e_2.dot(z_axis) / z_axis.norm();

  if (fabs(inner1) < fabs(inner2)) {
    x_axis = z_axis.cross(e_1);
    x_axis = x_axis / x_axis.norm();
    y_axis = z_axis.cross(x_axis);
    y_axis = y_axis / y_axis.norm();
  } else {
    x_axis = z_axis.cross(e_2);
    x_axis = x_axis / x_axis.norm();
    y_axis = z_axis.cross(x_axis);
    y_axis = y_axis / y_axis.norm();
  }

  // Rotation from our global (where gravity is only along the z-axis) to the
  // local one
  R_GtoI.template block<3, 1>(0, 0) = x_axis;
  R_GtoI.template block<3, 1>(0, 1) = y_axis;
  R_GtoI.template block<3, 1>(0, 2) = z_axis;
}

// Visualize the structure of a matrix with 0 to be zero entity and 1 to be
// non-zero entity
void visualize_matrix_structure(Eigen::Ref<const MatX> A) {
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (abs(A(i, j)) < std::numeric_limits<DataType>::min()) {
        std::cout << "0 ";
      } else {
        std::cout << RED << "1 " << RESET;
      }
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

void get_permutation_matrix(
    Eigen::Ref<const MatX> Hx,
    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> &P) {
  int row_size = Hx.rows();
  P.resize(row_size);
  Eigen::VectorXi order_index;
  order_index.resize(row_size);

  std::vector<std::vector<int>> row_ids;
  // Add an extra group for zero block
  for (int i = 0; i < Hx.cols() + 1; i++) {
    std::vector<int> ids;
    row_ids.push_back(ids);
  }

  // Sweep row by row, and for each row
  // Sweep column by column, and find the first element that is not zero
  // Record its id
  std::vector<int> zero_block_ids;
  for (int i = 0; i < Hx.rows(); i++) {
    bool is_zero_block = true;
    for (int j = 0; j < Hx.cols(); j++) {
      if (abs(Hx(i, j)) > std::numeric_limits<DataType>::min()) {
        row_ids[j].push_back(i);
        is_zero_block = false;
        break;
      }
    }
    if (is_zero_block) {
      zero_block_ids.push_back(i);
    }
  }
  row_ids[Hx.cols() - 1].insert(row_ids[Hx.cols() - 1].end(),
                                zero_block_ids.begin(), zero_block_ids.end());

  // Record the rows, and make the rows as upper-triangle as possible
  int curr_row_id = 0;
  for (const auto &group : row_ids) {
    for (const auto &id : group) {
      order_index(curr_row_id) = id;
      ++curr_row_id;
    }
  }
  P.indices() = order_index;
}

void efficient_QR(Eigen::Ref<MatX> Hx, Eigen::Ref<MatX> r,
                  Eigen::Ref<MatX> Hf) {
  assert(Hx.rows() > 1);
  assert(Hx.rows() == r.rows());
  assert(Hx.rows() == Hf.rows());

  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> P(Hx.rows());
  get_permutation_matrix(Hx, P);
  // Hx = (P.transpose() * Hx).eval();
  // Hf = (P.transpose() * Hf).eval();
  P.transpose().applyThisOnTheLeft(Hx);
  Hf = (P.transpose() * Hf).eval();
  r = (P.transpose() * r).eval();

  Eigen::JacobiRotation<DataType> temp_givens_rotation;
  for (int n = 0; n < Hf.cols(); ++n) {
    for (int m = (int)Hf.rows() - 1; m > n; m--) {
      // Givens matrix G
      temp_givens_rotation.makeGivens(Hf(m - 1, n), Hf(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      //       it is equivalent to applying G to the entire cols
      //       [0:Ho.cols()-1].
      (Hf.block(m - 1, n, 2, Hf.cols() - n))
          .applyOnTheLeft(0, 1, temp_givens_rotation.adjoint());
      int start_col = 0;
      for (; start_col < Hx.cols(); start_col++) {
        if (abs(Hx(m - 1, start_col)) > std::numeric_limits<DataType>::min())
          break;
      }

      // Leverage sparsity for MSCKF feature structure
      if (start_col < Hx.cols())
        (Hx.block(m - 1, start_col, 2, Hx.cols() - start_col))
            .applyOnTheLeft(0, 1, temp_givens_rotation.adjoint());
      (r.block(m - 1, 0, 2, 1))
          .applyOnTheLeft(0, 1, temp_givens_rotation.adjoint());
    }
  }
}

void efficient_QR(Eigen::Ref<MatX> Hx, Eigen::Ref<MatX> r) {
  assert(Hx.rows() > 1);
  assert(Hx.rows() == r.rows());

  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> P(Hx.rows());
  get_permutation_matrix(Hx, P);
  P.transpose().applyThisOnTheLeft(Hx);
  r = (P.transpose() * r).eval();

  int num_cols = Hx.cols();
  int num_rows = Hx.rows();

  int op_rows;
  VecX temp_vector2;
  VecX temp_vector1(num_cols);
  DataType beta;
  DataType tau;

  int total_op_cols = std::min(num_rows, num_cols);
  for (int k = 0; k < total_op_cols; ++k) {
    // Find the row id that has the first non-zero element from bottom to top
    int j = Hx.rows() - 1;
    op_rows = 1;
    for (; j > k; --j) {
      bool found_first_nonzero =
          abs(Hx(j, k)) > std::numeric_limits<DataType>::min();
      if (found_first_nonzero) {
        op_rows = j - k + 1;
        break;
      }
    }
    // skip if this row has already been upper triangular
    if (op_rows == 1)
      continue;

    Hx.col(k).segment(k, op_rows).makeHouseholder(temp_vector2, tau, beta);
    Hx.block(k, k, op_rows, num_cols - k)
        .applyHouseholderOnTheLeft(temp_vector2, tau, temp_vector1.data());
    r.block(k, 0, op_rows, 1)
        .applyHouseholderOnTheLeft(temp_vector2, tau, temp_vector1.data());
  }
}

// Perform Householder QR factorization that leverages the sparsity in A
// @param A The input matrix to be factorized
void efficient_QR(Eigen::Ref<MatX> A) {
  assert(A.rows() > 1);

  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> P(A.rows());
  get_permutation_matrix(A, P);
  P.transpose().applyThisOnTheLeft(A);

  int num_cols = A.cols();
  int op_rows;
  VecX temp_vector2;
  VecX temp_vector1(num_cols);
  DataType beta;
  DataType tau;

  int total_op_cols = std::min(A.cols(), A.rows());
  for (int k = 0; k < total_op_cols; ++k) {
    // Find the row id that has the first non-zero element from bottom to top
    int j = A.rows() - 1;
    op_rows = 1;
    for (; j > k; --j) {
      bool found_first_nonzero =
          abs(A(j, k)) > std::numeric_limits<DataType>::min();
      if (found_first_nonzero) {
        op_rows = j - k + 1;
        break;
      }
    }
    // skip if this row has already been upper triangular
    if (op_rows == 1)
      continue;

    A.col(k).segment(k, op_rows).makeHouseholder(temp_vector2, tau, beta);
    A.block(k, k, op_rows, A.cols() - k)
        .applyHouseholderOnTheLeft(temp_vector2, tau, temp_vector1.data());
  }
}

void efficient_QR_givens(Eigen::Ref<MatX> A) {
  assert(A.rows() > 1);

  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> P(A.rows());
  get_permutation_matrix(A, P);
  Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      A_tmp = P.transpose() * A;
  Eigen::JacobiRotation<DataType> temp_givens_rotation;

  for (int n = 0; n < A_tmp.cols(); ++n) {
    for (int m = (int)A_tmp.rows() - 1; m > n; m--) {
      if (abs(A_tmp(m, n)) < std::numeric_limits<DataType>::min()) {
        A_tmp(m, n) = 0;
        continue;
      }
      temp_givens_rotation.makeGivens(A_tmp(m - 1, n), A_tmp(m, n));
      (A_tmp.block(m - 1, n, 2, A_tmp.cols() - n))
          .applyOnTheLeft(0, 1, temp_givens_rotation.adjoint());
    }
  }
  A.swap(A_tmp);
}

double get_condition_number(Eigen::Ref<const MatX> A_original) {
  Eigen::MatrixXd A = A_original.template cast<double>();
  // Check condition number
  Eigen::BDCSVD<Eigen::MatrixXd> svd(A);
  Eigen::MatrixXd singular_values = svd.singularValues();
  return singular_values(0, 0) / singular_values(singular_values.rows() - 1, 0);
}

void triangular_matrix_inverse_solver(const Eigen::Transpose<MatX> &U,
                                      Eigen::Ref<MatX> X, int op_rows) {

  int rows = U.rows();
  int row_start = rows - op_rows;
  const int op_rows_first_block = rows - (rows / op_rows) * op_rows;

  for (int i = 0; i < rows / op_rows + 1; i++) {

    if (row_start >= 0)
      U.block(row_start, row_start, op_rows, op_rows)
          .template triangularView<Eigen::Upper>()
          .solveInPlace(
              X.block(row_start, row_start, op_rows, op_rows * (i + 1)));
    else {
      if (op_rows_first_block) {
        U.block(0, 0, op_rows_first_block, op_rows_first_block)
            .template triangularView<Eigen::Upper>()
            .solveInPlace(X.block(0, 0, op_rows_first_block, rows));
      }
      break;
    }

    if (row_start - op_rows >= 0) {
      X.block(row_start - op_rows, row_start, op_rows, op_rows * (i + 1))
          .noalias() -=
          U.block(row_start - op_rows, row_start, op_rows, op_rows * (i + 1)) *
          X.bottomRightCorner(op_rows * (i + 1), op_rows * (i + 1))
              .template triangularView<Eigen::Upper>();
    } else {
      if (op_rows_first_block) {
        X.block(0, row_start, op_rows_first_block, op_rows * (i + 1))
            .noalias() -=
            U.block(0, row_start, op_rows_first_block, op_rows * (i + 1)) *
            X.bottomRightCorner(op_rows * (i + 1), op_rows * (i + 1))
                .template triangularView<Eigen::Upper>();
      }
    }
    row_start -= op_rows;
  }
}

void triangular_matrix_multiplier_UU(Eigen::Ref<const MatX> U1,
                                     Eigen::Ref<const MatX> U2,
                                     Eigen::Ref<MatX> U1_U2, int op_rows) {
  int rows = U1.rows();
  int row_start = 0;
  while (row_start != rows) {
    if (row_start + op_rows > rows) {
      op_rows = rows - row_start;
    }
    U1_U2.middleRows(row_start, op_rows).rightCols(rows - row_start) =
        U1.block(row_start, row_start, op_rows, op_rows)
            .template triangularView<Eigen::Upper>() *
        U2.middleRows(row_start, op_rows).rightCols(rows - row_start);
    if (rows - row_start - op_rows != 0) {
      U1_U2.middleRows(row_start, op_rows)
          .rightCols(rows - row_start - op_rows) +=
          U1.block(row_start, row_start + op_rows, op_rows,
                   rows - row_start - op_rows) *
          U2.bottomRightCorner(rows - row_start - op_rows,
                               rows - row_start - op_rows)
              .template triangularView<Eigen::Upper>();
    }
    row_start += op_rows;
  }
}

void triangular_matrix_multiplier_LLT(Eigen::Ref<const MatX> L,
                                      Eigen::Ref<MatX> LLT, int op_rows) {
  int rows = L.rows();
  int row_start = 0;
  auto LT = L.transpose();
  while (row_start != rows) {
    if (row_start + op_rows > rows) {
      op_rows = rows - row_start;
    }
    if (row_start != 0) {
      LLT.middleRows(row_start, op_rows).leftCols(row_start) =
          L.middleRows(row_start, op_rows).leftCols(row_start) *
          LT.topLeftCorner(row_start, row_start)
              .template triangularView<Eigen::Upper>();
      LLT.middleRows(row_start, op_rows).middleCols(row_start, op_rows) =
          L.middleRows(row_start, op_rows).leftCols(row_start) *
          LT.topRows(row_start).middleCols(row_start, op_rows);
    }
    LLT.middleRows(row_start, op_rows).middleCols(row_start, op_rows) +=
        L.middleRows(row_start, op_rows)
            .middleCols(row_start, op_rows)
            .template triangularView<Eigen::Lower>() *
        LT.middleRows(row_start, op_rows).middleCols(row_start, op_rows);
    row_start += op_rows;
  }
  LLT = LLT.template selfadjointView<Eigen::Lower>();
}

void matrix_multiplier_ATA(Eigen::Ref<MatX> A, Eigen::Ref<MatX> ATA) {
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> P;
  get_permutation_matrix(A, P);
  A = P.transpose() * A;

  int num_cols = A.cols();
  int num_rows = A.rows();
  int curr_row_id = 0;
  int op_rows = 0;
  int op_cols = 0;
  for (int i = 0; i < num_cols; i++) {
    if (op_rows == 0)
      op_cols = num_cols - i;

    for (int j = curr_row_id + op_rows; j < num_rows; j++) {
      if (A(j, i) != 0.0) {
        op_rows++;
      } else {
        break;
      }
    }

    if (op_rows == 0) {
      continue;
    }

    ATA.bottomRightCorner(op_cols, op_cols)
        .template selfadjointView<Eigen::Upper>()
        .rankUpdate(
            A.middleRows(curr_row_id, op_rows).rightCols(op_cols).transpose(),
            1.0);

    curr_row_id += op_rows;
    op_rows = 0;
  }
  ATA = ATA.template selfadjointView<Eigen::Upper>();
}

Eigen::Matrix<DataType, 3, 2>
get_gravity_Jacobian(DataType alpha, DataType beta, DataType mag) {
  Eigen::Matrix<DataType, 3, 2> J = Eigen::Matrix<DataType, 3, 2>::Zero();
  J(0, 0) = -sin(alpha) * sin(beta);
  J(0, 1) = cos(alpha) * cos(beta);
  J(1, 0) = cos(alpha) * sin(beta);
  J(1, 1) = sin(alpha) * cos(beta);
  J(2, 1) = -sin(beta);
  J *= mag;
  return J;
}

Vec3 get_gravity(DataType alpha, DataType beta, DataType mag) {
  Vec3 g;
  g << cos(alpha) * sin(beta), sin(alpha) * sin(beta), cos(beta);
  g *= mag;
  return g;
}

std::vector<ov_core::ImuData>
select_imu_readings(const std::vector<ov_core::ImuData> &imu_data, double time0,
                    double time1, bool warn) {

  // Our vector imu readings
  std::vector<ov_core::ImuData> prop_data;

  // Ensure we have some measurements in the first place!
  if (imu_data.empty()) {
    if (warn)
      PRINT_WARNING(YELLOW "select_imu_readings(): No IMU measurements. "
                           "IMU-CAMERA are likely messed up!!!\n" RESET);
    return prop_data;
  }

  // Loop through and find all the needed measurements to propagate with
  // Note we split measurements based on the given state time, and the update
  // timestamp
  for (size_t i = 0; i < imu_data.size() - 1; i++) {

    // START OF THE INTEGRATION PERIOD
    // If the next timestamp is greater then our current state time
    // And the current is not greater then it yet...
    // Then we should "split" our current IMU measurement
    if (imu_data.at(i + 1).timestamp > time0 &&
        imu_data.at(i).timestamp < time0) {
      ov_core::ImuData data =
          interpolate_data(imu_data.at(i), imu_data.at(i + 1), time0);
      prop_data.push_back(data);
      // PRINT_DEBUG("propagation #%d = CASE 1 = %.3f => %.3f\n", (int)i,
      // data.timestamp - prop_data.at(0).timestamp,
      //             time0 - prop_data.at(0).timestamp);
      continue;
    }

    // MIDDLE OF INTEGRATION PERIOD
    // If our imu measurement is right in the middle of our propagation period
    // Then we should just append the whole measurement time to our propagation
    // vector
    if (imu_data.at(i).timestamp >= time0 &&
        imu_data.at(i + 1).timestamp <= time1) {
      prop_data.push_back(imu_data.at(i));
      // PRINT_DEBUG("propagation #%d = CASE 2 = %.3f\n", (int)i,
      // imu_data.at(i).timestamp - prop_data.at(0).timestamp);
      continue;
    }

    // END OF THE INTEGRATION PERIOD
    // If the current timestamp is greater then our update time
    // We should just "split" the NEXT IMU measurement to the update time,
    // NOTE: we add the current time, and then the time at the end of the
    // interval (so we can get a dt) NOTE: we also break out of this loop, as
    // this is the last IMU measurement we need!
    if (imu_data.at(i + 1).timestamp > time1) {
      // If we have a very low frequency IMU then, we could have only recorded
      // the first integration (i.e. case 1) and nothing else In this case, both
      // the current IMU measurement and the next is greater than the desired
      // intepolation, thus we should just cut the current at the desired time
      // Else, we have hit CASE2 and this IMU measurement is not past the
      // desired propagation time, thus add the whole IMU reading
      if (imu_data.at(i).timestamp > time1 && i == 0) {
        // This case can happen if we don't have any imu data that has occured
        // before the startup time This means that either we have dropped IMU
        // data, or we have not gotten enough. In this case we can't propgate
        // forward in time, so there is not that much we can do.
        break;
      } else if (imu_data.at(i).timestamp > time1) {
        ov_core::ImuData data =
            interpolate_data(imu_data.at(i - 1), imu_data.at(i), time1);
        prop_data.push_back(data);
        // PRINT_DEBUG("propagation #%d = CASE 3.1 = %.3f => %.3f\n",
        // (int)i,imu_data.at(i).timestamp-prop_data.at(0).timestamp,imu_data.at(i).timestamp-time0);
      } else {
        prop_data.push_back(imu_data.at(i));
        // PRINT_DEBUG("propagation #%d = CASE 3.2 = %.3f => %.3f\n",
        // (int)i,imu_data.at(i).timestamp-prop_data.at(0).timestamp,imu_data.at(i).timestamp-time0);
      }
      // If the added IMU message doesn't end exactly at the camera time
      // Then we need to add another one that is right at the ending time
      if (prop_data.at(prop_data.size() - 1).timestamp != time1) {
        ov_core::ImuData data =
            interpolate_data(imu_data.at(i), imu_data.at(i + 1), time1);
        prop_data.push_back(data);
        // PRINT_DEBUG("propagation #%d = CASE 3.3 = %.3f => %.3f\n",
        // (int)i,data.timestamp-prop_data.at(0).timestamp,data.timestamp-time0);
      }
      break;
    }
  }

  // Check that we have at least one measurement to propagate with
  if (prop_data.empty()) {
    if (warn)
      PRINT_WARNING(
          YELLOW "select_imu_readings(): No IMU measurements to propagate "
                 "with (%d of 2). IMU-CAMERA are likely messed up!!!\n" RESET,
          (int)prop_data.size());
    return prop_data;
  }

  // Loop through and ensure we do not have an zero dt values
  // This would cause the noise covariance to be Infinity
  for (size_t i = 0; i < prop_data.size() - 1; i++) {
    if (std::abs(prop_data.at(i + 1).timestamp - prop_data.at(i).timestamp) <
        1e-12) {
      if (warn)
        PRINT_WARNING(YELLOW "select_imu_readings(): Zero DT between IMU "
                             "reading %d and %d, removing it!\n" RESET,
                      (int)i, (int)(i + 1));
      prop_data.erase(prop_data.begin() + i);
      i--;
    }
  }

  // Check that we have at least one measurement to propagate with
  if (prop_data.size() < 2) {
    if (warn)
      PRINT_WARNING(
          YELLOW "select_imu_readings(): No IMU measurements to propagate "
                 "with (%d of 2). IMU-CAMERA are likely messed up!!!\n" RESET,
          (int)prop_data.size());
    return prop_data;
  }

  // Success :D
  return prop_data;
}

ov_core::ImuData interpolate_data(const ov_core::ImuData &imu_1,
                                  const ov_core::ImuData &imu_2,
                                  double timestamp) {
  // time-distance lambda
  double lambda =
      (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
  // PRINT_DEBUG("lambda - %d\n", lambda);
  // interpolate between the two times
  ov_core::ImuData data;
  data.timestamp = timestamp;
  data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
  data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
  return data;
}

} // namespace ov_srvins
