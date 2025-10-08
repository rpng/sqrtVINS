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




#ifndef OV_SRVINS_HELPER_H
#define OV_SRVINS_HELPER_H

#include "feat/FeatureDatabase.h"
#include "utils/DataType.h"
#include "utils/sensor_data.h"

namespace ov_srvins {
bool disparity_check(double oldest_time_allowed, double current_time,
                     std::shared_ptr<ov_core::FeatureDatabase> db,
                     DataType max_disparity, bool &is_move);

/**
 * [imu_jerk_check description]
 *
 * @param  old_time   Oldest time to check in database
 * @param  new_time   Newest time to check in database
 * @param  imu_data   IMU data to check for jerk
 * @param  max_accel  Maximum allowed acceleration
 * @param  is_move    Flag to indicate if there is movement
 * @param  win_size   Window size for jerk detection
 * @param  gravity_mag Gravity magnitude for normalization
 *
 * @return  bool      True if jerk is checked successfully, false otherwise
 */
bool imu_jerk_check(double old_time, double new_time,
                    std::shared_ptr<std::vector<ov_core::ImuData>> imu_data,
                    DataType max_accel, bool &is_move, const int win_size = 10,
                    const DataType gravity_mag = 9.81);

/**
 * Perform Gram-Schmidt decomposition to obtain the rotation matrix given
 * gravity vector
 * @param gravity_inI The gravity direction in IMU frame
 * @param R_GtoI     The output rotation matrix from global frame to IMU frame
 */
void gram_schmidt(const Vec3 &gravity_inI, Mat3 &R_GtoI);

/**
 * @brief Visualize the structure of a matrix
 * @param A The input matrix to be visualized
 */
void visualize_matrix_structure(Eigen::Ref<const MatX> A);

/**
 * @brief the permutation matrix to permutate matrix Hx to reorder the rows to
 * make it close to upper triangular. We sort the rows based on the first
 * non-zero column index ascendingly.
 * @param Hx The input matrix to be permutated
 * @param P The output permutation matrix
 */
void get_permutation_matrix(
    Eigen::Ref<const MatX> Hx,
    Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> &P);

/**
 * @brief Efficient Givens QR factorization for MSCKF feature, which also
 * leverages sparsity in Hx
 * @param Hx The Jacobian matrix wrt the state
 * @param r The residual vector
 * @param Hf The feature Jacobian matrix
 */
void efficient_QR(Eigen::Ref<MatX> Hx, Eigen::Ref<MatX> r, Eigen::Ref<MatX> Hf);

/**
 * @brief Perform efficient QR factorization that leverages the sparsity in Hx
 * @param Hx The Jacobian matrix wrt the state
 * @param r The residual vector
 */
void efficient_QR(Eigen::Ref<MatX> Hx, Eigen::Ref<MatX> r);

/**
 * @brief Perform efficient QR factorization that leverages the sparsity in A
 * @param A The input matrix to be factorized
 */
void efficient_QR(Eigen::Ref<MatX> A);

/**
 * @brief Perform Givens QR factorization that leverages the sparsity in A
 * @param A The input matrix to be factorized
 */
void efficient_QR_givens(Eigen::Ref<MatX> A);

/**
 * @brief Reverse the order of a matrix
 * @param A The input matrix to be reversed
 * @param do_column If true, reverse the columns; otherwise, reverse the rows
 */
inline void reverse_mat(Eigen::Ref<MatX> A, bool do_column = true) {
  if (do_column) {
    A.rowwise().reverseInPlace();
  } else {
    A.colwise().reverseInPlace();
  }
}

/**
 * @brief Reverse the order of a vector
 * @param v The input vector to be reversed
 */
inline void reverse_vec(Eigen::Ref<MatX> v) { v.reverseInPlace(); }

/**
 * @brief Get the condition number of a matrix
 * @param A_original The input matrix to be analyzed
 * @return  double  The condition number of the matrix
 */
double get_condition_number(Eigen::Ref<const MatX> A_original);

/**
 * @brief Block-based triangular matrix inverse solver to boost efficiency.
 *
 * @param U Upper triangular matrix (a transpose view of lower-triangular
 * matrix). Note: we keep this werid input here instead of MatX to avoid
 * behavior change with Eigen's internal implementation between
 * Eigen::Transpose<MatX> and MatX as input
 * @param X Matrix to store the result
 * @param op_rows Maximal number of rows to process in each block
 */
void triangular_matrix_inverse_solver(const Eigen::Transpose<MatX> &U,
                                      Eigen::Ref<MatX> X, int op_rows = 32);

/**
 * @brief Matrix multiplication for two upper triangular matrices
 * @param U1 First upper triangular matrix
 * @param U2 Second upper triangular matrix
 * @param U1_U2 Resulting upper triangular matrix
 * @param op_rows Maximal number of rows to process in each block
 */
void triangular_matrix_multiplier_UU(Eigen::Ref<const MatX> U1,
                                     Eigen::Ref<const MatX> U2,
                                     Eigen::Ref<MatX> U1_U2, int op_rows = 32);

/**
 * @brief Matrix multiplication for lower triangular matrix with its transpose
 * @param L Lower triangular matrix
 * @param LLT Resulting symmetric matrix
 * @param op_rows Maximal number of rows to process in each block
 */
void triangular_matrix_multiplier_LLT(Eigen::Ref<const MatX> L,
                                      Eigen::Ref<MatX> LLT, int op_rows = 32);
/**
 * @brief Matrix outer product of A
 * @param A Input matrix
 * @param ATA Resulting symmetric matrix
 */
void matrix_multiplier_ATA(Eigen::Ref<MatX> A, Eigen::Ref<MatX> ATA);

/**
 * @brief Get the gravity Jacobian w.r.t. the orientation parameters
 * @param alpha Orientation parameter alpha
 * @param beta Orientation parameter beta
 * @return MatX The gravity Jacobian matrix
 */
Eigen::Matrix<DataType, 3, 2> get_gravity_Jacobian(DataType alpha,
                                                   DataType beta, DataType mag);

/**
 * @brief Get the gravity vector given the orientation parameters and gravity
 * magnitude
 *
 * @param  alpha  Orientation parameter alpha
 * @param  beta   Orientation parameter beta
 * @param  mag    Gravity magnitude
 * @return  Vec3    Gravity vector in IMU frame
 */
Vec3 get_gravity(DataType alpha, DataType beta, DataType mag);

/**
 * @brief Helper function that given current imu data, will select imu
 * readings between the two times.
 *
 * This will create measurements that we will integrate with, and an extra
 * measurement at the end. We use the @ref interpolate_data() function to
 * "cut" the imu readings at the begining and end of the integration. The
 * timestamps passed should already take into account the time offset values.
 *
 * @param imu_data IMU data we will select measurements from
 * @param time0 Start timestamp
 * @param time1 End timestamp
 * @param warn If we should warn if we don't have enough IMU to propagate with
 * (e.g. fast prop will get warnings otherwise)
 * @return Vector of measurements (if we could compute them)
 */
std::vector<ov_core::ImuData>
select_imu_readings(const std::vector<ov_core::ImuData> &imu_data, double time0,
                    double time1, bool warn = true);

/**
 * @brief Nice helper function that will linearly interpolate between two imu
 * messages.
 *
 * This should be used instead of just "cutting" imu messages that bound the
 * camera times Give better time offset if we use this function, could try
 * other orders/splines if the imu is slow.
 *
 * @param imu_1 imu at begining of interpolation interval
 * @param imu_2 imu at end of interpolation interval
 * @param timestamp Timestamp being interpolated to
 */
ov_core::ImuData interpolate_data(const ov_core::ImuData &imu_1,
                                  const ov_core::ImuData &imu_2,
                                  double timestamp);

} // namespace ov_srvins
#endif // OV_SRVINS_HELPER_H
