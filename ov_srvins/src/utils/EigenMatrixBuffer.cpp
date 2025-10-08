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




#include "utils/EigenMatrixBuffer.h"
#include <iostream>
namespace ov_srvins {
bool EigenMatrixBuffer::set_size(int rows, int cols) {
  if (rows > max_rows_ || cols > max_cols_) {
    return false;
  }
  rows_ = rows;
  cols_ = cols;
  return true;
}

Eigen::Ref<MatX> EigenMatrixBuffer::get() {
  return buffer_.block(0, 0, rows_, cols_);
}

Eigen::Ref<MatX> EigenMatrixBuffer::get_rows(int start_row, int num_rows) {
  return buffer_.block(start_row, 0, num_rows, cols_);
}

Eigen::Ref<MatX> EigenMatrixBuffer::get_block(int start_row, int start_col,
                                              int num_rows, int num_cols) {
  return buffer_.block(start_row, start_col, num_rows, num_cols);
}

void EigenMatrixBuffer::reset() {
  get_block(0, 0, rows_, cols_).setZero();
  rows_ = 0;
  cols_ = 0;
}

void EigenMatrixBuffer::append_rows(Eigen::Ref<const MatX> mat) {
  if (rows_ + mat.rows() > max_rows_ || cols_ != mat.cols()) {
    std::abort();
  }
  buffer_.block(rows_, 0, mat.rows(), mat.cols()) = mat;
  rows_ += mat.rows();
}

void EigenMatrixBuffer::append_block_rows_with_order(
    Eigen::Ref<const MatX> mat, const std::vector<std::pair<Id, Size>> &order) {
  if (rows_ + mat.rows() > max_rows_ || mat.cols() > max_cols_) {
    std::abort();
  }

  int curr_cols = 0;
  int global_var_id = 0;
  int global_var_size = 0;
  int num_rows = mat.rows();
  for (size_t i = 0; i < order.size(); i++) {
    global_var_id = order[i].first;
    global_var_size = order[i].second;
    buffer_.block(rows_, global_var_id, num_rows, global_var_size) =
        mat.block(0, curr_cols, num_rows, global_var_size);
    curr_cols += global_var_size;
  }
  if (curr_cols != mat.cols()) {
    std::abort();
  }
  rows_ += mat.rows();
}

void EigenMatrixBuffer::append_left_rows(Eigen::Ref<const MatX> mat) {
  if (rows_ + mat.rows() > max_rows_ || cols_ < mat.cols()) {
    std::abort();
  }
  buffer_.block(rows_, 0, mat.rows(), mat.cols()) = mat;
  rows_ += mat.rows();
}

void EigenMatrixBuffer::append_top_cols_and_resize(Eigen::Ref<const MatX> mat) {
  if (rows_ > max_rows_ || cols_ + mat.cols() > max_cols_) {
    std::abort();
  }
  buffer_.block(0, cols_, mat.rows(), mat.cols()) = mat;
  rows_ = std::max(rows_, static_cast<int>(mat.rows()));
  cols_ += mat.cols();
}

} // namespace ov_srvins
