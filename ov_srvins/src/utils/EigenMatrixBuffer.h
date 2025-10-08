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




#ifndef OV_SRVINS_EIGEN_MATRIX_BUFFER_H
#define OV_SRVINS_EIGEN_MATRIX_BUFFER_H
#include "utils/DataType.h"

namespace ov_srvins {
class EigenMatrixBuffer {
public:
  typedef int Id;
  typedef int Size;

  EigenMatrixBuffer() = default;

  explicit EigenMatrixBuffer(int max_rows, int max_cols)
      : max_rows_(max_rows), max_cols_(max_cols),
        buffer_(MatX::Zero(max_rows, max_cols)) {}
  ~EigenMatrixBuffer() = default;

  bool set_size(int rows, int cols);

  Eigen::Ref<MatX> get();

  Eigen::Ref<MatX> get_rows(int start_row, int num_rows);

  Eigen::Ref<MatX> get_block(int start_row, int start_col, int num_rows,
                             int num_cols);

  void reset();

  void append_rows(Eigen::Ref<const MatX> mat);

  void
  append_block_rows_with_order(Eigen::Ref<const MatX> mat,
                               const std::vector<std::pair<Id, Size>> &order);

  void append_left_rows(Eigen::Ref<const MatX> mat);

  void append_top_cols_and_resize(Eigen::Ref<const MatX> mat);

  int rows() const { return rows_; }

  int cols() const { return cols_; }

private:
  int max_rows_ = 0;
  int max_cols_ = 0;

  int rows_ = 0;
  int cols_ = 0;

  MatX buffer_;
};

} // namespace ov_srvins
#endif // OV_SRVINS_EIGEN_MATRIX_BUFFER_H