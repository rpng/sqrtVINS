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




#include "CameraPoseBuffer.h"

namespace ov_srvins {

CameraPoseBuffer::CameraPoseBuffer(int max_clone_size, int max_camera_size)
    : max_clone_size_(max_clone_size) {
  int max_size = max_clone_size * max_camera_size;
  buffer_.resize(max_size);
}

PoseData &CameraPoseBuffer::get_buffer_unsafe(int camera_id, double timestamp) {
  int idx = get_buffer_index(camera_id, timestamp);
  assert(idx >= 0 && idx < static_cast<int>(buffer_.size())); // Debug check
  return buffer_[idx];
}

void CameraPoseBuffer::remove_timestamp(double timestamp) {
  if (timestamps_id_.find(timestamp) != timestamps_id_.end()) {
    timestamps_id_.erase(timestamp);
  }
}

void CameraPoseBuffer::add_timestamp(double timestamp) {
  if (timestamps_id_.find(timestamp) == timestamps_id_.end()) {
    timestamps_id_[timestamp] = curr_clone_id_;
    curr_clone_id_++;
    curr_clone_id_ %= max_clone_size_;
  }
}

int CameraPoseBuffer::get_buffer_index(int camera_id, double timestamp) const {
  if (timestamps_id_.find(timestamp) == timestamps_id_.end()) {
    return -1;
  }
  int clone_id = timestamps_id_.at(timestamp);
  return max_clone_size_ * camera_id + clone_id;
}

} // namespace ov_srvins