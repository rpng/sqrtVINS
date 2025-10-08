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




#ifndef OV_SRVINS_CAMERA_POSE_BUFFER_H
#define OV_SRVINS_CAMERA_POSE_BUFFER_H

#include "feat/FeatureDatabase.h"
#include "utils/DataType.h"
#include "utils/sensor_data.h"
#include <functional>
#include <optional>

namespace ov_srvins {

struct PoseData {
  Mat3 R_GtoC;
  Vec3 p_CinG;
};

class CameraPoseBuffer {
public:
  CameraPoseBuffer(int max_clone_size, int max_camera_size);

  PoseData &get_buffer_unsafe(int camera_id, double timestamp);

  void remove_timestamp(double timestamp);

  void add_timestamp(double timestamp);

private:
  int get_buffer_index(int camera_id, double timestamp) const;

  int curr_clone_id_ = 0;

  int max_clone_size_;

  std::map<double, int> timestamps_id_;
  std::vector<PoseData> buffer_;
};

} // namespace ov_srvins
#endif // OV_SRVINS_CAMERA_POSE_BUFFER_H
