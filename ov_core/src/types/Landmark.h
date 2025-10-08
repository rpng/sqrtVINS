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





#ifndef OV_TYPE_TYPE_LANDMARK_H
#define OV_TYPE_TYPE_LANDMARK_H

#include "LandmarkRepresentation.h"
#include "Vec.h"
#include "utils/colors.h"
#include "utils/print.h"

namespace ov_type {

/**
 * @brief Type that implements a persistent SLAM feature.
 *
 * We store the feature ID that should match the IDs in the trackers.
 * Additionally if this is an anchored representation we store what clone
 * timestamp this is anchored from and what camera. If this features should be
 * marginalized its flag can be set and during cleanup it will be removed.
 */
class Landmark : public Vec {

public:
  /// Default constructor (feature is a Vec of size 3 or Vec of size 1)
  Landmark(int dim) : Vec(dim) {}

  /// Feature ID of this landmark (corresponds to frontend id)
  size_t featid;

  /// What unique camera stream this slam feature was observed from
  int unique_camera_id = -1;

  /// What camera ID our pose is anchored in!! By default the first measurement
  /// is the anchor.
  int anchor_cam_id = -1;

  /// Timestamp of anchor clone
  double anchor_clone_timestamp = -1;

  /// Boolean if this landmark should be marginalized out
  bool should_marg = false;

  /// Number of times the update has failed for this feature (we should remove
  /// if it fails a couple times!)
  int update_fail_count = 0;

  /// First normalized uv coordinate bearing of this measurement (used for
  /// single depth representation)
  Vec3 uv_norm_zero;

  /// First estimate normalized uv coordinate bearing of this measurement (used
  /// for single depth representation)
  Vec3 uv_norm_zero_fej;

  /// What feature representation this feature currently has
  LandmarkRepresentation::Representation feat_representation;

  /**
   * @brief Overrides the default vector update rule
   * We want to selectively update the FEJ value if we are using an anchored
   * representation.
   * @param dx Additive error state correction
   */
  void update(const VecX &dx) override {
    // Update estimate
    assert(dx.rows() == _size);
    set_value(_value + dx);
    // Ensure we are not near zero in the z-direction
    // if
    // (LandmarkRepresentation::is_relative_representation(_feat_representation)
    // && _value(_value.rows() - 1) < 1e-8) {
    //  PRINT_DEBUG(YELLOW "WARNING DEPTH %.8f BECAME CLOSE TO ZERO IN
    //  UPDATE!!!\n" RESET, _value(_value.rows() - 1)); should_marg = true;
    // }
  }

  /**
   * @brief Will return the position of the feature in the global frame of
   * reference.
   * @param getfej Set to true to get the landmark FEJ value
   * @return Position of feature either in global or anchor frame
   */
  Vec3 get_xyz(bool getfej) const;

  /**
   * @brief Will set the current value based on the representation.
   * @param p_FinG Position of the feature either in global or anchor frame
   * @param isfej Set to true to set the landmark FEJ value
   */
  void set_from_xyz(Vec3 p_FinG, bool isfej);
};
} // namespace ov_type

#endif // OV_TYPE_TYPE_LANDMARK_H
