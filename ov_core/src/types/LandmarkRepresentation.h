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





#ifndef OV_TYPE_LANDMARKREPRESENTATION_H
#define OV_TYPE_LANDMARKREPRESENTATION_H

#include <string>

namespace ov_type {

/**
 * @brief Class has useful feature representation types
 */
class LandmarkRepresentation {

public:
  /**
   * @brief What feature representation our state can use
   */
  enum Representation {
    GLOBAL_3D,
    GLOBAL_FULL_INVERSE_DEPTH,
    ANCHORED_3D,
    ANCHORED_FULL_INVERSE_DEPTH,
    ANCHORED_MSCKF_INVERSE_DEPTH,
    UNKNOWN
  };

  /**
   * @brief Returns a string representation of this enum value.
   * Used to debug print out what the user has selected as the representation.
   * @param feat_representation  Representation we want to check
   * @return String version of the passed enum
   */
  static inline std::string as_string(Representation feat_representation) {
    if (feat_representation == GLOBAL_3D)
      return "GLOBAL_3D";
    if (feat_representation == GLOBAL_FULL_INVERSE_DEPTH)
      return "GLOBAL_FULL_INVERSE_DEPTH";
    if (feat_representation == ANCHORED_3D)
      return "ANCHORED_3D";
    if (feat_representation == ANCHORED_FULL_INVERSE_DEPTH)
      return "ANCHORED_FULL_INVERSE_DEPTH";
    if (feat_representation == ANCHORED_MSCKF_INVERSE_DEPTH)
      return "ANCHORED_MSCKF_INVERSE_DEPTH";
    return "UNKNOWN";
  }

  /**
   * @brief Returns a string representation of this enum value.
   * Used to debug print out what the user has selected as the representation.
   * @param feat_representation String we want to find the enum of
   * @return Representation, will be "unknown" if we coun't parse it
   */
  static inline Representation
  from_string(const std::string &feat_representation) {
    if (feat_representation == "GLOBAL_3D")
      return GLOBAL_3D;
    if (feat_representation == "GLOBAL_FULL_INVERSE_DEPTH")
      return GLOBAL_FULL_INVERSE_DEPTH;
    if (feat_representation == "ANCHORED_3D")
      return ANCHORED_3D;
    if (feat_representation == "ANCHORED_FULL_INVERSE_DEPTH")
      return ANCHORED_FULL_INVERSE_DEPTH;
    if (feat_representation == "ANCHORED_MSCKF_INVERSE_DEPTH")
      return ANCHORED_MSCKF_INVERSE_DEPTH;
    return UNKNOWN;
  }

  /**
   * @brief Helper function that checks if the passed feature representation is
   * a relative or global
   * @param feat_representation Representation we want to check
   * @return True if it is a relative representation
   */
  static inline bool
  is_relative_representation(Representation feat_representation) {
    return (
        feat_representation == Representation::ANCHORED_3D ||
        feat_representation == Representation::ANCHORED_FULL_INVERSE_DEPTH ||
        feat_representation == Representation::ANCHORED_MSCKF_INVERSE_DEPTH);
  }

private:
  /**
   * All function in this class should be static.
   * Thus an instance of this class cannot be created.
   */
  LandmarkRepresentation(){};
};

} // namespace ov_type

#endif // OV_TYPE_LANDMARKREPRESENTATION_H