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




#include "utils/NoiseManager.h"
#include "utils/DataType.h"
#include "utils/print.h"

namespace ov_srvins {

void NoiseManager::print() {
  PRINT_DEBUG("  - gyroscope_noise_density: %.6f\n", sigma_w);
  PRINT_DEBUG("  - accelerometer_noise_density: %.5f\n", sigma_a);
  PRINT_DEBUG("  - gyroscope_random_walk: %.7f\n", sigma_wb);
  PRINT_DEBUG("  - accelerometer_random_walk: %.6f\n", sigma_ab);
}

} // namespace ov_srvins
