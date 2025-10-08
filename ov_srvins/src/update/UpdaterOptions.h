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





#ifndef OV_SRVINS_UPDATER_OPTIONS_H
#define OV_SRVINS_UPDATER_OPTIONS_H

#include "utils/DataType.h"
#include "utils/print.h"

namespace ov_srvins {

/**
 * @brief Struct which stores general updater options
 */
struct UpdaterOptions {

  UpdaterOptions(DataType chi2_mult = 1, DataType sigma_pix_ = 1)
      : chi2_multipler(chi2_mult), sigma_pix(sigma_pix_) {
    sigma_pix_inv = 1 / sigma_pix;
    sigma_pix_sq = std::pow(sigma_pix, 2);
    sigma_pix_sq_inv = 1 / sigma_pix_sq;
  }

  /// What chi-squared multipler we should apply
  DataType chi2_multipler = 1;

  /// Noise sigma for our raw pixel measurements
  DataType sigma_pix = 1;
  DataType sigma_pix_inv = 1.0 / sigma_pix;

  /// Covariance for our raw pixel measurements
  DataType sigma_pix_sq = 1;
  DataType sigma_pix_sq_inv = 1.0 / sigma_pix_sq;

  /// Nice print function of what parameters we have loaded
  void print() {
    PRINT_DEBUG("    - chi2_multipler: %.1f\n", chi2_multipler);
    PRINT_DEBUG("    - sigma_pix: %.2f\n", sigma_pix);
  }
};

} // namespace ov_srvins

#endif // OV_SRVINS_UPDATER_OPTIONS_H