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




//
// Created by yuxiang on 5/18/23.
//

#ifndef OV_SRVINS_TIMER_H
#define OV_SRVINS_TIMER_H
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
namespace ov_srvins {

class Timer {
public:
  Timer() = default;

  void tic();

  /**
   * @brief Stop the timer
   *
   * @param func_name Name of the function being timed
   * @param output    Whether to printout the time taken immediately
   */
  void toc(const std::string &func_name = "func", bool output = false);

  /**
   * @brief Report the elapsed time for the function
   *
   * @param func_name Name of the function being reported
   */
  void report(const std::string &func_name);

  /**
   * @brief Reset the duration of the timer
   */
  void reset();

private:
  boost::posix_time::ptime time_start_, time_end_;
  double duration_ = 0.0;
};

} // namespace ov_srvins

#endif // OV_SRVINS_TIMER_H
