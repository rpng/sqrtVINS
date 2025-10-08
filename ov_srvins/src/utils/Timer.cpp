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




#include "Timer.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iomanip>
#include <iostream>
#include <string>

namespace ov_srvins {

void Timer::tic() {
  time_start_ = boost::posix_time::microsec_clock::local_time();
}

void Timer::toc(const std::string &func_name, bool output) {
  time_end_ = boost::posix_time::microsec_clock::local_time();
  duration_ += (time_end_ - time_start_).total_microseconds() * 1e-3;
  if (output) {
    std::cout << func_name << " takes " << std::setprecision(5)
              << (time_end_ - time_start_).total_microseconds() * 1e-3 << " ms"
              << std::endl;
  }
}

void Timer::report(const std::string &func_name) {
  std::cout << func_name << " takes " << std::setprecision(5) << duration_
            << " ms" << std::endl;
}

void Timer::reset() { duration_ = 0.0; }

} // namespace ov_srvins
