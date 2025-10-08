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





#include <Eigen/Eigen>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>

#include "calc/ResultTrajectory.h"
#include "utils/Loader.h"
#include "utils/colors.h"
#include "utils/print.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif

int main(int argc, char **argv) {

  // Verbosity setting
  ov_core::Printer::setPrintLevel("INFO");

  // Ensure we have a path
  if (argc < 4) {
    PRINT_ERROR(
        RED
        "ERROR: Please specify an align mode, folder, and algorithms\n" RESET);
    PRINT_ERROR(RED
                "ERROR: ./error_comparison <align_mode> <folder_groundtruth> "
                "<folder_algorithms> [<is_init_window>]\n" RESET);
    PRINT_ERROR(
        RED
        "ERROR: rosrun ov_eval error_comparison <align_mode> "
        "<folder_groundtruth> <folder_algorithms> [<is_init_window>]\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  double ori_err = 10;
  double pos_err = 0.5;
  if (argc > 5) {
    ori_err = std::stod(argv[5]);
    pos_err = std::stod(argv[6]);
  }

  bool is_init_window = false;
  if (argc > 4) {
    assert(argv[4][0] == '0' ||
           argv[4][0] == '1'); // Just send 0 or 1 to represent bool
    is_init_window = (bool)std::stoi(argv[4]);
  }

  // List the groundtruth files in this folder
  std::string path_gts(argv[2]);
  std::vector<boost::filesystem::path> path_groundtruths;
  for (const auto &p :
       boost::filesystem::recursive_directory_iterator(path_gts)) {
    if (p.path().extension() == ".txt") {
      path_groundtruths.push_back(p.path());
    }
  }
  std::sort(path_groundtruths.begin(), path_groundtruths.end());

  // Try to load our paths
  for (size_t i = 0; i < path_groundtruths.size(); i++) {
    // Load it!
    std::vector<double> times;
    std::vector<Eigen::Matrix<double, 7, 1>> poses;
    std::vector<Eigen::Matrix3d> cov_ori, cov_pos;
    ov_eval::Loader::load_data(path_groundtruths.at(i).string(), times, poses,
                               cov_ori, cov_pos);
    // Print its length and stats
    double length = ov_eval::Loader::get_total_length(poses);
    PRINT_INFO("[COMP]: %d poses in %s => length of %.2f meters\n",
               (int)times.size(), path_groundtruths.at(i).filename().c_str(),
               length);
  }

  // Get the algorithms we will process
  // Also create empty statistic objects for each of our datasets
  std::string path_algos(argv[3]);
  std::vector<boost::filesystem::path> path_algorithms;
  for (const auto &entry : boost::filesystem::directory_iterator(path_algos)) {
    if (boost::filesystem::is_directory(entry)) {
      path_algorithms.push_back(entry.path());
    }
  }
  std::sort(path_algorithms.begin(), path_algorithms.end());

  //===============================================================================
  //===============================================================================
  //===============================================================================

  // ATE summery information
  std::map<std::string,
           std::vector<std::pair<ov_eval::Statistics, ov_eval::Statistics>>>
      algo_ate;
  std::map<std::string, std::vector<ov_eval::Statistics>> algo_scale;
  for (const auto &p : path_algorithms) {
    std::vector<std::pair<ov_eval::Statistics, ov_eval::Statistics>> temp;
    std::vector<ov_eval::Statistics> temp2;
    for (size_t i = 0; i < path_groundtruths.size(); i++) {
      temp.push_back({ov_eval::Statistics(), ov_eval::Statistics()});
      temp2.push_back(ov_eval::Statistics());
    }
    algo_ate.insert({p.filename().string(), temp});
    algo_ate.insert({p.filename().string(), temp});
    algo_scale.insert({p.filename().string(), temp2});
  }

  // Relative pose error segment lengths
  std::vector<double> segments;
  if (is_init_window) {
    segments = {0.1, 0.2, 0.3, 0.4};
  } else {
    segments = {8.0, 16.0, 24.0, 32.0, 40.0, 48.0};
    // std::vector<double> segments = {7.0, 14.0, 21.0, 28.0, 35.0};
    // std::vector<double> segments = {10.0, 25.0, 50.0, 75.0, 120.0};
    // std::vector<double> segments = {5.0, 15.0, 30.0, 45.0, 60.0};
    // std::vector<double> segments = {40.0, 60.0, 80.0, 100.0, 120.0};
  }

  // The overall RPE error calculation for each algorithm type
  std::map<
      std::string,
      std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>>>
      algo_rpe;
  for (const auto &p : path_algorithms) {
    std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>> temp;
    for (const auto &len : segments) {
      temp.insert({len, {ov_eval::Statistics(), ov_eval::Statistics()}});
    }
    algo_rpe.insert({p.filename().string(), temp});
  }

  //===============================================================================
  //===============================================================================
  //===============================================================================

  // Loop through each algorithm type
  std::vector<std::vector<std::string>> path_names;
  for (size_t i = 0; i < path_algorithms.size(); i++) {

    // Debug print
    PRINT_DEBUG("======================================\n");
    PRINT_DEBUG("[COMP]: processing %s algorithm\n",
                path_algorithms.at(i).filename().c_str());

    // Get the list of datasets this algorithm records
    std::map<std::string, boost::filesystem::path> path_algo_datasets;
    for (auto &entry :
         boost::filesystem::directory_iterator(path_algorithms.at(i))) {
      if (boost::filesystem::is_directory(entry)) {
        path_algo_datasets.insert(
            {entry.path().filename().string(), entry.path()});
      }
    }

    // Loop through our list of groundtruth datasets, and see if we have it
    for (size_t j = 0; j < path_groundtruths.size(); j++) {

      // Check if we have runs for this dataset
      if (path_algo_datasets.find(path_groundtruths.at(j).stem().string()) ==
          path_algo_datasets.end()) {
        PRINT_ERROR(
            RED "[COMP]: %s dataset does not have any runs for %s!!!!!\n" RESET,
            path_algorithms.at(i).filename().c_str(),
            path_groundtruths.at(j).stem().c_str());
        continue;
      }

      // Debug print
      PRINT_DEBUG("[COMP]: processing %s algorithm => %s dataset\n",
                  path_algorithms.at(i).filename().c_str(),
                  path_groundtruths.at(j).stem().c_str());

      // Errors for this specific dataset (i.e. our averages over the total
      // runs)
      ov_eval::Statistics ate_dataset_ori;
      ov_eval::Statistics ate_dataset_pos;
      ov_eval::Statistics scale_dataset;

      std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>>
          rpe_dataset;
      for (const auto &len : segments) {
        rpe_dataset.insert(
            {len, {ov_eval::Statistics(), ov_eval::Statistics()}});
      }

      // Loop though the different runs for this dataset
      std::vector<std::string> file_paths;
      for (auto &entry :
           boost::filesystem::directory_iterator(path_algo_datasets.at(
               path_groundtruths.at(j).stem().string()))) {
        if (entry.path().extension() != ".txt")
          continue;
        file_paths.push_back(entry.path().string());
      }
      std::sort(file_paths.begin(), file_paths.end(),
                [](const std::string &a, const std::string &b) {
                  // Extract the last number from the file names
                  size_t lastUnderscoreA = a.find_last_of('_');
                  size_t lastUnderscoreB = b.find_last_of('_');
                  if (lastUnderscoreA != std::string::npos &&
                      lastUnderscoreB != std::string::npos) {
                    int numA = std::stoi(a.substr(lastUnderscoreA + 1));
                    int numB = std::stoi(b.substr(lastUnderscoreB + 1));
                    return numA < numB;
                  }
                  return a < b; // Default to lexicographic comparison if no
                                // number is found
                });

      path_names.push_back(file_paths);

      // Now loop through the sorted vector
      for (auto &path_esttxt : file_paths) {
        // Our paths
        std::string dataset = path_groundtruths.at(j).stem().string();
        std::string path_gttxt = path_groundtruths.at(j).string();

        // Create our trajectory object
        ov_eval::ResultTrajectory traj(path_esttxt, path_gttxt, argv[1]);
        // if (!traj.is_valid()) {
        //   continue; // Error message printed in ResultTrajectory constructor
        // }

        // Calculate ATE error for this dataset
        ov_eval::Statistics error_ori, error_pos;
        traj.calculate_ate(error_ori, error_pos);
        ate_dataset_ori.values.push_back(error_ori.rmse);
        ate_dataset_pos.values.push_back(error_pos.rmse);

        // Calculate RPE error for this dataset
        std::map<double, std::pair<ov_eval::Statistics, ov_eval::Statistics>>
            error_rpe;
        if (traj.is_valid()) {
          traj.calculate_rpe(segments, error_rpe);
          for (const auto &elm : error_rpe) {
            rpe_dataset.at(elm.first).first.values.insert(
                rpe_dataset.at(elm.first).first.values.end(),
                elm.second.first.values.begin(), elm.second.first.values.end());
            rpe_dataset.at(elm.first).first.timestamps.insert(
                rpe_dataset.at(elm.first).first.timestamps.end(),
                elm.second.first.timestamps.begin(),
                elm.second.first.timestamps.end());
            rpe_dataset.at(elm.first).second.values.insert(
                rpe_dataset.at(elm.first).second.values.end(),
                elm.second.second.values.begin(),
                elm.second.second.values.end());
            rpe_dataset.at(elm.first).second.timestamps.insert(
                rpe_dataset.at(elm.first).second.timestamps.end(),
                elm.second.second.timestamps.begin(),
                elm.second.second.timestamps.end());
            scale_dataset.values.push_back(traj.scale_est2gt);
          }
        }
      }

      // Compute our mean ATE score
      ate_dataset_ori.calculate();
      ate_dataset_pos.calculate();
      scale_dataset.calculate();

      // Print stats for this specific dataset
      std::string prefix =
          (ate_dataset_ori.mean > 10 || ate_dataset_pos.mean > 10) ? RED : "";
      PRINT_DEBUG(
          "%s\tATE: mean_ori = %.3f | mean_pos = %.3f (%d runs)\n" RESET,
          prefix.c_str(), ate_dataset_ori.mean, ate_dataset_pos.mean,
          (int)ate_dataset_pos.values.size());
      PRINT_DEBUG("\tATE: std_ori  = %.3f | std_pos  = %.3f\n",
                  ate_dataset_ori.std, ate_dataset_pos.std);
      for (auto &seg : rpe_dataset) {
        seg.second.first.calculate();
        seg.second.second.calculate();
        // PRINT_DEBUG("\tRPE: seg %d - mean_ori = %.3f | mean_pos = %.3f (%d
        // samples)\n",(int)seg.first,seg.second.first.mean,seg.second.second.mean,(int)seg.second.second.values.size());
        PRINT_DEBUG("\tRPE: seg %d - median_ori = %.4f | median_pos = %.4f (%d "
                    "samples)\n",
                    (int)seg.first, seg.second.first.median,
                    seg.second.second.median,
                    (int)seg.second.second.values.size());
        // PRINT_DEBUG("RPE: seg %d - std_ori  = %.3f | std_pos  =
        // %.3f\n",(int)seg.first,seg.second.first.std,seg.second.second.std);
      }

      // Update the global ATE error stats
      std::string algo = path_algorithms.at(i).filename().string();
      algo_ate.at(algo).at(j).first = ate_dataset_ori;
      algo_ate.at(algo).at(j).second = ate_dataset_pos;
      algo_scale.at(algo).at(j) = scale_dataset;

      // Update the global RPE error stats
      for (const auto &elm : rpe_dataset) {
        algo_rpe.at(algo).at(elm.first).first.values.insert(
            algo_rpe.at(algo).at(elm.first).first.values.end(),
            elm.second.first.values.begin(), elm.second.first.values.end());
        algo_rpe.at(algo).at(elm.first).first.timestamps.insert(
            algo_rpe.at(algo).at(elm.first).first.timestamps.end(),
            elm.second.first.timestamps.begin(),
            elm.second.first.timestamps.end());
        algo_rpe.at(algo).at(elm.first).second.values.insert(
            algo_rpe.at(algo).at(elm.first).second.values.end(),
            elm.second.second.values.begin(), elm.second.second.values.end());
        algo_rpe.at(algo).at(elm.first).second.timestamps.insert(
            algo_rpe.at(algo).at(elm.first).second.timestamps.end(),
            elm.second.second.timestamps.begin(),
            elm.second.second.timestamps.end());
      }
    }
  }
  PRINT_DEBUG("\n\n");

  // Successful rate
  PRINT_INFO("============================================\n");
  PRINT_INFO("SUCCESS RATE\n");
  PRINT_INFO("============================================\n");

  for (auto &algo : algo_ate) {
    int count = 0;
    std::cout << std::endl;

    int total_runs = 0;
    int total_success_runs = 0;
    std::string algoname = algo.first;
    boost::replace_all(algoname, "_", "\\_");
    PRINT_INFO(algoname.c_str());

    for (auto &seg : algo.second) {
      int data_runs = 0;
      int data_success_runs = 0;
      std::string gtname = path_groundtruths.at(count).stem().string();
      boost::replace_all(gtname, "_", "\\_");
      std::cout << std::endl;
      std::cout << gtname << " ";
      // int sum_ct = 0;
      if (seg.first.values.empty() || seg.second.values.empty()) {
        PRINT_INFO(" & - ");
      } else {
        std::cout << " & ";
        for (size_t i = 0; i < seg.first.values.size(); i++) {
          bool is_success = seg.first.values[i] < ori_err &&
                            seg.second.values[i] < pos_err &&
                            seg.first.values[i] && seg.second.values[i];
          if (!is_success)
            std::cout << RED;
          std::cout << " " << std::endl;
          std::cout << path_names.at(count).at(i) << " ";
          std::cout << seg.first.values[i] << " " << seg.second.values[i]
                    << " ";
          if (is_success) {
            data_success_runs++;
            total_success_runs++;
            std::cout << "1";
          } else {
            std::cout << "0";
          }

          if (!is_success)
            std::cout << RESET;

          data_runs++;
          total_runs++;
        }

        // PRINT_INFO(" & %.3f ", 100.0 * data_success_runs / data_runs);
      }
      count++;
    }
    PRINT_INFO(" & %.3f \\\\\n", 100.0 * total_success_runs / total_runs);
  }
  PRINT_INFO("============================================\n");

  // Done!
  return EXIT_SUCCESS;
}
