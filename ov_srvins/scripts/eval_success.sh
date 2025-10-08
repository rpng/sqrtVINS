#!/bin/bash

script_dir=$( dirname "${BASH_SOURCE[0]}" )
source ${script_dir}/../../../../devel/setup.bash
folder_name=$1
echo "folder_name: $folder_name"

pos_error=(
  0.1
  0.2
  0.3
  0.4
  0.5
  # 0.6
  # 0.7
  # 0.8
  # 0.9
  # 1.0
)

for a in "${!pos_error[@]}"; do

rosrun ov_eval error_comparison posyawsingle $HOME/results/srf_init/$folder_name/vio/truth $HOME/results/srf_init/$folder_name/vio/algorithms 0 500 ${pos_error[a]} &> "$HOME/results/srf_init/$folder_name/vio/results_traj_${pos_error[a]}.txt"
cat "$HOME/results/srf_init/$folder_name/vio/results_traj_${pos_error[a]}.txt"

done
