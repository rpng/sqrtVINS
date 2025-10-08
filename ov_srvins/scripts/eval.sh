#!/bin/bash

script_dir=$( dirname "${BASH_SOURCE[0]}" )
source ${script_dir}/../../../../devel/setup.bash
folder_name=$1
mode=$2
echo "folder_name: $folder_name"


if [ $mode == "init" ]; then
  pos_error=0.1
  is_init_window=1
  align_mode="posyawsingle"
elif [ $mode == "vio" ]; then
  pos_error=0.5
  is_init_window=0  
  align_mode="posyawsingle"
fi

echo "folder_name: $folder_name"
echo "mode: $mode"
echo "pos_error: $pos_error"
echo "is_init_window: $is_init_window"


DISPLAY=:0.0 && rosrun ov_eval error_comparison $align_mode $HOME/results/srf_init/$folder_name/$mode/truth $HOME/results/srf_init/$folder_name/$mode/algorithms $is_init_window 500 $pos_error &> "$HOME/results/srf_init/$folder_name/$mode/results_traj.txt"
cat "$HOME/results/srf_init/$folder_name/$mode/results_traj.txt"

if [ $mode == "init" ]; then
  pos_error=0.1
  is_init_window=1
  align_mode="sim3"
  DISPLAY=:0.0 && rosrun ov_eval error_comparison $align_mode $HOME/results/srf_init/$folder_name/$mode/truth $HOME/results/srf_init/$folder_name/$mode/algorithms $is_init_window 500 $pos_error &> "$HOME/results/srf_init/$folder_name/$mode/results_traj(sim3).txt"
  cat "$HOME/results/srf_init/$folder_name/$mode/results_traj(sim3).txt"
fi
