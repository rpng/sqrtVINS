#!/usr/bin/env bash
# Usage: bash run_abtest.sh <run_name_prefix> <mode>

script_dir=$( dirname "${BASH_SOURCE[0]}" )
init_script="${script_dir}/run_init_abtest.sh"
vio_script="${script_dir}/run_eth_abtest.sh"
timing_script="${script_dir}/run_timing_abtest.sh"

timestamp=$(date +"%Y-%m-%d_%H-%M-%S")
run_name=$1_${timestamp}
log_folder=$HOME/ablogs
log_path=$HOME/ablogs/${run_name}_report.txt
mode=$2

if [ ! -d $log_folder ]; then
  mkdir -p $log_folder
fi


touch ${log_path}
export MPLBACKEND=Agg

if [[ $mode == "init" || $mode == "all" ]]; then
  printf "==========================Init AB Test==========================\n" >> ${log_path}
  bash ${init_script} ${run_name} ${log_path}
fi

if [[ $mode == "vio" || $mode == "all" ]]; then
  printf "==========================VIO AB Test==========================\n" >> ${log_path}
  bash ${vio_script} ${run_name} ${log_path}
fi

if [[ $mode == "timing" || $mode == "all" ]]; then
  printf "==========================TIMING AB Test==========================\n" >> ${log_path}
  bash ${timing_script} ${run_name} ${log_path}
fi
