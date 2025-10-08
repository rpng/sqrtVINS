#!/usr/bin/env bash

# setup command
script_dir=$( dirname "${BASH_SOURCE[0]}" )
run_script="${script_dir}/run_init_serial.sh"


win_time=(
    "0.1"
    # "0.15"
    # "0.2"
    # "0.25"
    # "0.3"
    "0.5"
    # "0.75"
    # "1.0"
)

dataset=(
    # "euroc_mav"
    "tum_vi"
)

# mono
for e in "${!dataset[@]}"; do

run_name="time_nocal_${dataset[$e]}"

for a in "${!win_time[@]}"; do

config_path="${script_dir}/../../../config/${dataset[e]}/estimator_config.yaml"

num_kf=5
if [ "${win_time[a]}" == "0.1" ]; then
    num_kf=3
elif [ "${win_time[a]}" == "0.15" ]; then
    num_kf=4
fi

setup_ba="num_kf:=$num_kf,do_bg:=false,init_feat_num:=200,bag_durr:=10,max_cameras:=1,init_num_iter:=10,do_calibration:=false,win_time:=${win_time[a]}"
sys_name2="srf_10iter_${win_time[a]}"

bash ${run_script} ${dataset[e]} $sys_name2 "$HOME/results/srf_init/$run_name" "0" "${setup_ba}"

done 

eval_script="${script_dir}/eval.sh"
bash ${eval_script} $run_name "init"
bash ${eval_script} $run_name "vio"

done


# stereo
for e in "${!dataset[@]}"; do

run_name="time_nocal_${dataset[$e]}_stereo"

for a in "${!win_time[@]}"; do

config_path="${script_dir}/../../../config/${dataset[e]}/estimator_config.yaml"

num_kf=5
if [ "${win_time[a]}" == "0.1" ]; then
    num_kf=3
elif [ "${win_time[a]}" == "0.15" ]; then
    num_kf=4
fi

setup_ba="num_kf:=$num_kf,do_bg:=false,init_feat_num:=200,bag_durr:=10,max_cameras:=2,init_num_iter:=10,do_calibration:=false,win_time:=${win_time[a]}"
sys_name2="srf_10iter_${win_time[a]}"

bash ${run_script} ${dataset[e]} $sys_name2 "$HOME/results/srf_init/$run_name" "0" "${setup_ba}"

done 

eval_script="${script_dir}/eval.sh"
bash ${eval_script} $run_name "init"
bash ${eval_script} $run_name "vio"

done


