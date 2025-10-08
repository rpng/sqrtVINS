#!/usr/bin/env bash

# setup command
script_dir=$( dirname "${BASH_SOURCE[0]}" )
run_script="${script_dir}/run_init.sh"


win_time=(
    # "0.3"
    "0.5"
)


init_max_slam=(
    "0"
    "50"
)

dataset=(
    "euroc_mav"
    # "tum_vi"
)


# num_feats
for a in "${!win_time[@]}"; do

for e in "${!dataset[@]}"; do

run_name="do_slam_${win_time[$a]}_${dataset[$e]}"
config_path="${script_dir}/../../../config/${dataset[e]}/estimator_config.yaml"

for c in "${!init_max_slam[@]}"; do

setup="num_kf:=5,do_bg:=false,init_max_slam:=${init_max_slam[c]},bag_durr:=10,max_cameras:=1,init_num_iter:=100,win_time:=${win_time[a]}"
sys_name="srf_${init_max_slam[c]}feat"
bash ${run_script} ${dataset[e]} $sys_name "$HOME/results/srf_init/$run_name" "0" "${setup}"

done

eval_script="${script_dir}/eval.sh"
bash ${eval_script} $run_name "init"
bash ${eval_script} $run_name "vio"

done

done

