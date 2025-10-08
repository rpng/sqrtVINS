#!/bin/bash
# setup command
script_dir=$( dirname "${BASH_SOURCE[0]}" )
run_script="${script_dir}/run_init.sh"
eval_script="${script_dir}/eval.sh"

mono_setup="num_kf:=5,do_bg:=false,init_feat_num:=200,bag_durr:=10,max_cameras:=1,win_time:=0.3"
run_name=$1
log_path=$2

bash ${run_script} "euroc_mav" "srf_mono_300ms" "$HOME/results/srf_init/$run_name" "0" "${mono_setup},isrf_num_iter:=20,use_fej:=true"
bash ${eval_script} ${run_name} "init" >> ${log_path}
bash ${eval_script} ${run_name} "vio" >> ${log_path}