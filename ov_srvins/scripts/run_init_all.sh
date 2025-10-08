#!/bin/bash
# setup command
script_dir=$( dirname "${BASH_SOURCE[0]}" )
run_script="${script_dir}/run_init.sh"

mono_setup="num_kf:=5,do_bg:=false,init_feat_num:=200,bag_durr:=10,max_cameras:=1,win_time:=0.3"
run_name="test_300ms"

bash ${run_script} "euroc_mav" "srf_10iter_fej" "$HOME/results/srf_init/$run_name" "0" "${mono_setup},isrf_num_iter:=20,use_fej:=true"