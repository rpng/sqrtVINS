#!/usr/bin/env bash

# setup command
script_dir=$( dirname "${BASH_SOURCE[0]}" )
run_script="${script_dir}/run_init.sh"


win_time=(
    "0.3"
    "0.5"
)


init_feat_num=(
    "50"
    "100"
    "200"
    "300"
    "400"
    "800"
)

num_iter=(
    "0"
    "1"
    "5"
    "10"
    "15"
    "20"
)


dataset=(
    "euroc_mav"
    "tum_vi"
)

# num_iteration
for a in "${!win_time[@]}"; do

for e in "${!dataset[@]}"; do

run_name="num_iter_${win_time[$a]}_${dataset[$e]}"
config_path="${script_dir}/../../../config/${dataset[e]}/estimator_config.yaml"

for c in "${!num_iter[@]}"; do

setup="num_kf:=5,do_bg:=false,init_feat_num:=200,,bag_durr:=10,max_cameras:=1,init_num_iter:=${num_iter[c]},win_time:=${win_time[a]}"
sys_name="srf_${num_iter[c]}iter"
bash ${run_script} ${dataset[e]} $sys_name "$HOME/results/srf_init/$run_name" "0" "${setup}"

done 

eval_script="${script_dir}/eval.sh"
bash ${eval_script} $run_name "init"

done

done


for e in "${!dataset[@]}"; do

run_name="num_iter_0.1_${dataset[$e]}_stereo"
config_path="${script_dir}/../../../config/${dataset[e]}/estimator_config.yaml"

for c in "${!init_feat_num[@]}"; do

setup="num_kf:=3,do_bg:=false,init_feat_num:=${init_feat_num[c]},bag_durr:=10,max_cameras:=2,init_num_iter:=10,win_time:=0.1"
sys_name="srf_${num_iter[c]}iter"
bash ${run_script} ${dataset[e]} $sys_name "$HOME/results/srf_init/$run_name" "0" "${setup}"

done

eval_script="${script_dir}/eval.sh"
bash ${eval_script} $run_name "init"

done


# num_feats
for a in "${!win_time[@]}"; do

for e in "${!dataset[@]}"; do

run_name="num_feat_${win_time[$a]}_${dataset[$e]}"
config_path="${script_dir}/../../../config/${dataset[e]}/estimator_config.yaml"

for c in "${!init_feat_num[@]}"; do

setup="num_kf:=5,do_bg:=false,init_feat_num:=${init_feat_num[c]},bag_durr:=10,max_cameras:=1,init_num_iter:=10,win_time:=${win_time[a]}"
sys_name="srf_${init_feat_num[c]}feat"
bash ${run_script} ${dataset[e]} $sys_name "$HOME/results/srf_init/$run_name" "0" "${setup}"

done

eval_script="${script_dir}/eval.sh"
bash ${eval_script} $run_name "init"

done

done


for e in "${!dataset[@]}"; do

run_name="num_feat_0.1_${dataset[$e]}_stereo"
config_path="${script_dir}/../../../config/${dataset[e]}/estimator_config.yaml"

for c in "${!num_iter[@]}"; do

setup="num_kf:=3,do_bg:=false,init_feat_num:=${init_feat_num[c]},bag_durr:=10,max_cameras:=2,init_num_iter:=10,win_time:=0.1"
sys_name="srf_${init_feat_num[c]}feat"
bash ${run_script} ${dataset[e]} $sys_name "$HOME/results/srf_init/$run_name" "0" "${setup}"

done

eval_script="${script_dir}/eval.sh"
bash ${eval_script} $run_name "init"

done



feat_num=(
    "20"
    "30"
    "40"
    "50"
    "60"
    "70"
    "80"
)

# num_feats in ba
for a in "${!win_time[@]}"; do

for e in "${!dataset[@]}"; do

run_name="ba_feat_${win_time[$a]}_${dataset[$e]}"
config_path="${script_dir}/../../../config/${dataset[e]}/estimator_config.yaml"

for c in "${!feat_num[@]}"; do

setup="num_kf:=5,do_bg:=false,init_feat_num:=200,init_max_feat:=${feat_num[c]},bag_durr:=10,max_cameras:=1,init_num_iter:=10,win_time:=${win_time[a]}"
sys_name="srf_${feat_num[c]}feat"
bash ${run_script} ${dataset[e]} $sys_name "$HOME/results/srf_init/$run_name" "0" "${setup}"

done

eval_script="${script_dir}/eval.sh"
bash ${eval_script} $run_name "init"

done

done


for e in "${!dataset[@]}"; do

run_name="ba_feat_0.1_${dataset[$e]}_stereo"
config_path="${script_dir}/../../../config/${dataset[e]}/estimator_config.yaml"

for c in "${!feat_num[@]}"; do

setup="num_kf:=3,do_bg:=false,init_feat_num:=200,init_max_feat:=${feat_num[c]},bag_durr:=10,max_cameras:=2,init_num_iter:=10,win_time:=0.1"
sys_name="srf_${feat_num[c]}feat"
bash ${run_script} ${dataset[e]} $sys_name "$HOME/results/srf_init/$run_name" "0" "${setup}"

done

eval_script="${script_dir}/eval.sh"
bash ${eval_script} $run_name "init"


done




