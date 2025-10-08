#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_DIR}/../../../../devel/setup.bash

#=============================================================
#=============================================================
#=============================================================

# datasets
datasets=(
   "udel_gore"
    #  "udel_arl"
#    "udel_gore_zupt"
#    "tum_corridor1_512_16_okvis"
)

# location to save log files into
save_path_est="/home/yuxiang/results/srf_sim/algorithms"
save_path_gt="/home/yuxiang/results/srf_sim/truths"


#=============================================================
# Start Monte-Carlo Simulations
#=============================================================

# open a roscore within the script to avoid crash
roscore & disown
sleep 2

# Loop through datasets
for a in "${!datasets[@]}"; do

# Monte Carlo runs for this dataset
for d in {50..199}; do

# start timing
start_time="$(date -u +%s)"

filename="srf"
filename_est="$save_path_est/${filename}/${datasets[a]}/estimate_$d.txt"
filename_gt="$save_path_gt/${datasets[a]}.txt"

roslaunch ov_srvins simulation.launch \
  seed:="$((10#$d + 1))" \
  dataset:="${datasets[a]}.txt" \
  dosave_pose:="true" \
  path_est:="$filename_est" \
  path_gt:="$filename_gt" \
  node_name:="${filename}_${d}" &> /dev/null &

# check this node runs
# check that the node is running
while [ ! -z "$(rostopic info /${filename}_${d}/loop_depth_colored 2>/dev/null)" ]; do
  sleep 0.1
done

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${datasets[a]} - start runing $d took $elapsed seconds";


done
done

# waiting for all the ros nodce to stop running
while [ ! -z "$(rostopic info /tf 2> /dev/null)" ]; do
    echo "Waiting for the run to finish"
    sleep 0.1
done

# kill the roscore after finishing running
pkill roscore
