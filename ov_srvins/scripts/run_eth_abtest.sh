#!/usr/bin/env bash

# Assign the input variable to a meaningful name
folder="$1"
log_path="$2"
total_runs=33

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_DIR}/../../../../devel/setup.bash

#=============================================================
#=============================================================
#=============================================================

# dataset locations
bagnames=(
  "V1_01_easy"
  "V1_02_medium"
  "V1_03_difficult"
  "V2_01_easy"
  "V2_02_medium"
  "V2_03_difficult"
  "MH_01_easy"
  "MH_02_easy"
  "MH_03_medium"
  "MH_04_difficult"
  "MH_05_difficult"
)

# how far we should start into the dataset
# this can be used to skip the initial sections
bagstarttimes=(
  "0"
  "0"
  "0"
  "0"
  "0"
  "0"
  "40" # 40
  "35" # 35 # original was 35
  "5" # 10
  "10" # 17
  "6" # 18
)


# location to save log files into
save_path_est="$HOME/results/$folder/eurocmav/algorithms"
save_path_time="$HOME/results/$folder/eurocmav/timing"
bag_path="$HOME/datasets/euroc_mav"

# open a roscore within the script to avoid crash
roscore & disown
sleep 2

# Maximum cpu usage (0-100, 100 means maximal cpu load)
# If having multiple runs and enable opencv multithreading, we should better use a small number(e.g. 50) to avoid failure in some runs
max_cpu_usage=95

#=============================================================
#=============================================================
#=============================================================
count=0
# start timing
start_time="$(date -u +%s)"

for mode in "lite" "msckf" "srvins"; do

# Loop through all datasets
for a in "${!bagnames[@]}"; do


filename="srf_${mode}"
filename_est="$save_path_est/${filename}/${bagnames[a]}/estimate.txt"
filename_time="$save_path_time/${filename}/${bagnames[a]}/time.txt"
node_name="srf_${a}${mode}"
((count++))

# only start a new process when the cpu load is allowed
while true; do
  # Program Bar
  # Update progress bar
  current_time="$(date -u +%s)"
  elapsed="$(($current_time-$start_time))"
  percentage=$((count * 100 / total_runs))
  remaining_time=$((elapsed * total_runs / count - elapsed))
  hours=$(printf "%02d" $((remaining_time / 3600)))
  minutes=$(printf "%02d" $((remaining_time % 3600 / 60)))
  seconds=$(printf "%02d" $((remaining_time % 60)))
  echo -ne "Progress: [$percentage%] Remaining Time: ${hours}:${minutes}:${seconds} \r"

  # Get current CPU usage
  cpu_usage=$(top -b -n1 | grep "Cpu(s)" | awk '{print $2 + $4}')
  # Check if the CPU usage is greater than the maximum allowed
  if awk "BEGIN {exit !($cpu_usage < $max_cpu_usage)}"; then
    break
  fi
  sleep 1
done

# run our ROS launch file (note we send console output to terminator)
# subscribe=live pub, serial=read from bag (fast)
roslaunch ov_srvins serial.launch \
  max_cameras:="1" \
  use_stereo:="false" \
  config:="euroc_mav" \
  config_path:=" ${SCRIPT_DIR}/../../config/euroc_mav/estimator_config_${mode}.yaml" \
  dataset:="${bagnames[a]}" \
  bag:="$bag_path/${bagnames[a]}.bag" \
  bag_start:="${bagstarttimes[a]}" \
  dobag:="true" \
  dosave:="true" \
  dotime:="true" \
  node_name:="$node_name" \
  path_est:="$filename_est"\
  path_time:="$filename_time" &> /dev/null &
  sleep 1

# check this node runs
# check that the node is running
while [ ! -z "$(rostopic info /${filename}_${bagnames[a]}_${d}/loop_depth_colored 2>/dev/null)" ]; do
  sleep 0.1
done

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
# echo "BASH: ${bagnames[a]} - start runing $e took $elapsed seconds";
# Program Bar
# Update progress bar
current_time="$(date -u +%s)"
elapsed="$(($current_time-$start_time))"
percentage=$((count * 100 / total_runs))
remaining_time=$((elapsed * total_runs / count - elapsed))
hours=$(printf "%02d" $((remaining_time / 3600)))
minutes=$(printf "%02d" $((remaining_time % 3600 / 60)))
seconds=$(printf "%02d" $((remaining_time % 60)))
echo -ne "Progress: [$percentage%] Remaining Time: ${hours}:${minutes}:${seconds} \r"

done
done

echo "Waiting for the run to finish"

# waiting for all the ros nodce to stop running
while [ ! -z "$(rostopic info /tf 2> /dev/null)" ]; do
    sleep 0.1
done

# kill the roscore after finishing running
pkill roscore

DISPLAY=:0.0 && rosrun ov_eval error_comparison posyaw $HOME/datasets/euroc_mav/gt/vicon_room ${save_path_est} >> $log_path
DISPLAY=:0.0 && rosrun ov_eval error_comparison posyaw $HOME/datasets/euroc_mav/gt/machine_hall ${save_path_est} >> $log_path