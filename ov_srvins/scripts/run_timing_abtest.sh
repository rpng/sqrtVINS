#!/usr/bin/env bash

# Assign the input variable to a meaningful name
folder="${1}_timing"
log_path="$2"
total_runs=6

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_DIR}/../../../../devel/setup.bash

#=============================================================
#=============================================================
#=============================================================

# dataset locations
bagnames=(
  "V1_01_easy"
  "V2_03_difficult"
)

# how far we should start into the dataset
# this can be used to skip the initial sections
bagstarttimes=(
  "0"
  "0"
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


# run our ROS launch file (note we send console output to terminator)
# subscribe=live pub, serial=read from bag (fast)
roslaunch ov_srvins serial.launch \
  seed:="${e}" \
  max_cameras:="1" \
  use_stereo:="false" \
  config:="euroc_mav" \
  config_path:="${SCRIPT_DIR}/../../config/euroc_mav/estimator_config_${mode}.yaml" \
  dataset:="${bagnames[a]}" \
  bag:="$bag_path/${bagnames[a]}.bag" \
  bag_start:="${bagstarttimes[a]}" \
  dobag:="true" \
  dosave:="true" \
  dotime:="true" \
  node_name:="$node_name" \
  path_est:="$filename_est"\
  path_time:="$filename_time" &> /dev/null
  sleep 1

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

python3 ${SCRIPT_DIR}/../python/get_time_folder_all_srf.py ${save_path_time}/${filename} >> $log_path

done

echo "Waiting for the run to finish"

# kill the roscore after finishing running
pkill roscore

