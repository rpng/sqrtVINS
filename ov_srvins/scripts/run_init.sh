#!/bin/bash

# Run this script from the ov_init directory

if ! [ $1 ] || ! [ $2 ] || ! [ $3 ] || ! [ $4 ]; then 
  echo 'Usage: ./scripts/evaluate_dynamic_init.sh <dataset={euroc_mav,tum_vi,rpng_ar_table}> <algorithm> <outdir> <do_eval={0,1}> [[extra arg1 for ov_srvins] extra arg2 for ov_srvins]...]'
  exit 1
fi

# Run from the same start time multiple times to average results
runs_per_start_time=1

# Time in seconds for each init window to try and run. The sequence will be divided into
# seq_length/window_length windows, and init will be run on them all
window_length=10
max_cpu_usage=95
dataset=$1
algo=$2
outdir=$3
do_eval=$4
extra_args="${@:5}"


bag_path="$HOME/datasets/$dataset"


if [ "$extra_args" != "none" ]; then
  extra_args=$(echo "$extra_args" | sed 's/,/ /g')
else
  extra_args=""
fi


echo Dynamic init eval settings
echo dataset: $dataset
echo algorithm: $algo
echo outdir: $outdir
echo do_eval: $do_eval
echo extra args for ov_srvins: $extra_args

if [ $do_eval -eq 1 ]; then
  echo Running ov_eval after algorithm
elif [ $do_eval -eq 0 ]; then
  echo Skipping ov_eval based on do_eval arg
else
  echo ERROR: Invalid value for do_eval. Options are 0 or 1
  exit 1
fi


if [ $dataset = euroc_mav ]; then
  sequences=(
    V1_01_easy
    V1_02_medium
    V1_03_difficult
    V2_01_easy
    V2_02_medium
    V2_03_difficult
    MH_01_easy
    MH_02_easy
    MH_03_medium
    MH_04_difficult
    MH_05_difficult
  )
  start_t=0
elif [ $dataset = tum_vi ]; then
  sequences=(
    dataset-room1_512_16
    dataset-room2_512_16
    dataset-room3_512_16
    dataset-room4_512_16
    dataset-room5_512_16
    dataset-room6_512_16
  )
  start_t=0
elif [ $dataset = rpng_ar_table ]; then
  sequences=(
    table_01
    table_02
    table_03
    table_04
    table_05
    table_06
    table_07
    table_08
  )
  start_t=0
else
  echo ERROR: Invalid dataset. Options are euroc_mav, tum_vi, or rpng_ar_table
  exit 1
fi

# script_dir=$( dirname "${BASH_SOURCE[0]}" )
script_dir="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${script_dir}/../../../../devel/setup.bash
mkdir -p $outdir || { echo "ERROR: Could not mkdir $outdir. Quitting" ; exit 1; }

# Make two subdirectories for init window and VIO tracking eval
for subdir in init vio; do
  mkdir -p $outdir/$subdir/truth
  if [ -d $outdir/$subdir/algorithms/$algo ]; then
    # rm -rf $outdir/$subdir/algorithms/$algo
    continue
  fi
  mkdir -p $outdir/$subdir/algorithms/$algo
done
mkdir -p $outdir/init/metadata/$algo

gt_dataroot=${script_dir}/../../ov_data/$dataset

# Get the total number of runs
num_runs=0
num_runs_successful=0
runs_files=()
num_sequences=${#sequences[@]}
total_runs=0
for i in ${!sequences[@]}; do
  gt_file=$gt_dataroot/${sequences[i]}.txt
  max_start_t=`python3 -c "f=open(\"$gt_file\"); ts=[float(line.split()[0]) for line in f.readlines() if line[0]!='#']; print($window_length*int((ts[-1]-ts[0])/$window_length-1)); f.close()"`
  num_bags=$((max_start_t/window_length+1))
  total_runs=$((total_runs + num_bags * runs_per_start_time))
done
echo "==========================Start Sytem Realworld Test=========================="
echo "Running $total_runs total runs"
count=0
start_time="$(date -u +%s)"

# Start ROS
roscore &> /dev/null & disown 
sleep 2

for i in ${!sequences[@]}; do

  # Copy over GT
  gt_file=$gt_dataroot/${sequences[i]}.txt
  for subdir in init vio; do
    cp $gt_file $outdir/$subdir/truth || { echo "Did not find $gt_dataroot/${sequences[i]}.txt" ; exit 1; }
  done

  # Find the max bag start time for this sequence with this python one-liner <<== this is cool!
  max_start_t=`python3 -c "f=open(\"$gt_file\"); ts=[float(line.split()[0]) for line in f.readlines() if line[0]!='#']; print($window_length*int((ts[-1]-ts[0])/$window_length-1)); f.close()"`

  for subdir in init vio; do
    mkdir -p $outdir/$subdir/algorithms/$algo/${sequences[i]}
    mkdir -p $outdir/$subdir/metadata/$algo/${sequences[i]}
    mkdir -p $outdir/$subdir/timing/$algo/${sequences[i]}
  done

  for bag_start_t in `seq $start_t $window_length $max_start_t`; do
    for (( k=0; k<$runs_per_start_time; k++ )); do
    sequence_name=${sequences[i]//-/_}

    node_name="init_window_${sequence_name}_start_time_${bag_start_t}_run_${k}"
    init_win_pose_path="$outdir/init/algorithms/$algo/${sequences[i]}/run_${k}_start_time_${bag_start_t}.txt"
    init_win_metadata_path="$outdir/init/metadata/$algo/${sequences[i]}/metadata_run_${k}_start_time_${bag_start_t}.txt"
    vio_pose_path="$outdir/vio/algorithms/$algo/${sequences[i]}/run_${k}_start_time_${bag_start_t}.txt"
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

    # small offset to fix some gt starting later than the bag
    if [ "${bag_start_t}" = "0" ] && [ "${sequences[i]}" = "MH_01_easy" ]; then
      bag_start_t="2"
    fi

    if [ "${bag_start_t}" = "0" ] && [ "${sequences[i]}" = "MH_02_easy" ]; then
      bag_start_t="3"
    fi

    roslaunch ov_srvins serial.launch \
      dataset:="${sequences[i]}" \
      bag_start:="${bag_start_t}" \
      bag_durr:="$window_length" \
      config:="$dataset" \
      bag:="$bag_path/${sequences[i]}.bag" \
      dobag:="true" \
      dosave:="true" \
      config_path:=" ${script_dir}/../../config/$dataset/estimator_config_init.yaml" \
      path_est:="$vio_pose_path" \
      record_init_pose:="true" \
      record_init_timing:="true" \
      init_poses_log_file_path:="$init_win_pose_path" \
      init_metadata_log_file_path:="$init_win_metadata_path" \
      $extra_args \
      node_name:="$node_name" &> /dev/null &
      # sleep 1

    # Wait for the node to start
    # while [ -z "$(rostopic info /${node_name}/loop_depth_colored 2>/dev/null)" ]; do
    #   sleep 0.1
    # done

    current_time="$(date -u +%s)"
    elapsed="$(($current_time-$start_time))"

    # Program Bar
    # Update progress bar
    percentage=$((count * 100 / total_runs))
    remaining_time=$((elapsed * total_runs / count - elapsed))
    hours=$(printf "%02d" $((remaining_time / 3600)))
    minutes=$(printf "%02d" $((remaining_time % 3600 / 60)))
    seconds=$(printf "%02d" $((remaining_time % 60)))
    echo -ne "Progress: [$percentage%] Remaining Time: ${hours}:${minutes}:${seconds} \r"

    done
  done
done

echo "Waiting for the run to finish...................."
wait_start_time="$(date -u +%s)"
idle_cpu_usage=5
# waiting for all the ros nodes to stop running
while [ ! -z "$(rostopic info /tf 2> /dev/null)" ]; do
    wait_end_time="$(date -u +%s)"
    wait_elapsed="$(($wait_end_time-$wait_start_time))"
    if [ $wait_elapsed -gt 120 ]; then
        # Get current CPU usage
        cpu_usage=$(top -b -n1 | grep "Cpu(s)" | awk '{print $2 + $4}')
        # Check if the CPU usage is greater than the maximum allowed
        if awk "BEGIN {exit !($cpu_usage < $idle_cpu_usage)}"; then
          break
        fi
    fi
    sleep 0.1
done
pkill roscore
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
hours=$(printf "%02d" $((elapsed / 3600)))
minutes=$(printf "%02d" $((elapsed % 3600 / 60)))
seconds=$(printf "%02d" $((elapsed % 60)))
echo "Total run time: ${hours}:${minutes}:${seconds}"
echo "==========================Finshed Test=============================="


if [ $do_eval -eq 1 ]; then
  echo "==========================Evaluating...=============================="
  # NOTE we use posyawsingle to be fair since the trajectory is so short
  # NOTE made results worse too to use posyaw
  #rosrun ov_eval error_comparison posyaw $outdir/truth/ $outdir/algorithms/ 1 
  run_name=$(basename "$outdir")
  ${script_dir}/eval.sh $run_name "init" &> "$outdir/init/results_traj.txt"
  ${script_dir}/eval.sh $run_name "vio" &> "$outdir/vio/results_traj.txt"
  cat "$outdir/init/results_traj.txt"
  cat "$outdir/vio/results_traj.txt"
  echo "Done!"
fi
