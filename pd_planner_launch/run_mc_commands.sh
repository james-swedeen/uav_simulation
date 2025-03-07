#! /bin/bash

HOME_DIR="/home/james"
WORKSPACE_DIR="/home/james/ros_ws"
TOTAL_NUM_SIMS=100    # The total number of simulations to run
NUM_SIM_IN_PARALLEL=4 # How many simulations to run in parallel

#set -e

cd ${WORKSPACE_DIR}

echo "Building"
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --no-warn-unused-cli -DCMAKE_CXX_STANDARD=17
source .venv/bin/activate
source install/setup.bash

echo "Clearing Old Data"
rm -r "${HOME_DIR}/test_data_headless"
for file in "/tmp/sim_output_*"
do
  rm ${file}
done

echo "Running Simulations"

max_ros_domainid=232

sim_count=0
while (( ${sim_count} < ${TOTAL_NUM_SIMS} ))
do
  sub_sim_count=0
  echo "Starting Simulation Batch"
  while (( ${sub_sim_count} < ${NUM_SIM_IN_PARALLEL} )) && (( ${sim_count} < ${TOTAL_NUM_SIMS} ))
  do
    sim_count=$((sim_count+1))
    sub_sim_count=$((sub_sim_count+1))
    domain_id=$((max_ros_domainid-sim_count))
    bash -c "ROS_DOMAIN_ID=${domain_id} && ros2 launch pd_planner_launch headless_sim_scenario.launch.py" > "/tmp/sim_output_${domain_id}.txt" &
    #bash -c "ROS_DOMAIN_ID=${domain_id} && ros2 launch pd_planner_launch headless_sim_scenario.launch.py" &
    #time bash -c "ROS_DOMAIN_ID=${domain_id} && ros2 launch pd_planner_launch headless_sim_scenario.launch.py" > "/tmp/sim_output_${domain_id}.txt"
    sleep 3
  done
  echo "Simulation Batch Running"
  wait
  echo "Simulation Batch Finished"
done

echo "Plotting"

for data_file in ${HOME_DIR}/test_data_headless/*
do
  if [[ "${data_file}" == "${HOME_DIR}/test_data_headless/data_00000" ]]
  then
    continue
  fi
  data_index=${data_file:(-5)}
  for run_file in ${data_file}/run_*
  do
    mv --no-clobber "${run_file}" "${HOME_DIR}/test_data_headless/data_00000/${run_file:(-9)}_${data_index}"
  done
  rm -r "${data_file}"
done

ros2 run planner_interface mc_exporter_run.py ${HOME_DIR}/test_data_headless/data_00000/

notify-send "Simulation results ready for user input"

ros2 run planner_interface mc_plotter_run.py ${HOME_DIR}/test_data_headless/data_00000/

