#!/bin/bash

source ./install/setup.bash

echo "example: ./run.sh <project_loc> <bag_path> <mcap_path>"
echo "project_loc: support la, others fill in none"
echo "mcap_path  : can be blank, default /home/{USER}/mcap"
echo "if not loadbag: ./run.sh la"
echo "if     loadbag: ./run.sh la /home/uto/ros2.bag /home/uto/mcap"

project_loc="none"
if [[ "la" = $1 ]]; then
  project_loc="LA"
else
  echo "param 1 is $1, please confirm it!"
fi

if [ -z $2 ]; then
  ros2 run uto_per_fs uto_per_fs --ros-args -p project_name:=$project_loc -r __ns:=/mot
  exit
fi

if [ -z $3 ]; then
  mcap_path=$HOME/mcap
else
  mcap_path=$3
fi

ros2 run uto_per_fs uto_per_fs --ros-args -p project_name:=$project_loc -p recording_path:=$2 -p mcap_path:=$mcap_path -r __ns:=/mot
# ros2 launch uto_per_vot uto_per_vot.launch.xml
# ros2 run --prefix 'gdb -ex run --args' uto_per_vot uto_per_vot --ros-args -p vehicle_num:=$1

