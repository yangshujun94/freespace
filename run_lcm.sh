#!/bin/bash

source ./install/setup.bash

echo "example: ./run1.5.sh <vehicle_num> <project_loc> <logpath> "
echo "project_loc: ws/dx/qz/tc, default ws"
echo "if not loadlog: ./run1.5.sh 40 ws"
echo "if     loadlog: ./run1.5.sh 40 ws /home/uto/lcm.log"

if [ -z $2 ]; then
  project_loc="WS"
elif [ "ws" == $2 ]; then
  project_loc="WS"
elif [ "dx" == $2 ]; then
  project_loc="DX"
elif [ "qz" == $2 ]; then
  project_loc="QZ"
elif [ "tc" == $2 ]; then
  project_loc="TC"
else
  echo "param 2 is invalid, project_loc: ws/dx/qz/tc, default ws"
  exit 1
fi

if [ -z $3 ]; then
  ros2 run uto_per_fs uto_per_fs --ros-args -p vehicle_num:=$1 -p project_name:=$project_loc -r __ns:=/mot
else
  ros2 run uto_per_fs uto_per_fs --ros-args -p vehicle_num:=$1 -p project_name:=$project_loc -p recording_path:=$3 -r __ns:=/mot
fi