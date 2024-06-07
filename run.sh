#!/bin/bash

source ./install/setup.bash

echo "example: ./run.sh <project_loc> <bagpath>"
echo "project_loc: ws/dx/qz/tc/hdt"
echo "no loadbag : bash run.sh qz"
echo "loadbag    : bash run.sh qz /home/uto/ros2.bag"

if [[ "ws" = $1 ]]; then
  project_loc="WS"
elif [[ "dx" = $1 ]]; then
  project_loc="DX"
elif [[ "qz" = $1 ]]; then
  project_loc="QZ"
elif [[ "tc" = $1 ]]; then
  project_loc="TC"
elif [[ "hdt" = $1 ]]; then
  project_loc="HDT"
else
  echo "param 1 is invalid, project_loc: ws/dx/qz/tc/hdt"
  exit 1
fi

if [ -z $2 ]; then
  ros2 run uto_per_fs uto_per_fs --ros-args -p project_name:=$project_loc -r __ns:=/fs
  exit
fi
ros2 run uto_per_fs uto_per_fs --ros-args -p project_name:=$project_loc -p recording_path:=$2 -p path_out:=$3 -r __ns:=/fs
