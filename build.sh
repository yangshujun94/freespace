#!/bin/bash

if [ -d "install" ]; then
  source install/setup.bash
  source install/setup.sh
fi

help() {
  echo "please check param 1(required),   project_name: hdt|dt|bt|aiv3lcm|aiv3ros|aiv5lcm|aiv5ros..."
  echo "please check param 2(not required), build_type: debug/d | release/r , default: release      "
  echo "please check param 3(not required), vis_enable: 0|1                 , default: 1            "
  echo "please check param 4(not required), load_log  : 0|1                 , default: 0            "
}

if [ -z $1 ] || [[ $1 = "--help" ]] || [[ $1 = "-h" ]]; then
  help
  exit 1
elif [ "hdt" == $1 ]; then
  project_name="HDT"
elif [ "dt" == $1 ]; then
  project_name="DT"
elif [ "bt" == $1 ]; then
  project_name="BT"
elif [ "aiv3lcm" == $1 ]; then
  project_name="AIV3_LCM"
elif [ "aiv3ros" == $1 ]; then
  project_name="AIV3_ROS"
elif [ "aiv5lcm" == $1 ]; then
  project_name="AIV5_LCM"
elif [ "aiv5ros" == $1 ]; then
  project_name="AIV5_ROS"
else
  echo "please check param 1, project_name: hdt|dt|bt|aiv3lcm|aiv3ros|aiv5lcm|aiv5ros"
  echo "exit"
  exit 1
fi

if [ -z $2 ]; then
  echo "param 2 is null, use default setting, build_type: Release"
  build_type="Release"
  vis_enable="ON"
  load_log="OFF"
elif [ "debug" == $2 ] || [ "d" == $2 ]; then
  build_type="Debug"
  vis_enable="ON"
  load_log="OFF"
elif [ "release" == $2 ] || [ "r" == $2 ]; then
  build_type="Release"
  vis_enable="ON"
  load_log="OFF"
else
  echo "please check param 2(not required), build_type: debug/d | release/r , default: release "
  echo "exit"
  exit 1
fi

if [ -z $3 ]; then
  echo "param 3 is null, use default setting, vis_enable: 1"
  vis_enable="ON"
elif [ 0 -eq $3 ]; then
  vis_enable="OFF"
elif [ 1 -eq $3 ]; then
  vis_enable="ON"
else
  echo "please check param 3(not required), vis_enable: 0|1, default: 1"
  echo "exit"
  exit 1
fi

if [ -z $4 ]; then
  echo "param 4 is null, use default setting, load_log_enable: 0"
  load_log="OFF"
elif [ 0 -eq $4 ]; then
  load_log="OFF"
elif [ 1 -eq $4 ]; then
  load_log="ON"
else
  echo "please check param 4(not required), load_log: 0|1, default: 0"
  echo "exit"
  exit 1
fi

echo "build project: ${project_name}"
echo "build_type   : ${build_type}"
echo "vis_enable   : ${vis_enable}"
echo "load_log     : ${load_log}"
colcon build --cmake-args -DPROJECT_BUILD_FLAG=${project_name} -DCMAKE_BUILD_TYPE=${build_type} -DVIS_ENABLE=${vis_enable} -DLOAD_LOG_ENABLE=${load_log}
