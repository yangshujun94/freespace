#!/bin/bash
export TOOLPATH=/root/cc_ws/drive
export SYSROOT=/root/cc_ws/drive/toolchains/aarch64--glibc--stable-2020.08-1/aarch64-buildroot-linux-gnu/sysroot
export TARGET_TRIPLE=aarch64-buildroot-linux-gnu
export CC=$TOOLPATH/toolchains/aarch64--glibc--stable-2020.08-1/usr/bin/$TARGET_TRIPLE-gcc
export CXX=$TOOLPATH/toolchains/aarch64--glibc--stable-2020.08-1/usr/bin/$TARGET_TRIPLE-g++
export CROSS_COMPILE=$TOOLPATH/toolchains/aarch64--glibc--stable-2020.08-1/usr/bin/$TARGET_TRIPLE-
export CPLUS_INCLUDE_PATH=$SYSROOT/usr/include/eigen3
export UTO_OPTIMIZE_MARCH=armv8.2-a

source /opt/ros/galactic/local_setup.bash
source /opt/uto/pilot/local_setup.bash

CMAKE_MODULE_PATH="/opt/uto/pilot/tools/cmake/modules/;"
CMAKE_MODULE_PATH+="/opt/uto/pilot/tools/cmake/;"
export CMAKE_MODULE_PATH=$CMAKE_MODULE_PATH

./clean_project.sh

colcon build --parallel-worker 16 \
  --cmake-force-configure \
  --event-handlers=console_cohesion+ \
  --cmake-args \
  -DCMAKE_VERBOSE_MAKEFILE=ON \
  -DBoost_DIR=$SYSROOT/usr/lib/aarch64-linux-gnu \
  -DGeographicLib_DIR=$SYSROOT/usr/lib/aarch64-linux-gnu \
  -DOpenCV_DIR=$SYSROOT/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
  -DPCL_DIR=$SYSROOT/usr/lib/aarch64-linux-gnu/cmake/pcl \
  -DCMAKE_MODULE_PATH=$CMAKE_MODULE_PATH \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
  -DCMAKE_TOOLCHAIN_FILE=$TOOLPATH/Toolchain-V5L.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DPROJECT_BUILD_FLAG=HDT

#--merge-install
#--install-base=/opt/uto/pilot
