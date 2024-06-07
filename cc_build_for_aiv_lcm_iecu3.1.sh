#!/bin/bash
export TARGET_TRIPLE=aarch64-buildroot-linux-gnu
export CC=$TOOLPATH/toolchains/aarch64--glibc--stable-2020.08-1/usr/bin/$TARGET_TRIPLE-gcc
export CXX=$TOOLPATH/toolchains/aarch64--glibc--stable-2020.08-1/usr/bin/$TARGET_TRIPLE-g++
export CROSS_COMPILE=$TOOLPATH/toolchains/aarch64--glibc--stable-2020.08-1/usr/bin/$TARGET_TRIPLE-

source /opt/ros/galactic/local_setup.bash
source /opt/uto/pilot/local_setup.sh

export TOOLPATH=/root/cc_ws/drive
export SYSROOT=/root/cc_ws/drive/toolchains/aarch64--glibc--stable-2020.08-1/aarch64-buildroot-linux-gnu/sysroot
export CPLUS_INCLUDE_PATH=$SYSROOT/usr/include/eigen3
export UTO_OPTIMIZE_MARCH=armv8.2-a

CMAKE_MODULE_PATH="/opt/uto/pilot/tools/cmake/modules/;"
CMAKE_MODULE_PATH+="/opt/uto/pilot/tools/cmake/;"
export CMAKE_MODULE_PATH=$CMAKE_MODULE_PATH

#./clean_project.sh
#
#colcon build --parallel-worker 16 \
#  --cmake-force-configure \
#  --event-handlers=console_cohesion+ \
#  --cmake-args \
#  -DTHIRD_PART_PATH=/home/utopilot/atd-NERV/3rdparty \
#  -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
#  -DCMAKE_TOOLCHAIN_FILE=$TOOLPATH/Toolchain-V5L.cmake \
#  -DCMAKE_BUILD_TYPE=Release \
#  -DPROJECT_BUILD_FLAG=AIV5_LCM \
#  -DPROJECT_PLATFORM=arm ..

#--merge-install
#--install-base=/opt/uto/pilot

if [ ! -d "build-arm" ]; then
  mkdir build-arm
else
  rm -rf build-arm/*
fi

cd build-arm/

cmake -DTHIRD_PART_PATH=/home/utopilot/atd-NERV/3rdparty \
  -DCMAKE_TOOLCHAIN_FILE=$TOOLPATH/Toolchain-V5L.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DPROJECT_BUILD_FLAG=AIV5_LCM \
  -DCC_BUILD=ON ..
make -j

#  -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF \
#  -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \

cd ../bin-arm/
echo ""
pwd
echo ""
echo "-----date--------------$(date)"
ls -lh *
echo ""
echo "-----md5--------------"
md5sum *
cd ..
