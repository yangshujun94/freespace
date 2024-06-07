#!/bin/bash

echo "------------ Start CrossCompile ----------"
source /opt/ros/eloquent/local_setup.bash
source /opt/uto/pilot/local_setup.bash
export TOOLPATH=/cross

if [ ! -d "build-arm" ]; then
  mkdir build-arm
else
  rm -rf build-arm/*
fi

cd build-arm/

cmake -DTHIRD_PART_PATH=/home/utopilot/3rdparty \
  -DCMAKE_TOOLCHAIN_FILE=$TOOLPATH/Toolchain-V5L.cmake \
  -DVIBRANTE_PDK:STRING=$TOOLPATH/drive-t186ref-linux \
  -DCMAKE_BUILD_TYPE=Release \
  -DPROJECT_BUILD_FLAG=AIV3_LCM \
  -DCC_BUILD=ON ..
make -j

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
