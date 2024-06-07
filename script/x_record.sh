#!/bin/bash
#source /opt/uto/pilot/setup.bash
#ros2 bag record /perception/prediction_obstacles
#/freespace_fusion/freespace_grid_enu
#  /multi_objects_tracking/Lidar_obj \
#  /multi_objects_tracking/radar \
#  /multi_objects_tracking/image_fc120 \
#  /multi_objects_tracking/lidar_fs \
#  /multi_objects_tracking/track_obj

#Record all channels except point cloud
ros2  bag record -a  -x '(.*)/hal/sensor/lidar(.*)|(.*)/hal/sensor_raw(.*)'
