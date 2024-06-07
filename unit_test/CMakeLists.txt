cmake_minimum_required(VERSION 3.13.5 FATAL_ERROR)
project(unittest_fs)

set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS "-std=c++14 --coverage -Werror=return-type -Wno-enum-compare -Wno-narrowing")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb")

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# find packages
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(uto REQUIRED)
find_package(uto_common_base REQUIRED)
find_package(uto_common_proto REQUIRED)

# include paths
include_directories(
        "/opt/uto/pilot/geometry_msgs/include"
        "/opt/uto/pilot/visualization_msgs/include"
        "/opt/uto/pilot/sensor_msgs/include"
        "/opt/uto/pilot/tf2_eigen/include"
        "/opt/uto/pilot/tf2/include"
        "/opt/uto/pilot/tf2_ros/include"
        "/opt/uto/pilot/tf2_geometry_msgs/include"
        "/opt/uto/pilot/tf2_msgs/include"
        "/opt/uto/pilot/orocos_kdl/include"
        ${PROJECT_SOURCE_DIR}/../src/project/hdt
        ${PROJECT_SOURCE_DIR}/../src/
        ${PROJECT_SOURCE_DIR}/../msgs/
        ${PROJECT_SOURCE_DIR}/../3rd_library/lcm/include
        ${PROJECT_SOURCE_DIR}/../3rd_library/lcm/include/lcm
        ${EIGEN3_INCLUDE_DIR}
        ${uto_INCLUDE_DIRS}
        ${uto_common_base_INCLUDE_DIRS}
        ${uto_common_proto_INCLUDE_DIRS})

# --------------------------------
# install geographic lib by 'sudo apt install libgeographic-dev'
include("${uto_common_base_DIR}/../../../cmake/FindGeographicLib.cmake")

# add source files
file(GLOB_RECURSE SRC_CPPS ${PROJECT_SOURCE_DIR}/../src/*.cpp)
file(GLOB_RECURSE TEST_CPPS ${PROJECT_SOURCE_DIR}/../unit_test/tests/*.cpp)
list(REMOVE_ITEM SRC_CPPS
        ${PROJECT_SOURCE_DIR}/../src/peripheral/main.cpp
        ${PROJECT_SOURCE_DIR}/../src/peripheral/fs_node.cpp)

# add executable
add_executable(${PROJECT_NAME} ${SRC_CPPS} ${TEST_CPPS})

# dependency
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rosbag2_cpp)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        gtest
        gtest_main
        gcov
        ${GeographicLib_LIBRARIES}
        ${uto_LIBRARIES}
        ${uto_common_base_LIBRARIES}
        ${uto_common_proto_LIBRARIES})
