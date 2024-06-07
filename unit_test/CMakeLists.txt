cmake_minimum_required(VERSION 3.13.5 FATAL_ERROR)
project(unittest_fs)

set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS "-std=c++17 --coverage -Werror=return-type -Wno-enum-compare -Wno-narrowing")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb")

set(EM_ROOT ${PROJECT_SOURCE_DIR}/..)
add_compile_definitions(EM_ROOT=${EM_ROOT})

# find packages
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(uto REQUIRED)
find_package(uto_common_base REQUIRED)
find_package(uto_common_proto REQUIRED)

# include paths
include_directories(
        ${EM_ROOT}/src/
        ${EIGEN3_INCLUDE_DIR}
        ${uto_INCLUDE_DIRS}
        ${uto_common_base_INCLUDE_DIRS}
        ${uto_common_proto_INCLUDE_DIRS})

# add source files
#file(GLOB_RECURSE SRC_CPPS ${EM_ROOT}/src/*.cpp)
#file(GLOB_RECURSE TEST_CPPS ${EM_ROOT}/unit_test/tests/*.cpp)
#list(REMOVE_ITEM SRC_CPPS
#        ${EM_ROOT}/src/peripheral/main.cpp
#        ${EM_ROOT}/src/peripheral/fs_node.cpp)

# add executable
add_executable(${PROJECT_NAME} ${EM_ROOT}/unit_test/tests/ring_buffer_test.cpp)

# dependency
ament_target_dependencies(${PROJECT_NAME}
        rclcpp)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        gtest
        gtest_main
        gcov
        ${uto_LIBRARIES}
        ${uto_common_proto_LIBRARIES})