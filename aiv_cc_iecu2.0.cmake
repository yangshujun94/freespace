 message(STATUS "================  -] building IECU2.0 in AIV [-  ================")

 #settings
 add_compile_definitions(IECU_2_0)
 set(USE_CC_IECU2.0 ON)

 # includes
 find_package(ament_cmake REQUIRED)

 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/rclcpp.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/rcutils.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/sensor_msgs.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/nav_msgs.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/cv_bridge.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/tf2_msgs.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/tf2_ros.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/visualization_msgs.cmake)
 set(Dependencies ament_cmake rclcpp rcutils sensor_msgs nav_msgs cv_bridge tf2_msgs tf2_ros visualization_msgs)

 # # 3rdparty
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/eigen.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/pcl.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/opencv.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/lcm.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/protobuf.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/geographiclib.cmake)
 include(${PROJECT_SOURCE_DIR}/cmake/aiv_lcm/common.cmake)

 # include paths
 include_directories(
         ${PROJECT_SOURCE_DIR}/src/
         ${PROJECT_SOURCE_DIR}/msgs/
         ${PROJECT_SOURCE_DIR}/msgs/protobuf/3.3.0/)

 # add source files
 file(GLOB_RECURSE SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
 list(APPEND SOURCE_FILES ${PROTO_SOURCE_FILES} ${AUTO_GEN_CMAKE_INFO_FILE})

 # set(CMAKE_DISABLE_FIND_PACKAGE_HarfBuzz TRUE)
 list(APPEND ALL_LIBRARIES pthread)
# message("append all libraries: ${ALL_LIBRARIES}")
 set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin-arm)
 add_executable(app_fs ${SOURCE_FILES})
 target_link_libraries(app_fs ${ALL_LIBRARIES})
 ament_target_dependencies(app_fs ${Dependencies})
 ament_package()
