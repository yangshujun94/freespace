# Install script for directory: /home/yang/project/freespace/freespace-fusion

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yang/project/freespace/freespace-fusion/install/uto_per_fs")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs" TYPE EXECUTABLE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/uto_per_fs")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs"
         OLD_RPATH "/opt/uto/pilot/rosbag2_cpp/lib:/opt/uto/pilot/visualization_msgs/lib:/opt/uto/pilot/nav_msgs/lib:/opt/uto/pilot/cv_bridge/lib:/opt/uto/pilot/tf2_ros/lib:/opt/uto/pilot/rcutils/lib:/opt/uto/pilot/rosidl_runtime_c/lib:/opt/uto/pilot/rosidl_typesupport_cpp/lib:/opt/uto/pilot/rosidl_typesupport_introspection_cpp/lib:/opt/uto/pilot/rcpputils/lib:/opt/uto/pilot/rosidl_typesupport_c/lib:/opt/uto/pilot/rosidl_typesupport_introspection_c/lib:/opt/uto/pilot/uto/lib:/opt/uto/pilot/uto_common_base/lib:/opt/uto/pilot/builtin_interfaces/lib:/opt/uto/pilot/rosgraph_msgs/lib:/opt/uto/pilot/rcl_yaml_param_parser/lib:/opt/uto/pilot/statistics_msgs/lib:/opt/uto/pilot/tracetools/lib:/opt/uto/pilot/rclcpp/lib:/opt/uto/pilot/uto_common_proto/lib:/usr/local/lib:/opt/uto/pilot/rosbag2_storage/lib:/opt/uto/pilot/sensor_msgs/lib:/opt/uto/pilot/rclcpp_action/lib:/opt/uto/pilot/rcl_action/lib:/opt/uto/pilot/tf2/lib:/opt/uto/pilot/message_filters/lib:/opt/uto/pilot/rclcpp_components/lib:/opt/uto/pilot/libstatistics_collector/lib:/opt/uto/pilot/rcl/lib:/opt/uto/pilot/rmw_implementation/lib:/opt/uto/pilot/rcl_logging_spdlog/lib:/opt/uto/pilot/rcl_logging_interface/lib:/opt/uto/pilot/libyaml_vendor/lib:/opt/uto/pilot/rmw/lib:/opt/uto/pilot/ament_index_cpp/lib:/opt/uto/pilot/class_loader/lib:/opt/uto/pilot/console_bridge_vendor/lib:/opt/uto/pilot/composition_interfaces/lib:/opt/uto/pilot/rcl_interfaces/lib:/opt/uto/pilot/tf2_msgs/lib:/opt/uto/pilot/geometry_msgs/lib:/opt/uto/pilot/std_msgs/lib:/opt/uto/pilot/action_msgs/lib:/opt/uto/pilot/unique_identifier_msgs/lib:/opt/uto/pilot/orocos_kdl/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs" TYPE DIRECTORY FILES "/home/yang/project/freespace/freespace-fusion/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs" TYPE DIRECTORY FILES "/home/yang/project/freespace/freespace-fusion/param")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs" TYPE EXECUTABLE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/uto_per_fs")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs"
         OLD_RPATH "/opt/uto/pilot/rosbag2_cpp/lib:/opt/uto/pilot/visualization_msgs/lib:/opt/uto/pilot/nav_msgs/lib:/opt/uto/pilot/cv_bridge/lib:/opt/uto/pilot/tf2_ros/lib:/opt/uto/pilot/rcutils/lib:/opt/uto/pilot/rosidl_runtime_c/lib:/opt/uto/pilot/rosidl_typesupport_cpp/lib:/opt/uto/pilot/rosidl_typesupport_introspection_cpp/lib:/opt/uto/pilot/rcpputils/lib:/opt/uto/pilot/rosidl_typesupport_c/lib:/opt/uto/pilot/rosidl_typesupport_introspection_c/lib:/opt/uto/pilot/uto/lib:/opt/uto/pilot/uto_common_base/lib:/opt/uto/pilot/builtin_interfaces/lib:/opt/uto/pilot/rosgraph_msgs/lib:/opt/uto/pilot/rcl_yaml_param_parser/lib:/opt/uto/pilot/statistics_msgs/lib:/opt/uto/pilot/tracetools/lib:/opt/uto/pilot/rclcpp/lib:/opt/uto/pilot/uto_common_proto/lib:/usr/local/lib:/opt/uto/pilot/rosbag2_storage/lib:/opt/uto/pilot/sensor_msgs/lib:/opt/uto/pilot/rclcpp_action/lib:/opt/uto/pilot/rcl_action/lib:/opt/uto/pilot/tf2/lib:/opt/uto/pilot/message_filters/lib:/opt/uto/pilot/rclcpp_components/lib:/opt/uto/pilot/libstatistics_collector/lib:/opt/uto/pilot/rcl/lib:/opt/uto/pilot/rmw_implementation/lib:/opt/uto/pilot/rcl_logging_spdlog/lib:/opt/uto/pilot/rcl_logging_interface/lib:/opt/uto/pilot/libyaml_vendor/lib:/opt/uto/pilot/rmw/lib:/opt/uto/pilot/ament_index_cpp/lib:/opt/uto/pilot/class_loader/lib:/opt/uto/pilot/console_bridge_vendor/lib:/opt/uto/pilot/composition_interfaces/lib:/opt/uto/pilot/rcl_interfaces/lib:/opt/uto/pilot/tf2_msgs/lib:/opt/uto/pilot/geometry_msgs/lib:/opt/uto/pilot/std_msgs/lib:/opt/uto/pilot/action_msgs/lib:/opt/uto/pilot/unique_identifier_msgs/lib:/opt/uto/pilot/orocos_kdl/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uto_per_fs/uto_per_fs")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/uto_per_fs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/uto_per_fs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs/environment" TYPE FILE FILES "/opt/uto/pilot/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs/environment" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs/environment" TYPE FILE FILES "/opt/uto/pilot/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs/environment" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_index/share/ament_index/resource_index/packages/uto_per_fs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs/cmake" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs/cmake" TYPE FILE FILES
    "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_core/uto_per_fsConfig.cmake"
    "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/ament_cmake_core/uto_per_fsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uto_per_fs" TYPE FILE FILES "/home/yang/project/freespace/freespace-fusion/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/yang/project/freespace/freespace-fusion/build/uto_per_fs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
