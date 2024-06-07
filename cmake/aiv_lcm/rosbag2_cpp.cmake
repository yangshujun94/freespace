if (USE_CC_IECU2.0)
  message(STATUS "================  -] building IECU2.0 [-  ================")
  find_package(rosbag2_cpp REQUIRED)
  include_directories(${rosbag2_cpp_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${rosbag2_cpp_LIBRARIES})

else ()
  message(STATUS "================  -] building x86-64 [-  ================")
  find_package(rosbag2_cpp REQUIRED)
  include_directories(${rosbag2_cpp_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${rosbag2_cpp_LIBRARIES})

endif()