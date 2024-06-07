if (USE_CC_IECU2.0)
  message(STATUS "================  -] building IECU2.0 [-  ================")
  find_package(catkin REQUIRED COMPONENTS
          roscpp
          pcl_ros
          eigen_conversions
          cv_bridge
          tf)
  
  include_directories(${catkin_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${catkin_LIBRARIES})
else ()
  message(STATUS "================  -] building x86-64 [-  ================")
  find_package(catkin REQUIRED COMPONENTS
          roscpp
          pcl_ros
          eigen_conversions
          cv_bridge
          tf)
  
  include_directories(${catkin_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${catkin_LIBRARIES})
endif()