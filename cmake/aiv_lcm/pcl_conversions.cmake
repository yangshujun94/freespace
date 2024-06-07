if (USE_CC_IECU2.0)
  message(STATUS "================  -] building IECU2.0 [-  ================")
  find_package(pcl_conversions REQUIRED)
  include_directories(${pcl_conversions_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${pcl_conversions_LIBRARIES})

else ()
  message(STATUS "================  -] building x86-64 [-  ================")
  find_package(pcl_conversions REQUIRED)
  include_directories(${pcl_conversions_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${pcl_conversions_LIBRARIES})

endif()