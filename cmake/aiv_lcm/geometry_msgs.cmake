if (USE_CC_IECU2.0)
  message(STATUS "================  -] building IECU2.0 [-  ================")
  find_package(geometry_msgs REQUIRED)
  include_directories(${geometry_msgs_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${geometry_msgs_LIBRARIES})

else ()
  message(STATUS "================  -] building x86-64 [-  ================")
  find_package(geometry_msgs REQUIRED)
  include_directories(${geometry_msgs_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${geometry_msgs_LIBRARIES})

endif()