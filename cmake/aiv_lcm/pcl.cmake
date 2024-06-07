if (USE_CC_IECU2.0)
  message(STATUS "================  -] building IECU2.0 [-  ================")
  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${PCL_LIBRARIES})
else ()
  message(STATUS "================  -] building x86-64 [-  ================")
  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${PCL_LIBRARIES})
endif()