if (USE_CC_IECU2.0)
  message(STATUS "================  -] cv_bridge building IECU2.0 [-  ================")
  find_package(cv_bridge REQUIRED)
  include_directories(${cv_bridge_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${cv_bridge_LIBRARIES})
elseif (USE_CC_IECU3.1)
  message(STATUS "================  -] cv_bridge building IECU3.1 [-  ================")
  find_package(cv_bridge REQUIRED)
  include_directories(${cv_bridge_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${cv_bridge_LIBRARIES})
else ()
  message(STATUS "================  -] cv_bridge building x86-64 [-  ================")
  find_package(cv_bridge REQUIRED)
  include_directories(${cv_bridge_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${cv_bridge_LIBRARIES})
endif()