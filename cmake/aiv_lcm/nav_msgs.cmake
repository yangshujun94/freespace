if (USE_CC_IECU2.0)
    message(STATUS "================  -] nav_msgs building IECU2.0 [-  ================")
    find_package(nav_msgs REQUIRED)
    include_directories(${nav_msgs_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${nav_msgs_LIBRARIES})
elseif (USE_CC_IECU3.1)
    message(STATUS "================  -] nav_msgs building IECU3.1 [-  ================")
    find_package(nav_msgs REQUIRED)
    include_directories(${nav_msgs_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${nav_msgs_LIBRARIES})
else ()
    message(STATUS "================  -] nav_msgs building x86-64 [-  ================")
    find_package(nav_msgs REQUIRED)
    include_directories(${nav_msgs_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${nav_msgs_LIBRARIES})

endif ()