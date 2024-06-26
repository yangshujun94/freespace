if (USE_CC_IECU2.0)
    message(STATUS "================  -] visualization_msgs building IECU2.0 [-  ================")
    find_package(visualization_msgs REQUIRED)
    include_directories(${visualization_msgs_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${visualization_msgs_LIBRARIES})
elseif (USE_CC_IECU3.1)
    message(STATUS "================  -] visualization_msgs building IECU3.1 [-  ================")
    find_package(visualization_msgs REQUIRED)
    include_directories(${visualization_msgs_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${visualization_msgs_LIBRARIES})
else ()
    message(STATUS "================  -] visualization_msgs building x86-64 [-  ================")
    find_package(visualization_msgs REQUIRED)
    include_directories(${visualization_msgs_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${visualization_msgs_LIBRARIES})

endif ()