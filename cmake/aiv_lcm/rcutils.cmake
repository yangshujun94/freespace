if (USE_CC_IECU2.0)
    message(STATUS "================  -] rcutils building IECU2.0 [-  ================")
    find_package(rcutils REQUIRED)
    include_directories(${rcutils_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${rcutils_LIBRARIES})
elseif (USE_CC_IECU3.1)
    message(STATUS "================  -] rcutils building IECU3.1 [-  ================")
    find_package(rcutils REQUIRED)
    include_directories(${rcutils_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${rcutils_LIBRARIES})
else ()
    message(STATUS "================  -] rcutils building x86-64 [-  ================")
    find_package(rcutils REQUIRED)
    include_directories(${rcutils_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${rcutils_LIBRARIES})

endif ()