if (USE_CC_IECU2.0)
    message(STATUS "================  -] tf2_msgs building IECU2.0 [-  ================")
    find_package(tf2_msgs REQUIRED)
    include_directories(${tf2_msgs_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${tf2_msgs_LIBRARIES})

else ()
    message(STATUS "================  -] tf2_msgs building x86-64 [-  ================")
    find_package(tf2_msgs REQUIRED)
    include_directories(${tf2_msgs_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${tf2_msgs_LIBRARIES})

endif ()