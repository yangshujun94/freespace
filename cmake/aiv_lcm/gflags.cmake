if (USE_CC_IECU2.0)
    message(STATUS "================  -] building IECU2.0 [-  ================")
    set(GLOG_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/3rdparty/gflags-2.2.0/iecu2.0/include)
    include_directories(${GLOG_INCLUDE_DIRS})
else ()
    message(STATUS "================  -] building x86-64 [-  ================")
endif ()

