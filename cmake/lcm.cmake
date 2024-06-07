message(STATUS "================  -] building x86-64 [-  ================")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Modules")

find_package(LCM QUIET)
include_directories(${LCM_INCLUDE_DIRS})
#list(APPEND ALL_LIBRARIES ${GLOG_LIBRARIES})
list(APPEND ALL_LIBRARIES lcm)

