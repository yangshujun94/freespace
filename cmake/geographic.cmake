message(STATUS "================  -] building x86-64 [-  ================")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Modules")

find_package(GeographicLib QUIET)

include_directories(${GeographicLib_INCLUDE_DIRS})
#list(APPEND ALL_LIBRARIES ${GeographicLib_LIBRARIES})
list(APPEND ALL_LIBRARIES Geographic)

