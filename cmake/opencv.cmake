find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND ALL_LIBRARIES ${OpenCV_LIBRARIES})

