find_package(TurboJPEG QUIET)
include_directories(${TurboJPEG_INCLUDE_DIRS})
#list(APPEND ALL_LIBRARIES ${TurboJPEG_LIBRARIES})
list(APPEND ALL_LIBRARIES turbojpeg)