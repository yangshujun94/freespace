if (USE_CC_IECU2.0)
  message(STATUS "================  -] building IECU2.0 [-  ================")
  set(GLOG_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/3rdparty/glog-0.3.5/iecu2.0/include)
  include_directories(${GLOG_INCLUDE_DIRS})
  file(GLOB GLOG_LIBS ${PROJECT_SOURCE_DIR}/3rdparty/glog-0.3.5/iecu2.0/lib/*.so)
  list(APPEND ALL_LIBRARIES ${GLOG_LIBS})
  

else ()
  message(STATUS "================  -] building x86-64 [-  ================")
  set(GLOG_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/3rdparty/glog-0.4.0/include)
  include_directories(${GLOG_INCLUDE_DIRS})
  file(GLOB GLOG_LIBS ${PROJECT_SOURCE_DIR}/3rdparty/glog-0.4.0/lib/*.so)
  list(APPEND ALL_LIBRARIES ${GLOG_LIBS})
  

endif()