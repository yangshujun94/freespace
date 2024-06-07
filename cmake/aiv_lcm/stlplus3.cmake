if (USE_CC_IECU2.0)
  message(STATUS "================  -] building IECU2.0 [-  ================")
  set(STLPLUS3_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/3rdparty/stlplus3/iecu2.0/include)
  include_directories(${STLPLUS3_INCLUDE_DIRS})
  file(GLOB STLPLUS3_LIBS ${PROJECT_SOURCE_DIR}/3rdparty/stlplus3/iecu2.0/lib/*.so)
  list(APPEND ALL_LIBRARIES ${STLPLUS3_LIBS})
  

else ()
  message(STATUS "================  -] building x86-64 [-  ================")
  set(STLPLUS3_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/3rdparty/stlplus3/include)
  include_directories(${STLPLUS3_INCLUDE_DIRS})
  file(GLOB STLPLUS3_LIBS ${PROJECT_SOURCE_DIR}/3rdparty/stlplus3/lib/*.so)
  list(APPEND ALL_LIBRARIES ${STLPLUS3_LIBS})
  

endif()