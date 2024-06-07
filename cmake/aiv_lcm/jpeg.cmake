if (USE_CC_IECU2.0)
  message(STATUS "================  -] building IECU2.0 [-  ================")
  set(JPEG_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/3rdparty/libjpeg-turbo-2.0.0/iecu2.0/include)
  include_directories(${JPEG_INCLUDE_DIRS})
  file(GLOB JPEG_LIBS ${PROJECT_SOURCE_DIR}/3rdparty/libjpeg-turbo-2.0.0/iecu2.0/lib/*.so)
  list(APPEND ALL_LIBRARIES ${JPEG_LIBS})
  

else ()
  message(STATUS "================  -] building x86-64 [-  ================")
  set(JPEG_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/3rdparty/libjpeg-turbo-2.0.x/include)
  include_directories(${JPEG_INCLUDE_DIRS})
  file(GLOB JPEG_LIBS ${PROJECT_SOURCE_DIR}/3rdparty/libjpeg-turbo-2.0.x/lib/*.so)
  list(APPEND ALL_LIBRARIES ${JPEG_LIBS})
  

endif()
