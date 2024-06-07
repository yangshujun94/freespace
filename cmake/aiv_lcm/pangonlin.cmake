if (USE_CC_IECU2.0)
  message(STATUS "================  -] building IECU2.0 [-  ================")
  set(PANGOLIN_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/3rdparty/Pangolin-0.6/iecu2.0/include)
  include_directories(${PANGOLIN_INCLUDE_DIRS})
  file(GLOB PANGOLIN_LIBS ${PROJECT_SOURCE_DIR}/3rdparty/Pangolin-0.6/iecu2.0/lib/*.so)
  list(APPEND ALL_LIBRARIES ${PANGOLIN_LIBS})
  

else ()
  message(STATUS "================  -] building x86-64 [-  ================")
  set(PANGOLIN_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/3rdparty/Pangolin-0.6/include)
  include_directories(${PANGOLIN_INCLUDE_DIRS})
  file(GLOB PANGOLIN_LIBS ${PROJECT_SOURCE_DIR}/3rdparty/Pangolin-0.6/lib/*.so)
  list(APPEND ALL_LIBRARIES ${PANGOLIN_LIBS})
  

endif()