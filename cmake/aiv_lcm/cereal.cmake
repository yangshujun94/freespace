if (USE_CC_IECU2.0)
  message(STATUS "================  -] building IECU2.0 [-  ================")
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/cereal-1.3.0/iecu2.0/include)

else ()
  message(STATUS "================  -] building x86-64 [-  ================")
  include_directories(${PROJECT_SOURCE_DIR}/3rdparty/cereal-1.3.0/include)

endif()