if (USE_CC_IECU2.0)
  message(STATUS "================  -] building IECU2.0 [-  ================")
  find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
  include_directories(${CERES_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${CERES_LIBRARIES})

else ()
  message(STATUS "================  -] building x86-64 [-  ================")
  find_package(Ceres REQUIRED COMPONENTS )
  include_directories(${CERES_INCLUDE_DIRS})
  list(APPEND ALL_LIBRARIES ${CERES_LIBRARIES})

endif()

#set(CERES_INCLUDE_DIRS /home/chenyi/Library/ceres/include)
#include_directories(${CERES_INCLUDE_DIRS})
#file(GLOB CERES_LIBS /home/chenyi/Library/ceres/lib/*.a)
#list(APPEND ALL_LIBRARIES ${CERES_LIBS})