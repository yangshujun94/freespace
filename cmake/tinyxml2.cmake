message(STATUS "================  -] building x86-64 [-  ================")
set(TINYXML2_INCLUDE ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/tinyxml2/include)
set(TINYXML2_LIBRARIES ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/tinyxml2/lib/libtinyxml2.so)
include_directories(${TINYXML2_INCLUDE})
list(APPEND ALL_LIBRARIES ${TINYXML2_LIBRARIES})

# find_package(tinyxml2_vendor REQUIRED)
# find_package(TinyXML2 REQUIRED)
# include_directories(${TinyXML2_INCLUDE_DIRS})
# list(APPEND ALL_LIBRARIES ${TINYXML2_LIBRARIES})

