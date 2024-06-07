# #tinyxml
# include("${CMAKE_CURRENT_LIST_DIR}/tinyxml2.cmake")

#kotei
include("${CMAKE_CURRENT_LIST_DIR}/kotei.cmake")

#utomap_sdk
set(UTOMAP_SDK_INCLUDE ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/utomap_sdk/include)
set(UTOMAP_SDK_LIBRARIES ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/utomap_sdk/lib/libutomap_sdk.so)
include_directories(${UTOMAP_SDK_INCLUDE})
list(APPEND ALL_LIBRARIES ${UTOMAP_SDK_LIBRARIES})