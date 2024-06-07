#kotei
set(KOTEI_INCLUDE ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/kotei/include)
set(KOTEI_LIBRARIES
        ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/kotei/lib/libHorizonProviderAPI.so
        ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/kotei/lib/libSituationProviderAPI.so
        ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/kotei/lib/libPoiSearchAPI.so
        ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/kotei/lib/libPoiParseAPI.so
        ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/kotei/lib/libMapMatchAPI.so
        ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/kotei/lib/libRoutePlanAPI.so
        ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/kotei/lib/libDataManager.so
        ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/kotei/lib/libBaseLib.so)
include_directories(${KOTEI_INCLUDE})
list(APPEND ALL_LIBRARIES ${KOTEI_LIBRARIES})