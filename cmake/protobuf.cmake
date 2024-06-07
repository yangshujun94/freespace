if(USE_CC_IECU2.0)
    message(STATUS "================  -] building IECU2.0 [-  ================")
    find_package(Protobuf REQUIRED ) 
    include_directories(${Protobuf_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${Protobuf_LIBRARIES})
else ()
    message(STATUS "================  -] building x86-64 [-  ================")
    find_package(Protobuf REQUIRED )
    include_directories(${Protobuf_INCLUDE_DIRS})
    list(APPEND ALL_LIBRARIES ${Protobuf_LIBRARIES})
endif()
