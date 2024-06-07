if (USE_CC_IECU2.0)
    message(STATUS "================  -] common building IECU2.0 [-  ================")
    set(Uto_INCLUDE_DIRS /opt/uto/pilot/uto/include)
    set(uto_common_base_INCLUDE_DIRS /opt/uto/pilot/uto_common_base/include)
    set(uto_common_proto_INCLUDE_DIRS /opt/uto/pilot/uto_common_proto/include)

    include_directories(${uto_INCLUDE_DIRS}
            ${uto_common_base_INCLUDE_DIRS}
            ${uto_common_proto_INCLUDE_DIRS})

    file(GLOB uto_LIBRARIES /opt/uto/pilot/uto/lib/*.so)
    file(GLOB uto_common_base_LIBRARIES /opt/uto/pilot/uto_common_base/lib/*.so)
    file(GLOB uto_common_proto_LIBRARIES /opt/uto/pilot/uto_common_proto/lib/*.so)

    list(APPEND ALL_LIBRARIES
            ${uto_LIBRARIES}
            ${uto_common_base_LIBRARIES}
            ${uto_common_proto_LIBRARIES})
elseif (USE_CC_IECU3.1)
    message(STATUS "================  -] common building IECU3.1 [-  ================")
    set(Uto_INCLUDE_DIRS /opt/uto/pilot/uto/include)
    set(uto_common_base_INCLUDE_DIRS /opt/uto/pilot/uto_common_base/include)
    set(uto_common_proto_INCLUDE_DIRS /opt/uto/pilot/uto_common_proto/include)

    include_directories(${uto_INCLUDE_DIRS}
            ${uto_common_base_INCLUDE_DIRS}
            ${uto_common_proto_INCLUDE_DIRS})

    file(GLOB uto_LIBRARIES /opt/uto/pilot/uto/lib/*.so)
    file(GLOB uto_common_base_LIBRARIES /opt/uto/pilot/uto_common_base/lib/*.so)
    file(GLOB uto_common_proto_LIBRARIES /opt/uto/pilot/uto_common_proto/lib/*.so)

    file(GLOB ros2_LIBRARIES /opt/ros/galactic/lib/librosidl_typesupport_fastrtps_cpp.so
            /opt/ros/galactic/lib/librosidl_typesupport_fastrtps_c.so
            /opt/ros/galactic/lib/libfastrtps.so.2.3
            /opt/ros/galactic/lib/libfastcdr.so.1)

    list(APPEND ALL_LIBRARIES
            ${uto_LIBRARIES}
            ${uto_common_base_LIBRARIES}
            ${uto_common_proto_LIBRARIES}
            ${ros2_LIBRARIES})
else ()

endif ()