message(STATUS "================  -] building x86-64 [-  ================")
set(UTO_INCLUDE ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/uto/include)
set(UTO_LIBRARIES 
  ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/uto/lib/libuto__python.so
  ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/uto/lib/libuto__rosidl_generator_c.so
  ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/uto/lib/libuto__rosidl_typesupport_c.so
  ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/uto/lib/libuto__rosidl_typesupport_cpp.so
  ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/uto/lib/libuto__rosidl_typesupport_fastrtps_c.so
  ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/uto/lib/libuto__rosidl_typesupport_fastrtps_cpp.so
  ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/uto/lib/libuto__rosidl_typesupport_introspection_c.so
  ${CMAKE_CURRENT_LIST_DIR}/../3rdparty/uto/lib/libuto__rosidl_typesupport_introspection_cpp.so
)

include_directories(${UTO_INCLUDE})
list(APPEND ALL_LIBRARIES ${UTO_LIBRARIES})
