cmake_minimum_required(VERSION 3.13.5 FATAL_ERROR)
project(uto_per_fs)

#------------------------------------------------------------------------------------------
# settings
#set(PROJECT_BUILD_FLAG "HDT") #HDT BT DT RST BUS AIV ALT ICR ADT EAT MIFAEV
#set(CMAKE_BUILD_TYPE "Release") #Release Debug
#set(VIS_ENABLE OFF) #OFF ON
#set(LOAD_LOG_ENABLE OFF) #OFF ON

#------------------------------------------------------------------------------------------
# compiler settings
IF (CMAKE_BUILD_TYPE MATCHES "Debug")
    set(ENV{UTO_OPTIMIZE_DEBUG} "ON")
ELSE ()
    set(CMAKE_BUILD_TYPE "Release")
ENDIF ()

IF (${VIS_ENABLE})
    add_compile_definitions(VIS_ENABLE)
ENDIF ()
IF (${LOAD_LOG_ENABLE})
    add_compile_definitions(LOAD_LOG_ENABLE)
ENDIF ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "-std=c++17 -Werror=return-type -Wno-enum-compare -Wno-narrowing") #-w close all warnings
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG -O2")

if (UTO_SIMULATION)
    set(CMAKE_CXX_FLAGS "-std=c++17 -rdynamic -fPIC")
endif ()

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

#------------------------------------------------------------------------------------------
# git info
# the commit's SHA1, and whether the building workspace was dirty or not
find_package(Git)
execute_process(COMMAND
        "${GIT_EXECUTABLE}" describe --always --abbrev=10 --dirty --tags
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_SHA1
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

# the date of the commit
execute_process(COMMAND
        "${GIT_EXECUTABLE}" log -1 --format=%ad --date=iso
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_DATE
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

# the subject of the commit
execute_process(COMMAND
        "${GIT_EXECUTABLE}" log -1 --format=%s
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_SUBJECT
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

execute_process(COMMAND
        "${GIT_EXECUTABLE}" rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_BRANCH_NAME
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

set(GIT_COMMIT_SHA1 "${GIT_COMMIT_SHA1}  ${GIT_BRANCH_NAME}")

message("==== update git commit SHA    : ${GIT_COMMIT_SHA1}")
message("==== update git commit date   : ${GIT_COMMIT_DATE}")
message("==== update git commit message: ${GIT_COMMIT_SUBJECT}")

#------------------------------------------------------------------------------------------
# cicd settings
set(CICD_VEHICLE_TYPE $ENV{UTO_TARGET_VEHICLE_TYPE})
set(CICD_TARGET_BOARD $ENV{UTO_TARGET_BOARD})
set(CICD_MIDDLEWARE $ENV{UTO_TARGET_MIDDLEWARE})

message("**** get cicd vehicle_type: $ENV{UTO_TARGET_VEHICLE_TYPE}")
message("**** get cicd target_board: $ENV{UTO_TARGET_BOARD}")
message("**** get cicd middleware  : $ENV{UTO_TARGET_MIDDLEWARE}")

IF ("$ENV{UTO_TARGET_VEHICLE_TYPE}" STREQUAL "hdt")       #hdt
    set(PROJECT_BUILD_FLAG "HDT")
ELSEIF ("$ENV{UTO_TARGET_VEHICLE_TYPE}" STREQUAL "aiv")
    IF ("$ENV{UTO_TARGET_BOARD}" STREQUAL "xavier")
        set(PROJECT_BUILD_FLAG "AIV3")
    ELSEIF ("$ENV{UTO_TARGET_BOARD}" STREQUAL "orin")
        set(PROJECT_BUILD_FLAG "AIV5")
    ENDIF ()
ENDIF ()

#------------------------------------------------------------------------------------------
# generate fs_version.cpp
set(AUTO_GEN_CMAKE_INFO_FILE ${CMAKE_CURRENT_BINARY_DIR}/fs_version.cpp)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/fs_version.cpp.in" ${AUTO_GEN_CMAKE_INFO_FILE} @ONLY)
# add auto-generated git commit info file
# list(APPEND SOURCE_FILES ${AUTO_GEN_CMAKE_INFO_FILE})

#------------------------------------------------------------------------------------------
# project settings
IF ("${PROJECT_BUILD_FLAG}" STREQUAL "HDT")
    include_directories(${PROJECT_SOURCE_DIR}/src/project/hdt)
ELSEIF ("${PROJECT_BUILD_FLAG}" STREQUAL "AIV5")
    include_directories(${PROJECT_SOURCE_DIR}/src/project/aiv5)
ELSEIF ("${PROJECT_BUILD_FLAG}" STREQUAL "AIV3")
    include_directories(${PROJECT_SOURCE_DIR}/src/project/aiv3)
ELSE ()
    include_directories(${PROJECT_SOURCE_DIR}/src/project/hdt)
ENDIF ()

#------------------------------------------------------------------------------------------
# build messages
message("++++ this build project : ${PROJECT_BUILD_FLAG}")
message("++++ this build type    : ${CMAKE_BUILD_TYPE}")
message("++++ this build vis     : ${VIS_ENABLE}")
message("++++ this build load_log: ${LOAD_LOG_ENABLE}")

#------------------------------------------------------------------------------------------
# find packages
find_package(uto QUIET)
find_package(uto_common_base QUIET)
find_package(uto_common_proto QUIET)
find_package(uto_map_sdk QUIET)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
        ${uto_INCLUDE_DIRS}
        ${uto_common_base_INCLUDE_DIRS}
        ${uto_common_proto_INCLUDE_DIRS}
        ${uto_map_sdk_INCLUDE_DIRS})

list(APPEND ALL_LIBRARIES
        ${uto_LIBRARIES}
        ${uto_common_base_LIBRARIES}
        ${uto_common_proto_LIBRARIES}
        ${uto_map_sdk_LIBRARIES})

# common compiler settings
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${uto_common_base_INCLUDE_DIRS})
include(UtoCommonOptimizeOption)

# --------------------------------
# install geographic lib by 'sudo apt install libgeographic-dev'
include("${uto_common_base_DIR}/../../../cmake/FindGeographicLib.cmake")

# --------------------------------
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

# include paths
include_directories(${PROJECT_SOURCE_DIR}/src/)

# add source files
file(GLOB_RECURSE SOURCE_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
list(APPEND SOURCE_FILES ${PROTO_SOURCE_FILES} ${AUTO_GEN_CMAKE_INFO_FILE})

# add executable
if (UTO_SIMULATION)
    ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
else ()
    ament_auto_add_executable(${PROJECT_NAME} ${SOURCE_FILES})
endif ()

# dependency
ament_target_dependencies(${PROJECT_NAME})

list(APPEND ALL_LIBRARIES ${GeographicLib_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${ALL_LIBRARIES})

# install
install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY param
        DESTINATION share/${PROJECT_NAME})

ament_auto_package()
