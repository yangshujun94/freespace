# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_uto_per_fs_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED uto_per_fs_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(uto_per_fs_FOUND FALSE)
  elseif(NOT uto_per_fs_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(uto_per_fs_FOUND FALSE)
  endif()
  return()
endif()
set(_uto_per_fs_CONFIG_INCLUDED TRUE)

# output package information
if(NOT uto_per_fs_FIND_QUIETLY)
  message(STATUS "Found uto_per_fs: 0.1.0 (${uto_per_fs_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'uto_per_fs' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${uto_per_fs_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(uto_per_fs_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${uto_per_fs_DIR}/${_extra}")
endforeach()
