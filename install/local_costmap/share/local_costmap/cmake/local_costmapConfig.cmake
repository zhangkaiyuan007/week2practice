# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_local_costmap_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED local_costmap_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(local_costmap_FOUND FALSE)
  elseif(NOT local_costmap_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(local_costmap_FOUND FALSE)
  endif()
  return()
endif()
set(_local_costmap_CONFIG_INCLUDED TRUE)

# output package information
if(NOT local_costmap_FIND_QUIETLY)
  message(STATUS "Found local_costmap: 0.0.0 (${local_costmap_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'local_costmap' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${local_costmap_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(local_costmap_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${local_costmap_DIR}/${_extra}")
endforeach()
