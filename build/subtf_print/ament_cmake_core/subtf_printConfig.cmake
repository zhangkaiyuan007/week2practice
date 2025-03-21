# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_subtf_print_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED subtf_print_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(subtf_print_FOUND FALSE)
  elseif(NOT subtf_print_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(subtf_print_FOUND FALSE)
  endif()
  return()
endif()
set(_subtf_print_CONFIG_INCLUDED TRUE)

# output package information
if(NOT subtf_print_FIND_QUIETLY)
  message(STATUS "Found subtf_print: 0.0.0 (${subtf_print_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'subtf_print' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${subtf_print_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(subtf_print_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${subtf_print_DIR}/${_extra}")
endforeach()
