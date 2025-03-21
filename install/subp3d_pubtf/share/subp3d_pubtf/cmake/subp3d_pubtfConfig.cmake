# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_subp3d_pubtf_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED subp3d_pubtf_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(subp3d_pubtf_FOUND FALSE)
  elseif(NOT subp3d_pubtf_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(subp3d_pubtf_FOUND FALSE)
  endif()
  return()
endif()
set(_subp3d_pubtf_CONFIG_INCLUDED TRUE)

# output package information
if(NOT subp3d_pubtf_FIND_QUIETLY)
  message(STATUS "Found subp3d_pubtf: 0.0.0 (${subp3d_pubtf_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'subp3d_pubtf' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${subp3d_pubtf_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(subp3d_pubtf_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${subp3d_pubtf_DIR}/${_extra}")
endforeach()
