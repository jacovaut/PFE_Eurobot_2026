# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_deadwheel_odometry_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED deadwheel_odometry_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(deadwheel_odometry_FOUND FALSE)
  elseif(NOT deadwheel_odometry_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(deadwheel_odometry_FOUND FALSE)
  endif()
  return()
endif()
set(_deadwheel_odometry_CONFIG_INCLUDED TRUE)

# output package information
if(NOT deadwheel_odometry_FIND_QUIETLY)
  message(STATUS "Found deadwheel_odometry: 0.0.0 (${deadwheel_odometry_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'deadwheel_odometry' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT deadwheel_odometry_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(deadwheel_odometry_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${deadwheel_odometry_DIR}/${_extra}")
endforeach()
