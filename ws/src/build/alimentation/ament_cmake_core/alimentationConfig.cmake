# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_alimentation_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED alimentation_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(alimentation_FOUND FALSE)
  elseif(NOT alimentation_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(alimentation_FOUND FALSE)
  endif()
  return()
endif()
set(_alimentation_CONFIG_INCLUDED TRUE)

# output package information
if(NOT alimentation_FIND_QUIETLY)
  message(STATUS "Found alimentation: 0.0.0 (${alimentation_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'alimentation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT alimentation_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(alimentation_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${alimentation_DIR}/${_extra}")
endforeach()
