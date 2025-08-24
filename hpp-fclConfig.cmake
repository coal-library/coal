# This file provide bacward compatiblity for `find_package(hpp-fcl)`.

if(NOT COAL_DISABLE_HPP_FCL_WARNINGS)
  message(
    WARNING
    "Please update your CMake from 'hpp-fcl' to 'coal', or define COAL_DISABLE_HPP_FCL_WARNINGS"
  )
endif()

find_package(coal REQUIRED)

if(NOT TARGET hpp-fcl::hpp-fcl)
  add_library(hpp-fcl::hpp-fcl INTERFACE IMPORTED)

  # Compute the installation prefix relative to this file.
  # This code is taken from generated cmake xxxTargets.cmake.
  get_filename_component(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
  get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
  get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
  get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
  if(_IMPORT_PREFIX STREQUAL "/")
    set(_IMPORT_PREFIX "")
  endif()

  set_target_properties(
    hpp-fcl::hpp-fcl
    PROPERTIES
      INTERFACE_COMPILE_DEFINITIONS "COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL"
      INTERFACE_INCLUDE_DIRECTORIES "${_IMPORT_PREFIX}/include"
      INTERFACE_LINK_LIBRARIES "coal::coal"
  )
endif()
