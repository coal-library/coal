#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2019-2023 CNRS-LAAS INRIA
#  Author: Joseph Mirabel
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of CNRS-LAAS. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.10)
set(CXX_DISABLE_WERROR TRUE)

set(PROJECT_NAME coal-python)
set(PROJECT_ORG "coal-library")
set(
  PROJECT_DESCRIPTION
  "Coal Python bindings"
)
set(PROJECT_URL "https://github.com/coal-library/coal")
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_COMPATIBILITY_VERSION AnyNewerVersion)
# To enable jrl-cmakemodules compatibility with workspace we must define the two
# following lines
set(PROJECT_AUTO_RUN_FINALIZE FALSE)
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR})

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/../cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_property(
      JRL_CMAKE_MODULES
      TARGET jrl-cmakemodules::jrl-cmakemodules
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    )
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  elseif(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
      "\nCan't find jrl-cmakemodules. Please either:\n"
      "  - use git submodule: 'git submodule update --init'\n"
      "  - or install https://github.com/jrl-umi3218/jrl-cmakemodules\n"
      "  - or upgrade your CMake version to >= 3.14 to allow automatic fetching\n"
    )
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git"
    )
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

# Use BoostConfig module distributed by boost library instead of using FindBoost module distributed
# by CMake.
# There is one unresolved issue with FindBoost and clang-cl so we deactivate it in this case.
if(NOT WIN32 OR NOT ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
  if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
    # Set a default value to this policy to avoid issue with find_dependency
    # macro redefinition with different policy in some modules.
    set(CMAKE_POLICY_DEFAULT_CMP0167 NEW)
  endif()
endif()

include("${JRL_CMAKE_MODULES}/base.cmake")
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

include("${JRL_CMAKE_MODULES}/boost.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")
include("${JRL_CMAKE_MODULES}/apple.cmake")
include("${JRL_CMAKE_MODULES}/ide.cmake")
include(CMakeDependentOption)

SET_DEFAULT_CMAKE_BUILD_TYPE("RelWithDebInfo")

# If needed, fix CMake policy for APPLE systems
APPLY_DEFAULT_APPLE_CONFIGURATION()

option(
  GENERATE_PYTHON_STUBS
  "Generate the Python stubs associated to the Python library"
  OFF
)

set(PYTHON_COMPONENTS Interpreter Development NumPy)
FINDPYTHON(REQUIRED)
if(NOT TARGET coal::coal)
  ADD_PROJECT_PRIVATE_DEPENDENCY(coal REQUIRED)
endif()
ADD_PROJECT_PRIVATE_DEPENDENCY(eigenpy 2.9.2 REQUIRED)
ADD_PROJECT_PRIVATE_DEPENDENCY(Boost REQUIRED system)

if(Boost_VERSION_STRING VERSION_LESS 1.81)
  # Default C++ version should be C++11
  CHECK_MINIMAL_CXX_STANDARD(11 ENFORCE)
else()
  # Boost.Math will be C++14 starting in July 2023 (Boost 1.82 release)
  CHECK_MINIMAL_CXX_STANDARD(14 ENFORCE)
endif()

include(${JRL_CMAKE_MODULES}/python-helpers.cmake)
include(${JRL_CMAKE_MODULES}/stubs.cmake)

add_custom_target(${PROJECT_NAME}_python)
set_target_properties(
  ${PROJECT_NAME}_python
  PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True
)

# Name of the Python library
set(PYTHON_LIB_NAME ${PROJECT_NAME}_pywrap)

set(
  ${PYTHON_LIB_NAME}_HEADERS
  fwd.hh
  coal.hh
  deprecation.hh
  broadphase/fwd.hh
  broadphase/broadphase_collision_manager.hh
  broadphase/broadphase_callbacks.hh
  pickle.hh
  utils/std-pair.hh
  serializable.hh
)

set(
  ENABLE_PYTHON_DOXYGEN_AUTODOC
  TRUE
  CACHE BOOL
  "Enable automatic documentation of Python bindings from Doxygen documentation"
)

if(NOT ENABLE_PYTHON_DOXYGEN_AUTODOC OR NOT DOXYGEN_FOUND)
  set(ENABLE_DOXYGEN_AUTODOC FALSE)
else()
  set(ENABLE_DOXYGEN_AUTODOC TRUE)

  if(
    DOXYGEN_VERSION VERSION_GREATER_EQUAL 1.9.8
    AND DOXYGEN_VERSION VERSION_LESS 1.11.0
  )
    # deactivate python doxygen automoc for doxygen 1.9.8 and 1.10.0,
    # as it incorrectly parse "const" keyword,
    # generating wrong links in C++ doc, and fail generating python doc
    # ref. https://github.com/doxygen/doxygen/issues/10797
    set(ENABLE_DOXYGEN_AUTODOC FALSE)
    message(
      AUTHOR_WARNING
      "automoc deactivated because of doxygen 1.10. Please use <1.10 or >=1.11."
    )
  endif()
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import lxml"
    RESULT_VARIABLE _pypkg_found
    OUTPUT_QUIET
    ERROR_QUIET
  )
  if(_pypkg_found EQUAL 0)
    message(STATUS "Found Python package lxml.")
  else()
    set(ENABLE_DOXYGEN_AUTODOC FALSE)
    message(
      STATUS
      "Python package lxml not found. Python bindings will not be documented."
    )
    message(STATUS "  You can install it with: pip install lxml")
  endif()

  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import pylatexenc"
    RESULT_VARIABLE _pypkg_found
    OUTPUT_QUIET
    ERROR_QUIET
  )
  if(NOT _pypkg_found EQUAL 0)
    message(STATUS "Python package pylatexenc not found.")
    message(
      STATUS
      "  Formulas in the Python bindings documentation may look ugly."
    )
    message(STATUS "  You can install it with: pip install pylatexenc")
  endif()
  unset(_pypkg_found)
endif()
if(ENABLE_DOXYGEN_AUTODOC)
  add_custom_target(
    generate_doxygen_cpp_doc
    COMMAND
      ${PYTHON_EXECUTABLE}
      ${PROJECT_SOURCE_DIR}/doc/python/doxygen_xml_parser.py
      ${PROJECT_BINARY_DIR}/doc/doxygen-xml/index.xml
      ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc >
      ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc.log
    BYPRODUCTS
      ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc/doxygen_xml_parser_for_cmake.hh
      ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc.log
    COMMENT "Generating Doxygen C++ documentation"
  )
  add_dependencies(generate_doxygen_cpp_doc doc)

  list(
    APPEND
    ${PYTHON_LIB_NAME}_HEADERS
    ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc/doxygen_xml_parser_for_cmake.hh
  )
endif()

set(
  ${PYTHON_LIB_NAME}_SOURCES
  version.cc
  math.cc
  collision-geometries.cc
  collision.cc
  contact_patch.cc
  distance.cc
  coal.cc
  gjk.cc
  broadphase/broadphase.cc
)

if(COAL_HAS_OCTOMAP)
  list(APPEND ${PYTHON_LIB_NAME}_SOURCES octree.cc)
endif(COAL_HAS_OCTOMAP)

add_library(
  ${PYTHON_LIB_NAME}
  MODULE
  ${${PYTHON_LIB_NAME}_SOURCES}
  ${${PYTHON_LIB_NAME}_HEADERS}
)
add_library(
  coal_pywrap ALIAS ${PYTHON_LIB_NAME})
target_include_directories(
  ${PYTHON_LIB_NAME}
  SYSTEM
  PRIVATE ${PYTHON_INCLUDE_DIRS}
)
target_include_directories(
  ${PYTHON_LIB_NAME}
  PRIVATE "${CMAKE_CURRENT_BINARY_DIR}"
)
if(WIN32)
  target_link_libraries(${PYTHON_LIB_NAME} PUBLIC ${PYTHON_LIBRARY})
endif(WIN32)

add_dependencies(${PROJECT_NAME}_python ${PYTHON_LIB_NAME})
ADD_HEADER_GROUP(${PYTHON_LIB_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PYTHON_LIB_NAME}_SOURCES)
if(ENABLE_DOXYGEN_AUTODOC)
  add_dependencies(${PYTHON_LIB_NAME} generate_doxygen_cpp_doc)
  target_compile_definitions(
    ${PYTHON_LIB_NAME}
    PRIVATE -DCOAL_HAS_DOXYGEN_AUTODOC
  )
endif()

target_link_libraries(
  ${PYTHON_LIB_NAME}
  PUBLIC coal::coal eigenpy::eigenpy Boost::system
)

set_target_properties(
  ${PYTHON_LIB_NAME}
  PROPERTIES
    PREFIX ""
    SUFFIX "${PYTHON_EXT_SUFFIX}"
    OUTPUT_NAME "coal_pywrap"
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/coal"
    # On Windows, shared library are treat as binary
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/coal"
)

if(IS_ABSOLUTE ${PYTHON_SITELIB})
  set(ABSOLUTE_PYTHON_SITELIB ${PYTHON_SITELIB})
else()
  set(ABSOLUTE_PYTHON_SITELIB ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB})
endif()
set(${PYTHON_LIB_NAME}_INSTALL_DIR ${ABSOLUTE_PYTHON_SITELIB}/coal)
if(UNIX)
  GET_RELATIVE_RPATH(
    ${${PYTHON_LIB_NAME}_INSTALL_DIR}
    ${PYTHON_LIB_NAME}_INSTALL_RPATH
  )
  set_target_properties(
    ${PYTHON_LIB_NAME}
    PROPERTIES INSTALL_RPATH "${${PYTHON_LIB_NAME}_INSTALL_RPATH}"
  )
endif()

install(
  TARGETS ${PYTHON_LIB_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION ${${PYTHON_LIB_NAME}_INSTALL_DIR}
)

# --- GENERATE STUBS
if(GENERATE_PYTHON_STUBS)
  LOAD_STUBGEN()
  GENERATE_STUBS(
    ${CMAKE_CURRENT_BINARY_DIR}
    coal
    ${ABSOLUTE_PYTHON_SITELIB}
    ${PYTHON_LIB_NAME}
  )
endif(GENERATE_PYTHON_STUBS)

# --- INSTALL SCRIPTS
set(PYTHON_FILES __init__.py viewer.py windows_dll_manager.py)

foreach(python ${PYTHON_FILES})
  PYTHON_BUILD(coal ${python})
  install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/coal/${python}"
    DESTINATION ${${PYTHON_LIB_NAME}_INSTALL_DIR}
  )
endforeach(python)

if(COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL)
  PYTHON_INSTALL_ON_SITE(hppfcl __init__.py COMPONENT hpp-fcl-compatibility)
  PYTHON_INSTALL_ON_SITE(hppfcl viewer.py COMPONENT hpp-fcl-compatibility)
endif()

SETUP_PROJECT_FINALIZE()
