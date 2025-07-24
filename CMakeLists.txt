#
# Software License Agreement (BSD License)
#
# Copyright (c) 2014-2025 CNRS-LAAS, INRIA Author: Florent Lamiraux, Joseph
# Mirabel All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of CNRS-LAAS. nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.22)
set(CXX_DISABLE_WERROR TRUE)

set(PROJECT_NAME coal)
set(PROJECT_ORG "coal-library")
set(
  PROJECT_DESCRIPTION
  "Coal, The Collision Detection Library. Previously known as HPP-FCL, fork of FCL -- The Flexible Collision Library"
)
set(PROJECT_URL "https://github.com/coal-library/coal")
if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif()
set(PROJECT_COMPATIBILITY_VERSION AnyNewerVersion)
# To enable jrl-cmakemodules compatibility with workspace we must define the two
# following lines
set(PROJECT_AUTO_RUN_FINALIZE FALSE)
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR})

set(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)
set(DOXYGEN_USE_TEMPLATE_CSS TRUE)

# ----------------------------------------------------
# --- OPTIONS  ---------------------------------------
# Need to be set before including base.cmake
# ----------------------------------------------------
option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
option(
  COAL_TURN_ASSERT_INTO_EXCEPTION
  "Turn some critical Coal asserts to exception."
  FALSE
)
option(
  COAL_ENABLE_LOGGING
  "Activate logging for warnings or error messages. Turned on by default in Debug."
  FALSE
)
option(
  COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL
  "Make Coal retro-compatible with HPP-FCL."
  FALSE
)
option(
  COAL_USE_FLOAT_PRECISION
  "Use float precision (32-bit) instead of the default double precision (64-bit)"
  FALSE
)
option(
  COAL_BUILD_WITH_TRACY
  "Build with tracy profiler for performance analysis"
  FALSE
)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
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

# Use BoostConfig module distributed by boost library instead of using FindBoost
# module distributed by CMake. There is one unresolved issue with FindBoost and
# clang-cl so we deactivate it in this case.
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
include("${JRL_CMAKE_MODULES}/tracy.cmake")
include(CMakeDependentOption)

set(
  CMAKE_MODULE_PATH
  ${JRL_CMAKE_MODULES}/find-external/assimp/
  ${CMAKE_MODULE_PATH}
)

function(set_standard_output_directory target)
  set_target_properties(
    ${target}
    PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
  )
endfunction()

SET_DEFAULT_CMAKE_BUILD_TYPE("RelWithDebInfo")

# If needed, fix CMake policy for APPLE systems
APPLY_DEFAULT_APPLE_CONFIGURATION()

cmake_dependent_option(
  COAL_PYTHON_NANOBIND
  "Build the nanobind-based bindings."
  OFF
  BUILD_PYTHON_INTERFACE
  OFF
)
# Stubs only optional for legacy Boost.Python bindings
if(NOT COAL_PYTHON_NANOBIND)
  cmake_dependent_option(
    GENERATE_PYTHON_STUBS
    "Generate the Python stubs for the Python bindings (only for legacy Boost.Python bindings)"
    OFF
    BUILD_PYTHON_INTERFACE
    OFF
  )
endif()

ADD_PROJECT_DEPENDENCY(Eigen3 REQUIRED PKG_CONFIG_REQUIRES "eigen3 >= 3.0.0")

# -- tracy profiling (optional)
if(COAL_BUILD_WITH_TRACY)
  # assume it is installed somewhere
  ADD_PROJECT_DEPENDENCY(Tracy REQUIRED)
  set_target_properties(
    Tracy::TracyClient
    PROPERTIES POSITION_INDEPENDENT_CODE True
  )
endif(COAL_BUILD_WITH_TRACY)

# Required dependencies
SET_BOOST_DEFAULT_OPTIONS()
EXPORT_BOOST_DEFAULT_OPTIONS()
ADD_PROJECT_DEPENDENCY(
  Boost
  REQUIRED
  chrono
  thread
  date_time
  serialization
  filesystem
)
if(COAL_ENABLE_LOGGING)
  ADD_PROJECT_DEPENDENCY(Boost REQUIRED log)
endif()

if(Boost_VERSION_STRING VERSION_LESS 1.81)
  # Default C++ version should be C++11
  CHECK_MINIMAL_CXX_STANDARD(11 ENFORCE)
else()
  # Boost.Math will be C++14 starting in July 2023 (Boost 1.82 release)
  CHECK_MINIMAL_CXX_STANDARD(14 ENFORCE)
endif()

# Optional dependencies
ADD_PROJECT_DEPENDENCY(octomap 1.8.0 PKG_CONFIG_REQUIRES "octomap >= 1.8")
if(octomap_FOUND)
  set(COAL_HAS_OCTOMAP TRUE)
  string(REPLACE "." ";" VERSION_LIST ${octomap_VERSION})
  list(GET VERSION_LIST 0 OCTOMAP_MAJOR_VERSION)
  list(GET VERSION_LIST 1 OCTOMAP_MINOR_VERSION)
  list(GET VERSION_LIST 2 OCTOMAP_PATCH_VERSION)
  message(STATUS "COAL uses Octomap")
else()
  set(COAL_HAS_OCTOMAP FALSE)
  message(STATUS "COAL does not use Octomap")
endif()

option(COAL_HAS_QHULL "use qhull library to compute convex hulls." FALSE)
if(COAL_HAS_QHULL)
  find_package(Qhull REQUIRED COMPONENTS qhull_r qhullcpp)
endif()

find_package(assimp REQUIRED)

set(
  ${PROJECT_NAME}_HEADERS
  include/coal/collision_data.h
  include/coal/BV/kIOS.h
  include/coal/BV/BV.h
  include/coal/BV/RSS.h
  include/coal/BV/OBBRSS.h
  include/coal/BV/BV_node.h
  include/coal/BV/AABB.h
  include/coal/BV/OBB.h
  include/coal/BV/kDOP.h
  include/coal/broadphase/broadphase.h
  include/coal/broadphase/broadphase_SSaP.h
  include/coal/broadphase/broadphase_SaP.h
  include/coal/broadphase/broadphase_bruteforce.h
  include/coal/broadphase/broadphase_collision_manager.h
  include/coal/broadphase/broadphase_continuous_collision_manager-inl.h
  include/coal/broadphase/broadphase_continuous_collision_manager.h
  include/coal/broadphase/broadphase_dynamic_AABB_tree-inl.h
  include/coal/broadphase/broadphase_dynamic_AABB_tree.h
  include/coal/broadphase/broadphase_dynamic_AABB_tree_array-inl.h
  include/coal/broadphase/broadphase_dynamic_AABB_tree_array.h
  include/coal/broadphase/broadphase_interval_tree.h
  include/coal/broadphase/broadphase_spatialhash-inl.h
  include/coal/broadphase/broadphase_spatialhash.h
  include/coal/broadphase/broadphase_callbacks.h
  include/coal/broadphase/default_broadphase_callbacks.h
  include/coal/broadphase/detail/hierarchy_tree-inl.h
  include/coal/broadphase/detail/hierarchy_tree.h
  include/coal/broadphase/detail/hierarchy_tree_array-inl.h
  include/coal/broadphase/detail/hierarchy_tree_array.h
  include/coal/broadphase/detail/interval_tree.h
  include/coal/broadphase/detail/interval_tree_node.h
  include/coal/broadphase/detail/morton-inl.h
  include/coal/broadphase/detail/morton.h
  include/coal/broadphase/detail/node_base-inl.h
  include/coal/broadphase/detail/node_base.h
  include/coal/broadphase/detail/node_base_array-inl.h
  include/coal/broadphase/detail/node_base_array.h
  include/coal/broadphase/detail/simple_hash_table-inl.h
  include/coal/broadphase/detail/simple_hash_table.h
  include/coal/broadphase/detail/simple_interval-inl.h
  include/coal/broadphase/detail/simple_interval.h
  include/coal/broadphase/detail/sparse_hash_table-inl.h
  include/coal/broadphase/detail/sparse_hash_table.h
  include/coal/broadphase/detail/spatial_hash-inl.h
  include/coal/broadphase/detail/spatial_hash.h
  include/coal/narrowphase/narrowphase.h
  include/coal/narrowphase/gjk.h
  include/coal/narrowphase/narrowphase_defaults.h
  include/coal/narrowphase/minkowski_difference.h
  include/coal/narrowphase/support_data.h
  include/coal/narrowphase/support_functions.h
  include/coal/shape/convex.h
  include/coal/shape/convex.hxx
  include/coal/shape/geometric_shape_to_BVH_model.h
  include/coal/shape/geometric_shapes.h
  include/coal/shape/geometric_shapes.hxx
  include/coal/shape/geometric_shapes_traits.h
  include/coal/shape/geometric_shapes_utility.h
  include/coal/distance_func_matrix.h
  include/coal/collision.h
  include/coal/collision_func_matrix.h
  include/coal/contact_patch.h
  include/coal/contact_patch_func_matrix.h
  include/coal/contact_patch/contact_patch_solver.h
  include/coal/contact_patch/contact_patch_solver.hxx
  include/coal/distance.h
  include/coal/math/matrix_3f.h
  include/coal/math/vec_3f.h
  include/coal/math/types.h
  include/coal/math/transform.h
  include/coal/data_types.h
  include/coal/BVH/BVH_internal.h
  include/coal/BVH/BVH_model.h
  include/coal/BVH/BVH_front.h
  include/coal/BVH/BVH_utility.h
  include/coal/collision_object.h
  include/coal/collision_utility.h
  include/coal/hfield.h
  include/coal/fwd.hh
  include/coal/logging.h
  include/coal/mesh_loader/assimp.h
  include/coal/mesh_loader/loader.h
  include/coal/internal/BV_fitter.h
  include/coal/internal/BV_splitter.h
  include/coal/internal/shape_shape_func.h
  include/coal/internal/shape_shape_contact_patch_func.h
  include/coal/internal/intersect.h
  include/coal/internal/intersect.hxx
  include/coal/internal/tools.h
  include/coal/internal/traversal_node_base.h
  include/coal/internal/traversal_node_bvh_shape.h
  include/coal/internal/traversal_node_bvhs.h
  include/coal/internal/traversal_node_hfield_shape.h
  include/coal/internal/traversal_node_setup.h
  include/coal/internal/traversal_node_shapes.h
  include/coal/internal/traversal_recurse.h
  include/coal/internal/traversal.h
  include/coal/serialization/fwd.h
  include/coal/serialization/serializer.h
  include/coal/serialization/archive.h
  include/coal/serialization/transform.h
  include/coal/serialization/AABB.h
  include/coal/serialization/BV_node.h
  include/coal/serialization/BV_splitter.h
  include/coal/serialization/BVH_model.h
  include/coal/serialization/collision_data.h
  include/coal/serialization/contact_patch.h
  include/coal/serialization/collision_object.h
  include/coal/serialization/convex.h
  include/coal/serialization/eigen.h
  include/coal/serialization/geometric_shapes.h
  include/coal/serialization/memory.h
  include/coal/serialization/OBB.h
  include/coal/serialization/RSS.h
  include/coal/serialization/OBBRSS.h
  include/coal/serialization/kIOS.h
  include/coal/serialization/kDOP.h
  include/coal/serialization/hfield.h
  include/coal/serialization/quadrilateral.h
  include/coal/serialization/triangle.h
  include/coal/timings.h
)

if(
  COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL
  AND NOT BUILD_STANDALONE_PYTHON_INTERFACE
)
  set(
    HPP_FCL_BACKWARD_COMPATIBILITY_HEADERS
    include/hpp/fcl/broadphase/broadphase_bruteforce.h
    include/hpp/fcl/broadphase/broadphase_callbacks.h
    include/hpp/fcl/broadphase/broadphase_collision_manager.h
    include/hpp/fcl/broadphase/broadphase_continuous_collision_manager.h
    include/hpp/fcl/broadphase/broadphase_continuous_collision_manager-inl.h
    include/hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array.h
    include/hpp/fcl/broadphase/broadphase_dynamic_AABB_tree_array-inl.h
    include/hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h
    include/hpp/fcl/broadphase/broadphase_dynamic_AABB_tree-inl.h
    include/hpp/fcl/broadphase/broadphase.h
    include/hpp/fcl/broadphase/broadphase_interval_tree.h
    include/hpp/fcl/broadphase/broadphase_SaP.h
    include/hpp/fcl/broadphase/broadphase_spatialhash.h
    include/hpp/fcl/broadphase/broadphase_spatialhash-inl.h
    include/hpp/fcl/broadphase/broadphase_SSaP.h
    include/hpp/fcl/broadphase/default_broadphase_callbacks.h
    include/hpp/fcl/broadphase/detail/hierarchy_tree_array.h
    include/hpp/fcl/broadphase/detail/hierarchy_tree_array-inl.h
    include/hpp/fcl/broadphase/detail/hierarchy_tree.h
    include/hpp/fcl/broadphase/detail/hierarchy_tree-inl.h
    include/hpp/fcl/broadphase/detail/interval_tree.h
    include/hpp/fcl/broadphase/detail/interval_tree_node.h
    include/hpp/fcl/broadphase/detail/morton.h
    include/hpp/fcl/broadphase/detail/morton-inl.h
    include/hpp/fcl/broadphase/detail/node_base_array.h
    include/hpp/fcl/broadphase/detail/node_base_array-inl.h
    include/hpp/fcl/broadphase/detail/node_base.h
    include/hpp/fcl/broadphase/detail/node_base-inl.h
    include/hpp/fcl/broadphase/detail/simple_hash_table.h
    include/hpp/fcl/broadphase/detail/simple_hash_table-inl.h
    include/hpp/fcl/broadphase/detail/simple_interval.h
    include/hpp/fcl/broadphase/detail/simple_interval-inl.h
    include/hpp/fcl/broadphase/detail/sparse_hash_table.h
    include/hpp/fcl/broadphase/detail/sparse_hash_table-inl.h
    include/hpp/fcl/broadphase/detail/spatial_hash.h
    include/hpp/fcl/broadphase/detail/spatial_hash-inl.h
    include/hpp/fcl/BV/AABB.h
    include/hpp/fcl/BV/BV.h
    include/hpp/fcl/BV/BV_node.h
    include/hpp/fcl/BVH/BVH_front.h
    include/hpp/fcl/BVH/BVH_internal.h
    include/hpp/fcl/BVH/BVH_model.h
    include/hpp/fcl/BVH/BVH_utility.h
    include/hpp/fcl/BV/kDOP.h
    include/hpp/fcl/BV/kIOS.h
    include/hpp/fcl/BV/OBB.h
    include/hpp/fcl/BV/OBBRSS.h
    include/hpp/fcl/BV/RSS.h
    include/hpp/fcl/coal.hpp
    include/hpp/fcl/collision_data.h
    include/hpp/fcl/collision_func_matrix.h
    include/hpp/fcl/collision.h
    include/hpp/fcl/collision_object.h
    include/hpp/fcl/collision_utility.h
    include/hpp/fcl/config.hh
    include/hpp/fcl/contact_patch/contact_patch_solver.h
    include/hpp/fcl/contact_patch/contact_patch_solver.hxx
    include/hpp/fcl/contact_patch_func_matrix.h
    include/hpp/fcl/contact_patch.h
    include/hpp/fcl/data_types.h
    include/hpp/fcl/deprecated.hh
    include/hpp/fcl/distance_func_matrix.h
    include/hpp/fcl/distance.h
    include/hpp/fcl/fwd.hh
    include/hpp/fcl/hfield.h
    include/hpp/fcl/internal/BV_fitter.h
    include/hpp/fcl/internal/BV_splitter.h
    include/hpp/fcl/internal/intersect.h
    include/hpp/fcl/internal/shape_shape_contact_patch_func.h
    include/hpp/fcl/internal/shape_shape_func.h
    include/hpp/fcl/internal/tools.h
    include/hpp/fcl/internal/traversal.h
    include/hpp/fcl/internal/traversal_node_base.h
    include/hpp/fcl/internal/traversal_node_bvhs.h
    include/hpp/fcl/internal/traversal_node_bvh_shape.h
    include/hpp/fcl/internal/traversal_node_hfield_shape.h
    include/hpp/fcl/internal/traversal_node_setup.h
    include/hpp/fcl/internal/traversal_node_shapes.h
    include/hpp/fcl/internal/traversal_recurse.h
    include/hpp/fcl/internal/traversal_node_octree.h
    include/hpp/fcl/logging.h
    include/hpp/fcl/math/matrix_3f.h
    include/hpp/fcl/math/transform.h
    include/hpp/fcl/math/types.h
    include/hpp/fcl/math/vec_3f.h
    include/hpp/fcl/mesh_loader/assimp.h
    include/hpp/fcl/mesh_loader/loader.h
    include/hpp/fcl/narrowphase/gjk.h
    include/hpp/fcl/narrowphase/minkowski_difference.h
    include/hpp/fcl/narrowphase/narrowphase_defaults.h
    include/hpp/fcl/narrowphase/narrowphase.h
    include/hpp/fcl/narrowphase/support_data.h
    include/hpp/fcl/narrowphase/support_functions.h
    include/hpp/fcl/narrowphase/support_functions.hxx
    include/hpp/fcl/octree.h
    include/hpp/fcl/serialization/AABB.h
    include/hpp/fcl/serialization/archive.h
    include/hpp/fcl/serialization/BVH_model.h
    include/hpp/fcl/serialization/BV_node.h
    include/hpp/fcl/serialization/BV_splitter.h
    include/hpp/fcl/serialization/collision_data.h
    include/hpp/fcl/serialization/collision_object.h
    include/hpp/fcl/serialization/contact_patch.h
    include/hpp/fcl/serialization/convex.h
    include/hpp/fcl/serialization/eigen.h
    include/hpp/fcl/serialization/fwd.h
    include/hpp/fcl/serialization/geometric_shapes.h
    include/hpp/fcl/serialization/hfield.h
    include/hpp/fcl/serialization/kDOP.h
    include/hpp/fcl/serialization/kIOS.h
    include/hpp/fcl/serialization/memory.h
    include/hpp/fcl/serialization/OBB.h
    include/hpp/fcl/serialization/OBBRSS.h
    include/hpp/fcl/serialization/octree.h
    include/hpp/fcl/serialization/quadrilateral.h
    include/hpp/fcl/serialization/RSS.h
    include/hpp/fcl/serialization/serializer.h
    include/hpp/fcl/serialization/transform.h
    include/hpp/fcl/serialization/triangle.h
    include/hpp/fcl/shape/convex.h
    include/hpp/fcl/shape/convex.hxx
    include/hpp/fcl/shape/geometric_shapes.h
    include/hpp/fcl/shape/geometric_shapes.hxx
    include/hpp/fcl/shape/geometric_shapes_traits.h
    include/hpp/fcl/shape/geometric_shapes_utility.h
    include/hpp/fcl/shape/geometric_shape_to_BVH_model.h
    include/hpp/fcl/timings.h
    include/hpp/fcl/warning.hh
  )
  list(APPEND ${PROJECT_NAME}_HEADERS ${HPP_FCL_BACKWARD_COMPATIBILITY_HEADERS})
  HEADER_INSTALL(
    COMPONENT hpp-fcl-compatibility
    ${HPP_FCL_BACKWARD_COMPATIBILITY_HEADERS}
  )
endif()

if(COAL_HAS_OCTOMAP)
  list(
    APPEND
    ${PROJECT_NAME}_HEADERS
    include/coal/octree.h
    include/coal/serialization/octree.h
    include/coal/internal/traversal_node_octree.h
  )
endif(COAL_HAS_OCTOMAP)

add_subdirectory(doc)
if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  add_subdirectory(src)
endif()
if(BUILD_PYTHON_INTERFACE)
  if(BUILD_STANDALONE_PYTHON_INTERFACE)
    ADD_PROJECT_DEPENDENCY(${PROJECT_NAME} REQUIRED CONFIG)
  endif()
  add_custom_target(${PROJECT_NAME}_python)
  set_target_properties(
    ${PROJECT_NAME}_python
    PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True
  )
  if(COAL_PYTHON_NANOBIND)
    find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
    # jrl-cmakemodules use PYTHON_EXECUTABLE defined in FINDPYTHON macro. Since
    # we doesn't call it in this branch, we must redefine it.
    set(PYTHON_EXECUTABLE ${Python_EXECUTABLE})
    add_subdirectory(python-nb)
  else()
    set(PYTHON_COMPONENTS Interpreter Development NumPy)
    FINDPYTHON(REQUIRED)
    ADD_PROJECT_PRIVATE_DEPENDENCY(eigenpy 2.9.2 REQUIRED)
    add_subdirectory(python)
  endif()
endif()
if(BUILD_TESTING)
  add_subdirectory(test)
endif(BUILD_TESTING)

PKG_CONFIG_APPEND_LIBS("coal")
if(COAL_HAS_OCTOMAP)
  # FCL_HAVE_OCTOMAP kept for backward compatibility reasons.
  PKG_CONFIG_APPEND_CFLAGS(
    "-DCOAL_HAS_OCTOMAP -DCOAL_HAVE_OCTOMAP -DFCL_HAVE_OCTOMAP -DOCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION} -DOCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION} -DOCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION}"
  )
endif(COAL_HAS_OCTOMAP)

if(
  COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL
  AND NOT BUILD_STANDALONE_PYTHON_INTERFACE
)
  include(CMakePackageConfigHelpers)
  write_basic_package_version_file(
    hpp-fclConfigVersion.cmake
    VERSION 3.0.0
    COMPATIBILITY AnyNewerVersion
  )
  install(
    FILES
      hpp-fclConfig.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/hpp-fclConfigVersion.cmake
    DESTINATION lib/cmake/hpp-fcl
    COMPONENT hpp-fcl-compatibility
  )
  include("${JRL_CMAKE_MODULES}/install-helpers.cmake")
  ADD_INSTALL_TARGET(NAME hpp-fcl-compatibility COMPONENT hpp-fcl-compatibility)
endif()

SETUP_PROJECT_FINALIZE()
