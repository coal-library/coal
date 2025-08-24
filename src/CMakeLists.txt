#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, 2020-2024 CNRS-LAAS INRIA
#  Author: Florent Lamiraux
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

set(LIBRARY_NAME ${PROJECT_NAME})
set(
  ${LIBRARY_NAME}_SOURCES
  collision.cpp
  contact_patch.cpp
  contact_patch/contact_patch_solver.cpp
  contact_patch_func_matrix.cpp
  distance_func_matrix.cpp
  collision_data.cpp
  collision_node.cpp
  collision_object.cpp
  BV/RSS.cpp
  BV/AABB.cpp
  BV/kIOS.cpp
  BV/kDOP.cpp
  BV/OBBRSS.cpp
  BV/OBB.cpp
  broadphase/default_broadphase_callbacks.cpp
  broadphase/broadphase_dynamic_AABB_tree.cpp
  broadphase/broadphase_dynamic_AABB_tree_array.cpp
  broadphase/broadphase_bruteforce.cpp
  broadphase/broadphase_collision_manager.cpp
  broadphase/broadphase_SaP.cpp
  broadphase/broadphase_SSaP.cpp
  broadphase/broadphase_interval_tree.cpp
  broadphase/detail/interval_tree.cpp
  broadphase/detail/interval_tree_node.cpp
  broadphase/detail/simple_interval.cpp
  broadphase/detail/spatial_hash.cpp
  broadphase/detail/morton.cpp
  narrowphase/gjk.cpp
  narrowphase/minkowski_difference.cpp
  narrowphase/support_functions.cpp
  narrowphase/details.h
  shape/geometric_shapes.cpp
  shape/geometric_shapes_utility.cpp
  distance/box_halfspace.cpp
  distance/box_plane.cpp
  distance/box_sphere.cpp
  distance/capsule_capsule.cpp
  distance/capsule_halfspace.cpp
  distance/capsule_plane.cpp
  distance/cone_halfspace.cpp
  distance/cone_plane.cpp
  distance/cylinder_halfspace.cpp
  distance/cylinder_plane.cpp
  distance/sphere_sphere.cpp
  distance/sphere_cylinder.cpp
  distance/sphere_halfspace.cpp
  distance/sphere_plane.cpp
  distance/sphere_capsule.cpp
  distance/ellipsoid_halfspace.cpp
  distance/ellipsoid_plane.cpp
  distance/convex_halfspace.cpp
  distance/convex_plane.cpp
  distance/triangle_halfspace.cpp
  distance/triangle_plane.cpp
  distance/triangle_triangle.cpp
  distance/triangle_sphere.cpp
  distance/halfspace_plane.cpp
  distance/plane_plane.cpp
  distance/halfspace_halfspace.cpp
  intersect.cpp
  math/transform.cpp
  traversal/traversal_recurse.cpp
  distance.cpp
  BVH/BVH_utility.cpp
  BVH/BV_fitter.cpp
  BVH/BVH_model.cpp
  BVH/BV_splitter.cpp
  collision_func_matrix.cpp
  collision_utility.cpp
  mesh_loader/assimp.cpp
  mesh_loader/loader.cpp
  hfield.cpp
  serialization/serialization.cpp
)

if(COAL_HAS_OCTOMAP)
  list(APPEND ${LIBRARY_NAME}_SOURCES octree.cpp)
endif(COAL_HAS_OCTOMAP)

set(PROJECT_HEADERS_FULL_PATH)
foreach(header ${${PROJECT_NAME}_HEADERS})
  list(APPEND PROJECT_HEADERS_FULL_PATH ${PROJECT_SOURCE_DIR}/${header})
endforeach()
list(
  APPEND
  PROJECT_HEADERS_FULL_PATH
  ${PROJECT_BINARY_DIR}/include/coal/config.hh
)
list(
  APPEND
  PROJECT_HEADERS_FULL_PATH
  ${PROJECT_BINARY_DIR}/include/coal/deprecated.hh
)
list(
  APPEND
  PROJECT_HEADERS_FULL_PATH
  ${PROJECT_BINARY_DIR}/include/coal/warning.hh
)
add_library(
  ${LIBRARY_NAME}
  SHARED
  ${PROJECT_HEADERS_FULL_PATH}
  ${${LIBRARY_NAME}_SOURCES}
)
add_library(${LIBRARY_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})
set_standard_output_directory(${LIBRARY_NAME})

if(COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL)
  add_library(hpp-fcl ALIAS ${LIBRARY_NAME})
  add_library(hpp-fcl::hpp-fcl ALIAS ${LIBRARY_NAME})
endif()

if(UNIX)
  GET_RELATIVE_RPATH(${CMAKE_INSTALL_LIBDIR} ${PROJECT_NAME}_INSTALL_RPATH)
  set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES INSTALL_RPATH "${${PROJECT_NAME}_INSTALL_RPATH}"
  )
endif()
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

CXX_FLAGS_BY_COMPILER_FRONTEND(MSVC "/bigobj" OUTPUT PUBLIC_OPTIONS FILTER)
target_compile_options(${PROJECT_NAME} PUBLIC ${PUBLIC_OPTIONS})

CXX_FLAGS_BY_COMPILER_FRONTEND(MSVC "NOMINMAX" OUTPUT PUBLIC_DEFINITIONS)
target_compile_definitions(${PROJECT_NAME} INTERFACE ${PUBLIC_DEFINITIONS})

# IDE sources and headers sorting
ADD_SOURCE_GROUP(${LIBRARY_NAME}_SOURCES)
ADD_HEADER_GROUP(PROJECT_HEADERS_FULL_PATH)

MODERNIZE_TARGET_LINK_LIBRARIES(
  ${LIBRARY_NAME}
  SCOPE PRIVATE
  TARGETS assimp::assimp
  LIBRARIES ${assimp_LIBRARIES}
  INCLUDE_DIRS ${assimp_INCLUDE_DIR}
)

target_link_libraries(
  ${LIBRARY_NAME}
  PUBLIC Boost::serialization Boost::chrono Boost::filesystem
)

if(COAL_ENABLE_LOGGING)
  target_link_libraries(${LIBRARY_NAME} PUBLIC Boost::log)
  # The compile flag `BOOST_LOG_DYN_LINK` is required here.
  target_compile_definitions(
    ${LIBRARY_NAME}
    PUBLIC COAL_ENABLE_LOGGING BOOST_LOG_DYN_LINK
  )
endif()

if(COAL_USE_FLOAT_PRECISION)
  target_compile_definitions(${LIBRARY_NAME} PUBLIC COAL_USE_FLOAT_PRECISION)
endif()

if(COAL_BUILD_WITH_TRACY)
  target_compile_definitions(${LIBRARY_NAME} PUBLIC COAL_TRACY_ENABLE)
  target_link_libraries(${LIBRARY_NAME} PUBLIC Tracy::TracyClient)
endif()

if(WIN32)
  target_link_libraries(
    ${LIBRARY_NAME}
    INTERFACE Boost::thread Boost::date_time
  )
  # There is an issue with MSVC 2017 and Eigen (due to std::aligned_storage).
  # See https://github.com/ceres-solver/ceres-solver/issues/481
  target_compile_definitions(
    ${LIBRARY_NAME}
    PRIVATE _ENABLE_EXTENDED_ALIGNED_STORAGE
  )
endif(WIN32)

if(COAL_TURN_ASSERT_INTO_EXCEPTION)
  target_compile_definitions(
    ${LIBRARY_NAME}
    PUBLIC COAL_TURN_ASSERT_INTO_EXCEPTION
  )
endif()

if(COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL)
  target_compile_definitions(
    ${LIBRARY_NAME}
    PUBLIC COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL
  )
endif()

if(COAL_HAS_QHULL)
  target_compile_definitions(${LIBRARY_NAME} PRIVATE COAL_HAS_QHULL)
  target_link_libraries(${LIBRARY_NAME} PRIVATE Qhull::qhull_r Qhull::qhullcpp)
endif()

MODERNIZE_TARGET_LINK_LIBRARIES(
  ${PROJECT_NAME}
  SCOPE PUBLIC
  TARGETS Eigen3::Eigen
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
)

target_include_directories(
  ${LIBRARY_NAME}
  PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

if(octomap_FOUND)
  MODERNIZE_TARGET_LINK_LIBRARIES(
    ${PROJECT_NAME}
    SCOPE PUBLIC
    TARGETS octomap
    LIBRARIES ${OCTOMAP_LIBRARIES}
    INCLUDE_DIRS ${OCTOMAP_INCLUDE_DIRS}
  )
  target_compile_definitions(
    ${LIBRARY_NAME}
    PUBLIC
      COAL_HAS_OCTOMAP
      COAL_HAVE_OCTOMAP
      OCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION}
      OCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION}
      OCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION}
  )
endif(octomap_FOUND)

install(
  TARGETS ${LIBRARY_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
