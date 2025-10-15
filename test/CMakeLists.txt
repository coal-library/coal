# Find Boost.UnitTestFramework
if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework filesystem)

  CONFIG_FILES(fcl_resources/config.h)

  function(add_coal_test test_name source)
    set(target_name ${PROJECT_NAME}-${test_name})
    ADD_UNIT_TEST(${target_name} ${source})
    set_standard_output_directory(${target_name})
    target_link_libraries(
      ${target_name}
      PUBLIC ${LIBRARY_NAME} Boost::filesystem ${utility_target}
    )
    CXX_FLAGS_BY_COMPILER_FRONTEND(
      GNU "-Wno-c99-extensions"
      OUTPUT PRIVATE_OPTIONS
    )
    target_compile_options(${target_name} PRIVATE ${PRIVATE_OPTIONS})
    if(COAL_HAS_QHULL)
      target_compile_definitions(${target_name} PRIVATE COAL_HAS_QHULL)
    endif()
  endfunction()

  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  set(utility_target ${PROJECT_NAME}-utility)
  add_library(${utility_target} STATIC utility.cpp)
  set_standard_output_directory(${utility_target})
  target_link_libraries(${utility_target} PUBLIC ${PROJECT_NAME})

  add_coal_test(math math.cpp)

  add_coal_test(collision collision.cpp)
  add_coal_test(contact_patch contact_patch.cpp)
  add_coal_test(distance distance.cpp)
  add_coal_test(swept_sphere_radius swept_sphere_radius.cpp)
  add_coal_test(normal_and_nearest_points normal_and_nearest_points.cpp)
  add_coal_test(distance_lower_bound distance_lower_bound.cpp)
  add_coal_test(security_margin security_margin.cpp)
  add_coal_test(geometric_shapes geometric_shapes.cpp)
  add_coal_test(shape_inflation shape_inflation.cpp)
  #add_coal_test(shape_mesh_consistency shape_mesh_consistency.cpp)
  add_coal_test(gjk_asserts gjk_asserts.cpp)
  add_coal_test(frontlist frontlist.cpp)
  set_tests_properties(${PROJECT_NAME}-frontlist PROPERTIES TIMEOUT 7200)

  # add_coal_test(sphere_capsule sphere_capsule.cpp)
  add_coal_test(capsule_capsule capsule_capsule.cpp)
  add_coal_test(box_box_distance box_box_distance.cpp)
  add_coal_test(box_box_collision box_box_collision.cpp)
  add_coal_test(simple simple.cpp)
  add_coal_test(capsule_box_1 capsule_box_1.cpp)
  add_coal_test(capsule_box_2 capsule_box_2.cpp)
  add_coal_test(obb obb.cpp)
  add_coal_test(convex convex.cpp)

  add_coal_test(bvh_models bvh_models.cpp)
  add_coal_test(collision_node_asserts collision_node_asserts.cpp)
  add_coal_test(hfields hfields.cpp)

  add_coal_test(profiling profiling.cpp)

  add_coal_test(gjk gjk.cpp)
  add_coal_test(accelerated_gjk accelerated_gjk.cpp)
  add_coal_test(gjk_convergence_criterion gjk_convergence_criterion.cpp)
  if(COAL_HAS_OCTOMAP)
    add_coal_test(octree octree.cpp)
  endif(COAL_HAS_OCTOMAP)

  add_coal_test(serialization serialization.cpp)

  # Broadphase
  add_coal_test(broadphase broadphase.cpp)
  set_tests_properties(${PROJECT_NAME}-broadphase PROPERTIES WILL_FAIL TRUE)
  add_coal_test(broadphase_dynamic_AABB_tree broadphase_dynamic_AABB_tree.cpp)
  add_coal_test(broadphase_collision_1 broadphase_collision_1.cpp)
  add_coal_test(broadphase_collision_2 broadphase_collision_2.cpp)

  ## Benchmark
  set(test_benchmark_target ${PROJECT_NAME}-test-benchmark)
  add_executable(${test_benchmark_target} benchmark.cpp)
  set_standard_output_directory(${test_benchmark_target})
  target_link_libraries(
    ${test_benchmark_target}
    PUBLIC ${utility_target} Boost::filesystem ${PROJECT_NAME}
  )
endif()

## Python tests
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python_unit)
endif(BUILD_PYTHON_INTERFACE)
