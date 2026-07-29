function(coal_add_test test_name)
  add_executable(coal-test-${test_name} ${test_name}.cpp)
  target_link_libraries(
    coal-test-${test_name}
    PRIVATE Boost::unit_test_framework coal-tests-utility coal-tests-resources
  )
  jrl_target_enforce_msvc_conformance(coal-test-${test_name} PRIVATE)

  add_test(NAME coal-test-${test_name} COMMAND coal-test-${test_name})
  set_tests_properties(
    coal-test-${test_name}
    PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )
  if(ARGN)
    set_tests_properties(coal-test-${test_name} PROPERTIES ${ARGN})
  endif()
endfunction()

coal_add_test(math)
coal_add_test(collision)
coal_add_test(contact_patch)
coal_add_test(distance)
coal_add_test(swept_sphere_radius)
coal_add_test(normal_and_nearest_points)
coal_add_test(distance_lower_bound)
coal_add_test(security_margin)
coal_add_test(geometric_shapes)
coal_add_test(shape_inflation)
# #coal_add_test(shape_mesh_consistency)
coal_add_test(gjk_asserts)
coal_add_test(frontlist TIMEOUT 7200)

# coal_add_test(sphere_capsule)
coal_add_test(capsule_capsule)
coal_add_test(box_box_distance)
coal_add_test(box_box_collision)
coal_add_test(simple)
coal_add_test(capsule_box_1)
coal_add_test(capsule_box_2)
coal_add_test(convex)

coal_add_test(bvh_models)
coal_add_test(collision_node_asserts)
coal_add_test(hfields)

coal_add_test(gjk)
coal_add_test(accelerated_gjk)
coal_add_test(gjk_convergence_criterion)

if(COAL_BUILD_WITH_OCTOMAP)
  coal_add_test(octree)
endif()

coal_add_test(serialization)

# Broadphase
coal_add_test(broadphase)
coal_add_test(broadphase_dynamic_AABB_tree)
coal_add_test(broadphase_collision_1)
coal_add_test(broadphase_collision_2)

coal_add_test(alloca)
coal_add_test(patch_simplifier)
coal_add_test(polygon_convex_hull)
