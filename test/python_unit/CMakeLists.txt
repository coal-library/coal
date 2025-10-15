set(
  ${PROJECT_NAME}_PYTHON_TESTS
  geometric_shapes
  api
  collision
  collision_manager
  pickling
)

add_dependencies(build_tests ${PROJECT_NAME}_pywrap)
foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  ADD_PYTHON_UNIT_TEST(
    "${PROJECT_NAME}-py-${TEST}"
    "test/python_unit/${TEST}.py"
    "python"
  )
endforeach()
