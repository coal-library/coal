set(
  ${PROJECT_NAME}_PYTHON_TESTS
  geometric_shapes
  api
  collision
  collision_manager
  pickling
)

if(COAL_PYTHON_NANOBIND)
  set(PYTHON_BINDINGS_BINARY_DIR "python-nb")
else()
  set(PYTHON_BINDINGS_BINARY_DIR "python")
endif()

message(
  STATUS
  "  Python tests will be run with Python executable at ${Python_EXECUTABLE}"
)

add_dependencies(build_tests ${PROJECT_NAME}_python)
foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  ADD_PYTHON_UNIT_TEST(
    "${PROJECT_NAME}-py-${TEST}"
    "test/python_unit/${TEST}.py"
    ${PYTHON_BINDINGS_BINARY_DIR}
  )
endforeach()
