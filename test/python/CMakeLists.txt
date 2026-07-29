if(COAL_PYTHON_NANOBIND)
  set(coal_python_bindings coal_pywrap_nb)
else()
  set(coal_python_bindings coal_pywrap)
endif()

function(coal_python_add_test name)
  set(test_name "coal: Python ${name}")
  add_test(
    NAME ${test_name}
    COMMAND $<TARGET_FILE:Python::Interpreter> ${CMAKE_CURRENT_SOURCE_DIR}/test_${name}.py
  )
  set_tests_properties(
    ${test_name}
    PROPERTIES
      ENVIRONMENT_MODIFICATION
        "PYTHONPATH=path_list_prepend:$<TARGET_FILE_DIR:${coal_python_bindings}>/.."
      LABELS "python"
  )
endfunction()

coal_python_add_test(api)
coal_python_add_test(collision_manager)
coal_python_add_test(collision)
coal_python_add_test(geometric_shapes)
coal_python_add_test(pickling)
