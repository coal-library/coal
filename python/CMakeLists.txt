set(
  coal_pywrap_HEADERS
  fwd.hh
  coal.hh
  deprecation.hh
  broadphase/fwd.hh
  broadphase/broadphase_collision_manager.hh
  broadphase/broadphase_callbacks.hh
  pickle.hh
  utils/std-pair.hh
  serializable.hh
  printable.hh
)

if(ENABLE_PYTHON_DOXYGEN_AUTODOC)
  add_custom_target(
    generate_doxygen_cpp_doc
    COMMAND
      $<TARGET_FILE:Python::Interpreter> ${PROJECT_SOURCE_DIR}/doc/python/doxygen_xml_parser.py
      ${PROJECT_BINARY_DIR}/doc/doxygen-xml/index.xml ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc >
      ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc.log
    BYPRODUCTS
      ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc/doxygen_xml_parser_for_cmake.hh
      ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc.log
    COMMENT "Generating Doxygen C++ documentation"
  )
  add_dependencies(generate_doxygen_cpp_doc doc)

  list(
    APPEND coal_pywrap_HEADERS
    ${CMAKE_CURRENT_BINARY_DIR}/doxygen_autodoc/doxygen_xml_parser_for_cmake.hh
  )
endif()

set(
  coal_pywrap_SOURCES
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

if(COAL_BUILD_WITH_OCTOMAP)
  list(APPEND coal_pywrap_SOURCES octree.cc)
endif()

jrl_boostpy_add_module(coal_pywrap ${coal_pywrap_SOURCES} ${coal_pywrap_HEADERS})
jrl_target_enforce_msvc_conformance(coal_pywrap PRIVATE)
jrl_check_python_module_name(coal_pywrap)
target_link_libraries(coal_pywrap PRIVATE eigenpy::eigenpy coal)
jrl_target_set_default_compile_options(coal_pywrap PRIVATE)
jrl_target_set_output_directory(coal_pywrap OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/site-packages/coal)

if(ENABLE_PYTHON_DOXYGEN_AUTODOC)
  # The sources include the generated headers as doxygen_autodoc/<...>.h, and
  # those in turn include the documented private headers as <narrowphase/...>
  target_include_directories(
    coal_pywrap
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${PROJECT_SOURCE_DIR}/src
  )
  add_dependencies(coal_pywrap generate_doxygen_cpp_doc)
  target_compile_definitions(coal_pywrap PRIVATE COAL_HAS_DOXYGEN_AUTODOC)
endif()

jrl_python_generate_init_py(
    coal_pywrap
    OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/site-packages/coal/__init__.py
)

if(GENERATE_PYTHON_STUBS)
  jrl_boostpy_add_stubs(
      coal_pywrap_stubs
      MODULE coal.coal_pywrap
      OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/site-packages
      PYTHON_PATH ${CMAKE_BINARY_DIR}/lib/site-packages
      DEPENDS coal_pywrap
      VERBOSE
  )
endif()

jrl_python_compile_all(DIRECTORY
    ${CMAKE_BINARY_DIR}/lib/site-packages/coal
    VERBOSE
)

jrl_python_compute_install_dir(python_install_dir)
install(TARGETS coal_pywrap DESTINATION ${python_install_dir}/coal)
install(
  DIRECTORY ${CMAKE_BINARY_DIR}/lib/site-packages/coal ${CMAKE_CURRENT_SOURCE_DIR}/coal
  DESTINATION ${python_install_dir}
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "*.pyc"
  PATTERN "*.pyi"
  PATTERN "*.typed"
  PATTERN "*.so"
  PATTERN "*.pyd"
)
