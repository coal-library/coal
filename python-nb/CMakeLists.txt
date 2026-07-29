set(
  coal_pywrap_nb_HEADERS
  broadphase/broadphase_callbacks_collision_manager.hh
  fwd.h
  pickle.hh
  serializable.hh
)

set(
  coal_wrap_nb_SOURCES
  broadphase/broadphase.cc
  aabb.cc
  bvh.cc
  collision-geometries.cc
  collision.cc
  contact_patch.cc
  distance.cc
  math.cc
  gjk.cc
  height_field.cc
  memory-footprint.cc
  shapes.cc
  module.cc
)
if(COAL_BUILD_WITH_OCTOMAP)
  list(APPEND coal_wrap_nb_SOURCES octree.cc)
endif()

nanobind_add_module(coal_pywrap_nb NB_STATIC LTO NB_SUPPRESS_WARNINGS ${coal_wrap_nb_SOURCES} ${coal_pywrap_nb_HEADERS})
jrl_target_enforce_msvc_conformance(coal_pywrap_nb PRIVATE)
target_link_libraries(coal_pywrap_nb PRIVATE coal)
jrl_target_set_output_directory(coal_pywrap_nb OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/site-packages/coal)

jrl_python_generate_init_py(
    coal_pywrap_nb
    OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/site-packages/coal/__init__.py
)

if(GENERATE_PYTHON_STUBS)
  nanobind_add_stub(
    coal_pywrap_nb_stub
    MODULE coal.coal_pywrap_nb
    OUTPUT ${CMAKE_BINARY_DIR}/lib/site-packages/coal/coal_pywrap_nb.pyi
    PYTHON_PATH ${CMAKE_BINARY_DIR}/lib/site-packages
    DEPENDS coal_pywrap_nb
  )
endif()

jrl_python_compile_all(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

jrl_python_compute_install_dir(python_install_dir)
install(
  DIRECTORY ${CMAKE_BINARY_DIR}/lib/site-packages/coal
  DESTINATION ${python_install_dir}
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "*.pyc"
  PATTERN "*.pyi"
  PATTERN "*.typed"
  PATTERN "*.so"
  PATTERN "*.pyd"
)
