add_library(coal-tests-resources INTERFACE)
target_compile_definitions(
  coal-tests-resources
  INTERFACE TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fcl_resources"
)

if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  add_subdirectory(utility)
  add_subdirectory(cpp)
endif()

if(COAL_BUILD_BENCHMARKS AND NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  add_subdirectory(benchmark)
endif()

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()
