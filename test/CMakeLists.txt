if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  add_subdirectory(cpp)
endif()

if(COAL_BUILD_BENCHMARKS AND NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  add_subdirectory(benchmark)
endif()

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()
