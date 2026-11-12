if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  add_subdirectory(cpp)
endif()

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()
