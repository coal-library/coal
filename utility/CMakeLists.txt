add_library(coal-tests-resources INTERFACE)
target_compile_definitions(
  coal-tests-resources
  INTERFACE TEST_RESOURCES_DIR="${PROJECT_SOURCE_DIR}/test/fcl_resources"
)

add_library(coal-tests-utility STATIC utility.cpp)
target_link_libraries(coal-tests-utility PUBLIC coal)
target_include_directories(coal-tests-utility PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
