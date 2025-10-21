add_project_dependency(benchmark REQUIRED)
add_project_dependency(hwy REQUIRED)

# Add icosahedron building library
add_library(coal-bench-utils-icosahedron STATIC utils/icosahedron.cpp)
target_link_libraries(coal-bench-utils-icosahedron PUBLIC Eigen3::Eigen)

# Add coal-bench-convex-support benchmark
add_executable(coal-bench-convex-support convex_support.cpp
                                         convex_support_highway.cpp)
set_standard_output_directory(coal-bench-convex-support)
target_link_libraries(
  coal-bench-convex-support
  PRIVATE ${LIBRARY_NAME} Eigen3::Eigen Boost::boost benchmark::benchmark
          hwy::hwy coal-bench-utils-icosahedron)
# This is mandatory to support highway auto include
target_include_directories(coal-bench-convex-support
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
