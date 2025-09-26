add_project_dependency(benchmark REQUIRED)
add_project_dependency(hwy REQUIRED)

function(add_coal_benchmark bench_name)
  set(sources ${ARGN})
  set(target_name ${PROJECT_NAME}-bench-${bench_name})
  add_executable(${target_name} ${sources})
  set_standard_output_directory(${target_name})
  target_link_libraries(${target_name} PRIVATE ${LIBRARY_NAME} Eigen3::Eigen
                                               benchmark::benchmark hwy::hwy)
  # This is mandatory to support highway auto include
  target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

add_library(${PROJECT_NAME}-bench-utils-icosahedron STATIC
            utils/icosahedron.cpp)
target_link_libraries(${PROJECT_NAME}-bench-utils-icosahedron
                      PUBLIC Eigen3::Eigen)

add_coal_benchmark(convex_support convex_support.cpp convex_support_highway.cpp)
target_link_libraries(${PROJECT_NAME}-bench-convex_support
                      PRIVATE ${PROJECT_NAME}-bench-utils-icosahedron)
