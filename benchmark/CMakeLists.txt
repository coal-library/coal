add_project_dependency(benchmark REQUIRED)

function(add_coal_benchmark bench_name source)
  set(target_name ${PROJECT_NAME}-bench-${bench_name})
  add_executable(${target_name} ${source})
  set_standard_output_directory(${target_name})
  target_link_libraries(${target_name} PRIVATE ${LIBRARY_NAME} Eigen3::Eigen
                                               benchmark::benchmark)
endfunction()

add_coal_benchmark(convex_support convex_support.cpp)
