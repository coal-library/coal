function(coal_add_benchmark name)
  add_executable(coal-benchmark-${name} ${name}.cpp)
  target_link_libraries(coal-benchmark-${name} PRIVATE coal-tests-utility coal-tests-resources)
endfunction()

coal_add_benchmark(obb)
coal_add_benchmark(profiling)
coal_add_benchmark(benchmark)
