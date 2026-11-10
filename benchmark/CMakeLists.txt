function(coal_add_benchmark name)
  set(target_name coal-benchmark-${name})
  add_executable(${target_name} ${name}.cpp)
  target_link_libraries(
    ${target_name}
    PRIVATE coal-tests-utility coal-tests-resources coal benchmark::benchmark
  )
  jrl_target_enforce_msvc_conformance(${target_name} PRIVATE)
endfunction()

coal_add_benchmark(bvh_collision_distance)
