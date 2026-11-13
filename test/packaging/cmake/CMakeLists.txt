cmake_minimum_required(VERSION 3.22)
project(coal_test_packaging)

find_package(coal CONFIG REQUIRED)

add_executable(coal_test_packaging coal_test_packaging.cpp)
target_link_libraries(coal_test_packaging PRIVATE coal::coal coal::coal-hpp-fcl-compatibility)
