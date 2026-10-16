cmake_minimum_required(VERSION 3.16)
project(maker_arm_hardware)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wire test/test_wire.cpp)
  target_include_directories(test_wire PRIVATE include)
endif()

ament_package()
