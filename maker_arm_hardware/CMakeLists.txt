cmake_minimum_required(VERSION 3.16)
project(maker_arm_hardware)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(maker_arm_hardware SHARED src/daemon_link.cpp src/maker_arm_system.cpp)
target_include_directories(maker_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(maker_arm_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface maker_arm_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS maker_arm_hardware EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wire test/test_wire.cpp)
  target_include_directories(test_wire PRIVATE include)
  # Each test starts a sim daemon, and a spawn may wait up to 10 s.
  ament_add_gtest(test_daemon_link test/test_daemon_link.cpp TIMEOUT 120)
  target_link_libraries(test_daemon_link maker_arm_hardware)
  target_compile_definitions(test_daemon_link PRIVATE
    FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
