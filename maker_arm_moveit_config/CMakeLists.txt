cmake_minimum_required(VERSION 3.8)
project(maker_arm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)

add_library(maker_arm_planning_adapters SHARED src/default_planner_id.cpp)
ament_target_dependencies(maker_arm_planning_adapters moveit_core pluginlib)
pluginlib_export_plugin_description_file(moveit_core maker_arm_planning_adapters.xml)
install(TARGETS maker_arm_planning_adapters LIBRARY DESTINATION lib)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/limits_from_profiles.py DESTINATION lib/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_joint_limits test/test_joint_limits.py)
  ament_add_pytest_test(test_srdf test/test_srdf.py)
endif()

ament_package()
