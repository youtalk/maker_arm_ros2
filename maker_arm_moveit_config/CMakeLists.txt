cmake_minimum_required(VERSION 3.8)
project(maker_arm_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/limits_from_profiles.py DESTINATION lib/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_joint_limits test/test_joint_limits.py)
  ament_add_pytest_test(test_srdf test/test_srdf.py)
endif()

ament_package()
