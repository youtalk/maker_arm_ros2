cmake_minimum_required(VERSION 3.8)
project(maker_arm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/release DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_files test/test_launch_files.py)
  ament_add_pytest_test(test_release_script test/test_release_script.py)
  # V1 on SimArm. maker-arm-cli must be on PATH. ctest runs the two one at a time, and both
  # use the default sim daemon socket.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_v1_simarm.py TIMEOUT 600)
  ament_add_pytest_test(test_v1_hold test/test_v1_hold.py TIMEOUT 600)
endif()

ament_package()
