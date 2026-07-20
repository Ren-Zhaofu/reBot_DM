cmake_minimum_required(VERSION 3.22)
project(rebotarm_moveit_config)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_moveit_config test/test_moveit_config.py)
endif()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
