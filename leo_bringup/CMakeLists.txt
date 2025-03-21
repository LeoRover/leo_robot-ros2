cmake_minimum_required(VERSION 3.5)
project(leo_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    camera_info
    camera_tuning_files
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/leo_system
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
