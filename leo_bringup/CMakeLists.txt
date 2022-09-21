cmake_minimum_required(VERSION 3.5)
project(leo_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    camera_info
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/leo_system
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
