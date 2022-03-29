cmake_minimum_required(VERSION 3.5)
project(leo_fw)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(leo_msgs REQUIRED)

add_executable(firmware_message_converter
  src/firmware_message_converter.cpp
)
target_link_libraries(firmware_message_converter
  rclcpp::rclcpp
)
ament_target_dependencies(firmware_message_converter
  sensor_msgs
  leo_msgs
  nav_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS
    firmware_message_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/flash
    scripts/update
    scripts/test_hw
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
