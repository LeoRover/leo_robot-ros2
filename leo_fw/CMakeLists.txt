cmake_minimum_required(VERSION 3.5)
project(leo_fw)

find_package(ament_cmake REQUIRED)
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

install(
  TARGETS
    firmware_message_converter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
