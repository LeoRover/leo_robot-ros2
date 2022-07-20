cmake_minimum_required(VERSION 3.5)
project(leo_fw)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(leo_msgs REQUIRED)

add_library(leo_fw SHARED
  src/firmware_message_converter.cpp
)
target_link_libraries(leo_fw
  rclcpp::rclcpp
  rclcpp_components::component
)
ament_target_dependencies(leo_fw
  sensor_msgs
  leo_msgs
  nav_msgs
)
rclcpp_components_register_node(leo_fw
  PLUGIN "leo_fw::FirmwareMessageConverter"
  EXECUTABLE "firmware_message_converter"
)

ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS leo_fw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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
