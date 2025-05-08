cmake_minimum_required(VERSION 3.5)
project(leo_fw)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(leo_msgs REQUIRED)

include_directories(
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(leo_fw SHARED
  src/firmware_message_converter.cpp
)
target_link_libraries(leo_fw
  ${leo_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
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
    scripts/firmware_parameter_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
