cmake_minimum_required(VERSION 3.5)
project(leo_bringup)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

generate_parameter_library(
  heading_controller_parameters
  src/heading_controller_parameters.yaml
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/heading_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  heading_controller_parameters
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  tf2::tf2
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN leo_bringup::HeadingController
  EXECUTABLE heading_controller
  EXECUTOR EventsExecutor
)

install(
  DIRECTORY
    camera_info
    camera_tuning_files
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    scripts/charging_monitor
    scripts/leo_system
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
