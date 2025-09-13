cmake_minimum_required(VERSION 3.5)
project(leo_filters)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs)
find_package(generate_parameter_library REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

generate_parameter_library(
  imu_filter_parameters
  src/imu_filter_parameters.yaml
)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/imu_filter.cpp
  src/odom_filter.cpp
  src/complementary_filter.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${geometry_msgs_TARGETS}
  imu_filter_parameters
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  ${YAML_CPP_LIBRARIES}
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN leo_filters::OdomFilter
  EXECUTABLE odom_filter
  EXECUTOR MultiThreadedExecutor
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN leo_filters::ImuFilter
  EXECUTABLE imu_filter
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