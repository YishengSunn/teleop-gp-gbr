cmake_minimum_required(VERSION 3.8)
project(geo_gp_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(franka_example_controllers REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geo_gp_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/comless/motion_generator.cpp
  src/subscriber/cartesian_impedance_controller.cpp
  src/subscriber/hybrid_position_force_controller.cpp
  src/subscriber/joint_impedance_controller.cpp
  src/subscriber/gravity_compensation_with_joint_torque_feedback_controller.cpp
  src/subscriber/gravity_compensation_with_cartesian_force_feedback_controller.cpp
  src/tdpa/cartesian.cpp
  src/tdpa/joint.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  franka_semantic_components
  franka_example_controllers
  franka_msgs
  sensor_msgs
  std_msgs
  realtime_tools
  geo_gp_interfaces
)

target_link_libraries(
  ${PROJECT_NAME}
  geo_gp_interfaces::geo_gp_interfaces__rosidl_typesupport_cpp
)

add_executable(tdpa_delay_relay src/tdpa/tdpa_delay_relay.cpp)
ament_target_dependencies(
  tdpa_delay_relay
  rclcpp
  geo_gp_interfaces
)

pluginlib_export_plugin_description_file(
  controller_interface geo_gp_controllers.xml
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS tdpa_delay_relay
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  franka_semantic_components
  franka_example_controllers
  franka_msgs
  sensor_msgs
  std_msgs
  realtime_tools
  geo_gp_interfaces
)

ament_package()
