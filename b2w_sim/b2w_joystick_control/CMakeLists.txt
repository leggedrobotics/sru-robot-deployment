cmake_minimum_required(VERSION 3.8)
project(b2w_joystick_control)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Joystick teleop executable
add_executable(joystick_teleop_node
  src/joystick_teleop_node.cpp
  src/joystick_teleop.cpp
)

ament_target_dependencies(joystick_teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Install executables
install(TARGETS
  joystick_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
