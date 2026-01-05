cmake_minimum_required(VERSION 3.5)
project(b2w_gazebo_ros2)

# Set C++ standard (optional if no C++ code)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)  # For Gazebo integration

# Export dependencies (BEFORE ament_package!)
ament_export_dependencies(ros_gz_sim)

# Declare ament package (MUST BE LAST!)
ament_package()

# Install launch/config/world files
install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)