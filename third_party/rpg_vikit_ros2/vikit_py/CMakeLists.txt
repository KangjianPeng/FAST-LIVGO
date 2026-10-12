cmake_minimum_required(VERSION 3.5)
project(vikit_py)

find_package(ament_cmake REQUIRED)

# Find Python dependencies
find_package(rclpy REQUIRED)

# Uncomment this if the package has a setup.py. This macro ensures
# modules and global scripts declared therein get installed
ament_python_install_package(${PROJECT_NAME})

# Include directories
include_directories(
  ${rclpy_INCLUDE_DIRS}
)

# Declare ament package
ament_package()