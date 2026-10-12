# Set project name and version
set(PROJECT_NAME vikit_ros)
project(${PROJECT_NAME})
cmake_minimum_required(VERSION 3.5)

# Set build type
set(CMAKE_BUILD_TYPE Release)  # Release, RelWithDebInfo
set(CMAKE_VERBOSE_MAKEFILE OFF)

# Set build flags
set(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")  # ROS 2 uses C++14 by default
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()


# Find ament_cmake and required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vikit_common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}      
  ${tf2_ros_INCLUDE_DIRS}  
  ${visualization_msgs_INCLUDE_DIRS}       

)

# Set source files
list(APPEND SOURCEFILES
  src/output_helper.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})
target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${vikit_common_LIBRARIES}
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  visualization_msgs
  tf2
  tf2_ros
  vikit_common
  OpenCV
  Eigen3
  Sophus
)

# Declare ament package
ament_package()