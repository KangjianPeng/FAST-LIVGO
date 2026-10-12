cmake_minimum_required(VERSION 3.8)
project(fast_livgo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 64-bit ARM optimizations (e.g., RK3588 and Jetson Orin NX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    # 32-bit ARM optimizations with NEON support
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_definitions(-DARM_ARCH)
else()
  # Keep Eigen's alignment ABI compatible with the system PCL binaries.
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops") #-flto
  message(STATUS "Using ABI-compatible x86 optimizations: -O3 -funroll-loops")
  add_definitions(-DX86_ARCH)
endif()

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# find dependencies
if(EXISTS "/usr/share/cmake/geographiclib")
  list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
endif()
find_package(GeographicLib REQUIRED)
find_package(glog CONFIG REQUIRED)
find_package(GTSAM REQUIRED)
find_package(gnss_comm REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  # add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Include directories
set(VIKIT_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rpg_vikit_ros2/vikit_common")
if(NOT EXISTS "${VIKIT_COMMON_DIR}/include/vikit/abstract_camera.h")
  message(FATAL_ERROR
    "Vikit submodule is missing. Run: git submodule update --init --recursive")
endif()

set(SOPHUS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Sophus")
if(NOT EXISTS "${SOPHUS_DIR}/sophus/se3.h")
  message(FATAL_ERROR
    "Sophus submodule is missing. Run: git submodule update --init --recursive")
endif()

include_directories(
  include
  "${VIKIT_COMMON_DIR}/include"
  "${SOPHUS_DIR}"
  ${GeographicLib_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# Add libraries
add_library(fast_livgo_sophus STATIC
  "${SOPHUS_DIR}/sophus/so3.cpp"
  "${SOPHUS_DIR}/sophus/se3.cpp"
)
set_target_properties(fast_livgo_sophus PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(fast_livgo_sophus
  PUBLIC
    "${SOPHUS_DIR}"
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(fast_livgo_vikit_common STATIC
  "${VIKIT_COMMON_DIR}/src/atan_camera.cpp"
  "${VIKIT_COMMON_DIR}/src/omni_camera.cpp"
  "${VIKIT_COMMON_DIR}/src/math_utils.cpp"
  "${VIKIT_COMMON_DIR}/src/vision.cpp"
  "${VIKIT_COMMON_DIR}/src/performance_monitor.cpp"
  "${VIKIT_COMMON_DIR}/src/robust_cost.cpp"
  "${VIKIT_COMMON_DIR}/src/user_input_thread.cpp"
  "${VIKIT_COMMON_DIR}/src/pinhole_camera.cpp"
  "${VIKIT_COMMON_DIR}/src/equidistant_camera.cpp"
  "${VIKIT_COMMON_DIR}/src/polynomial_camera.cpp"
  "${VIKIT_COMMON_DIR}/src/homography.cpp"
  "${VIKIT_COMMON_DIR}/src/img_align.cpp"
)
set_target_properties(fast_livgo_vikit_common PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(fast_livgo_vikit_common
  PUBLIC
    "${VIKIT_COMMON_DIR}/include"
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    "${SOPHUS_DIR}"
)
target_link_libraries(fast_livgo_vikit_common ${OpenCV_LIBS} fast_livgo_sophus)
ament_target_dependencies(fast_livgo_vikit_common rclcpp)

ament_auto_add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
ament_auto_add_library(lio src/voxel_map.cpp)
ament_auto_add_library(pre src/preprocess.cpp)
ament_auto_add_library(imu_proc src/IMU_Processing.cpp)
ament_auto_add_library(laser_mapping src/LIVMapper.cpp)
ament_auto_add_library(optimization src/optimization.cpp)
target_link_libraries(vio fast_livgo_vikit_common)
target_link_libraries(laser_mapping ${GeographicLib_LIBRARIES})
target_link_libraries(optimization gtsam ${GeographicLib_LIBRARIES})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(pre "${cpp_typesupport_target}")
target_link_libraries(laser_mapping "${cpp_typesupport_target}")

# Add the main executable
ament_auto_add_executable(fastlivgo_mapping src/main.cpp)
target_link_libraries(fastlivgo_mapping
  vio lio pre imu_proc laser_mapping optimization fast_livgo_sophus "${cpp_typesupport_target}")

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

install(DIRECTORY
  launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
