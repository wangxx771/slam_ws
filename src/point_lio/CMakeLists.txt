cmake_minimum_required(VERSION 3.8)
project(point_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

ADD_COMPILE_OPTIONS(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -Wno-reorder")


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processor number:  ${N}")

  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message(STATUS "core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message(STATUS "core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interface REQUIRED)

# Unify how PCL is found, so different packages cannot pick different PCL copies and clash on symbols.
# Provided by nav_common_cmake; falls back to a plain find_package(PCL) when absent.
find_package(nav_common_cmake QUIET)
if(COMMAND nav_prepare_pcl)
  nav_prepare_pcl()
else()
  message(STATUS "nav_common_cmake unavailable, falling back to find_package(PCL)")
  find_package(PCL REQUIRED)
  add_definitions(${PCL_DEFINITIONS})
endif()

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(slam_log REQUIRED)
find_package(OpenMP QUIET)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../small_gicp_relocalization/include  # scan_context.hpp (single header, shared by both ends)
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

# Declare a ROS2 executable
add_executable(pointlio_mapping
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp
  src/loop_closure.cpp
  src/gnss_processing.cpp
)

ament_target_dependencies(pointlio_mapping
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  interface
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
  rosbag2_cpp
)

target_link_libraries(pointlio_mapping
  ${PCL_LIBRARIES}
  slam_log::slam_log
)

# Install the executable
# Tile merge tool. C++ rather than a script because it processes millions to tens of millions of points.
add_executable(merge_tiles src/merge_tiles.cpp)
ament_target_dependencies(merge_tiles rclcpp pcl_conversions)
target_link_libraries(merge_tiles ${PCL_LIBRARIES})

# Offline evaluation tool for the dynamic-point criterion (labelled datasets only, not part of runtime)

install(TARGETS
  pointlio_mapping
  merge_tiles
  DESTINATION lib/${PROJECT_NAME}
)

# Export headers: tile_map_writer.h is the single implementation of map saving and
# FreeDOM uses it too, so the two packages cannot each write their own format.
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
  Eigen3
  PCL
)

ament_package()