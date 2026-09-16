cmake_minimum_required(VERSION 3.12)
project(lidar_imu_init)

# Force C++17 throughout; all C++14 settings removed
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

SET(CMAKE_BUILD_TYPE "Release")

# Compile options (C++17 only)
ADD_COMPILE_OPTIONS(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# Find Python3 and NumPy (replaces the original PythonLibs)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
if(Python3_NumPy_FOUND)
    message(STATUS "NumPy include dir: ${Python3_NumPy_INCLUDE_DIRS}")
else()
    message(WARNING "NumPy not found, using fallback path")
    set(Python3_NumPy_INCLUDE_DIRS "/usr/lib/python3/dist-packages/numpy/core/include")
endif()

# Find matplotlibcpp.h
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Ceres REQUIRED)
find_package(ament_index_cpp  REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Python executable path
set(Python3_EXECUTABLE "/usr/bin/python3")

if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}   # NumPy include path
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  "msg/States.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(li_init src/laserMapping.cpp
                  src/preprocess.cpp
                  include/LI_init/LI_init.cpp
                  include/ikd-Tree/ikd_Tree.cpp
)

target_include_directories(li_init PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIR}
  ${Python3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  )

ament_target_dependencies(li_init
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "geometry_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
  "pcl_conversions"
  "livox_ros_driver2"
  "ament_index_cpp"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(
  li_init
  ${EIGEN_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Python3_LIBRARIES}
  ${CERES_LIBRARIES}
  ${cpp_typesupport_target}
)

install(TARGETS li_init
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()