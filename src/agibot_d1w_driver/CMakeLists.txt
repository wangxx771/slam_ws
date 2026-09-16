cmake_minimum_required(VERSION 3.8)
project(agibot_d1w_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# ROS2 dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(agibot_d1w_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)


# ============================================================
# AgiBot SDK
# nav/src/agibot_d1w_driver -> nav/agibot
# ============================================================

set(
  AGIBOT_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../agibot"
  CACHE PATH
  "Root directory of the AgiBot SDK"
)

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" AGIBOT_PROCESSOR)

if(AGIBOT_PROCESSOR MATCHES "aarch64|arm64")
  set(AGIBOT_ARCH "aarch64")
elseif(AGIBOT_PROCESSOR MATCHES "x86_64|amd64")
  set(AGIBOT_ARCH "x86_64")
else()
  message(
    FATAL_ERROR
    "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}"
  )
endif()


# ============================================================
# AgiBot SDK include
# ============================================================

find_path(
  AGIBOT_SDK_INCLUDE_DIR
  NAMES zsl-1w/highlevel.h
  PATHS
    "${AGIBOT_SDK_ROOT}/include"
  NO_DEFAULT_PATH
  REQUIRED
)


# ============================================================
# AgiBot SDK library
# ============================================================

find_library(
  AGIBOT_D1W_SDK_LIBRARY
  NAMES
    "mc_sdk_zsl_1w_${AGIBOT_ARCH}"
    "mc_sdk_zsl_1w"
  PATHS
    "${AGIBOT_SDK_ROOT}/lib/zsl-1w/${AGIBOT_ARCH}"
    "${AGIBOT_SDK_ROOT}/lib/zsl-1w"
    "${AGIBOT_SDK_ROOT}/lib/${AGIBOT_ARCH}"
  NO_DEFAULT_PATH
  REQUIRED
)

message(STATUS "AgiBot SDK root: ${AGIBOT_SDK_ROOT}")
message(STATUS "AgiBot SDK include: ${AGIBOT_SDK_INCLUDE_DIR}")
message(STATUS "AgiBot ZSL-1w library: ${AGIBOT_D1W_SDK_LIBRARY}")


# ============================================================
# D1W Driver Node
# ============================================================

add_executable(
  agibot_d1w_driver_node
  src/agibot_d1w_driver.cpp
)

target_include_directories(
  agibot_d1w_driver_node
  PRIVATE
    "${AGIBOT_SDK_INCLUDE_DIR}"
)

target_link_libraries(
  agibot_d1w_driver_node
  "${AGIBOT_D1W_SDK_LIBRARY}"
)

ament_target_dependencies(
  agibot_d1w_driver_node
  agibot_d1w_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)


# ============================================================
# Keyboard Velocity Command Node
# ============================================================

add_executable(
  keyboard_vel_cmd
  src/keyboard_vel_cmd.cpp
)

ament_target_dependencies(
  keyboard_vel_cmd
  rclcpp
  geometry_msgs
)


# ============================================================
# Runtime library path
# ============================================================

get_filename_component(
  AGIBOT_D1W_SDK_LIBRARY_DIR
  "${AGIBOT_D1W_SDK_LIBRARY}"
  DIRECTORY
)

set_target_properties(
  agibot_d1w_driver_node
  PROPERTIES
    BUILD_RPATH "${AGIBOT_D1W_SDK_LIBRARY_DIR}"
    INSTALL_RPATH "$ORIGIN"
    INSTALL_RPATH_USE_LINK_PATH FALSE
)


# ============================================================
# Install executable nodes
# ============================================================

install(
  TARGETS
    agibot_d1w_driver_node
    keyboard_vel_cmd
  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================
# Install AgiBot SDK shared library
# ============================================================

install(
  FILES
    "${AGIBOT_D1W_SDK_LIBRARY}"
  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================
# Install config / launch
# ============================================================

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()