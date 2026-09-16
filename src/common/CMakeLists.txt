cmake_minimum_required(VERSION 3.8)
project(nav_common_cmake)
find_package(ament_cmake REQUIRED)

# Install cmake/ into share and register it as an extension, so downstream can
# call nav_prepare_pcl() / nav_find_pcl() straight after
# find_package(nav_common_cmake) without knowing where it lives.
install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
ament_package(CONFIG_EXTRAS "cmake/FindPCLUnified.cmake")
