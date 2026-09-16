cmake_minimum_required(VERSION 3.8)
project(slam_log)

find_package(ament_cmake REQUIRED)
find_package(glog REQUIRED)

add_library(slam_log src/log.cpp)
target_include_directories(slam_log PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(slam_log PUBLIC glog::glog)
target_compile_features(slam_log PUBLIC cxx_std_17)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS slam_log EXPORT export_slam_log
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_slam_log HAS_LIBRARY_TARGET)
ament_export_dependencies(glog)
ament_package()
