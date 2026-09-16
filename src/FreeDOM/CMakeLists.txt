cmake_minimum_required(VERSION 3.8)
project(freedom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slam_log REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
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
find_package(interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
# Map saving goes through point_lio's tile_map_writer.h, the single implementation
find_package(point_lio REQUIRED)

include_directories(
        SYSTEM
        include
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
)


add_executable(freedom_node
        src/freedom.cpp
        src/depth_image.cpp
        src/map.cpp
        src/scanmap.cpp
        src/raycast.cpp
        src/mrmap.cpp
        src/freenode.cpp
        src/visualization.cpp
        src/node.cpp
)

ament_target_dependencies(freedom_node
        rclcpp
        interface
        geometry_msgs
        cv_bridge
        PCL
        OpenCV
        pcl_conversions
        sensor_msgs
        tf2_ros
        visualization_msgs
        std_msgs
        tf2_eigen
        tf2_eigen_kdl
        point_lio
)
install(TARGETS
        freedom_node
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
        launch config
        DESTINATION share/${PROJECT_NAME}
)
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

target_link_libraries(freedom_node slam_log::slam_log)

ament_package()
