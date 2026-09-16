# Added for the ROS 2 workspace build; not present upstream.
#
# Upstream ships no CMake package config, so a dependent project can only guess
# at /usr/local. Exporting one has two effects: colcon sees a config file and
# therefore puts this prefix on CMAKE_PREFIX_PATH for dependents, and the driver
# can find_package() the library built from this source tree instead of whatever
# older copy happens to be installed system-wide.

get_filename_component(_livox_sdk2_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

set(Livox-SDK2_INCLUDE_DIRS "${_livox_sdk2_prefix}/include")

find_library(Livox-SDK2_SHARED_LIBRARY
  NAMES livox_lidar_sdk_shared
  PATHS "${_livox_sdk2_prefix}/lib"
  NO_DEFAULT_PATH)
find_library(Livox-SDK2_STATIC_LIBRARY
  NAMES livox_lidar_sdk_static
  PATHS "${_livox_sdk2_prefix}/lib"
  NO_DEFAULT_PATH)

if(NOT TARGET livox_sdk2::shared)
  add_library(livox_sdk2::shared UNKNOWN IMPORTED)
  set_target_properties(livox_sdk2::shared PROPERTIES
    IMPORTED_LOCATION "${Livox-SDK2_SHARED_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${Livox-SDK2_INCLUDE_DIRS}")
endif()

if(NOT TARGET livox_sdk2::static)
  add_library(livox_sdk2::static UNKNOWN IMPORTED)
  set_target_properties(livox_sdk2::static PROPERTIES
    IMPORTED_LOCATION "${Livox-SDK2_STATIC_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${Livox-SDK2_INCLUDE_DIRS}")
endif()

set(Livox-SDK2_LIBRARIES livox_sdk2::shared)
set(Livox-SDK2_FOUND TRUE)
