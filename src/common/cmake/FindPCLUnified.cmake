include(CMakeParseArguments)

# Unified PCL setup for this workspace.
# Usage:
#   nav_prepare_pcl()
#   nav_find_pcl(COMPONENTS common io filters)
#   nav_find_pcl()
function(nav_prepare_pcl)
  # Prefer the valid system PCL config path under /usr.
  set(PCL_CMAKE_DIR_HINT "/usr/lib/aarch64-linux-gnu/cmake/pcl")
  if(EXISTS "${PCL_CMAKE_DIR_HINT}/PCLConfig.cmake")
    set(PCL_DIR "${PCL_CMAKE_DIR_HINT}" CACHE PATH "Path to PCLConfig.cmake" FORCE)
  endif()

  # Force linker search preference to /lib/aarch64-linux-gnu.
  set(PCL_FORCE_LIB_DIR "/lib/aarch64-linux-gnu")
  if(EXISTS "${PCL_FORCE_LIB_DIR}")
    link_directories(${PCL_FORCE_LIB_DIR})
  endif()

  # Export variables to caller scope.
  set(PCL_DIR "${PCL_DIR}" PARENT_SCOPE)
  set(PCL_FORCE_LIB_DIR "${PCL_FORCE_LIB_DIR}" PARENT_SCOPE)
endfunction()

function(nav_find_pcl)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs COMPONENTS)
  cmake_parse_arguments(NAVPCL "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  nav_prepare_pcl()

  # Avoid re-importing PCL if it is already loaded transitively (e.g. via
  # pcl_conversions / pcl_ros), which can cause duplicated imported targets
  # like "pcl_io".
  if(NOT PCL_FOUND AND NOT TARGET pcl_common AND NOT TARGET PCL::common)
    if(NAVPCL_COMPONENTS)
      find_package(PCL REQUIRED COMPONENTS ${NAVPCL_COMPONENTS})
    else()
      find_package(PCL REQUIRED)
    endif()
  endif()

  if(PCL_FORCE_LIB_DIR AND EXISTS "${PCL_FORCE_LIB_DIR}")
    list(PREPEND PCL_LIBRARY_DIRS "${PCL_FORCE_LIB_DIR}")
    list(REMOVE_DUPLICATES PCL_LIBRARY_DIRS)
  endif()

  # Export variables to caller scope.
  set(PCL_DIR "${PCL_DIR}" PARENT_SCOPE)
  set(PCL_INCLUDE_DIRS "${PCL_INCLUDE_DIRS}" PARENT_SCOPE)
  set(PCL_LIBRARY_DIRS "${PCL_LIBRARY_DIRS}" PARENT_SCOPE)
  set(PCL_LIBRARIES "${PCL_LIBRARIES}" PARENT_SCOPE)
  set(PCL_DEFINITIONS "${PCL_DEFINITIONS}" PARENT_SCOPE)
endfunction()
