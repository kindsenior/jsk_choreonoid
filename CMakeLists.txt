cmake_minimum_required(VERSION 3.1.0)
project(jsk_choreonoid)

cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0057 NEW)

## include cmake util function
include(${PROJECT_SOURCE_DIR}/cmake/util.cmake)

include(FindPkgConfig)
include(CheckIncludeFiles)

find_package(catkin REQUIRED cmake_modules)
find_package(choreonoid)
if(NOT ${CHOREONOID_FOUND})
  message("choreonoid not found")
  find_package(PkgConfig)
  pkg_check_modules(choreonoid choreonoid REQUIRED)
endif()
# apply choreonoid cmake settings
add_definitions(${CHOREONOID_DEFINITIONS})
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

# set path for installing jsk_choreonoid libraries in choreonoid
set(choreonoid_PLUGINDIR ${CHOREONOID_PLUGIN_DIR})
set(choreonoid_PYTHON_DIR ${CHOREONOID_ROOT_DIR}/${CHOREONOID_PYTHON_SUBDIR}/cnoid)

message("********************choreonoid pkg infomation********************")
message("CHOREONOID_FOUND: ${CHOREONOID_FOUND}")
message("CHOREONOID_VERSION_STRING: ${CHOREONOID_VERSION_STRING}")
message("CHOREONOID_VERSION_MAJOR: ${CHOREONOID_VERSION_MAJOR}")
message("CHOREONOID_VERSION_MINOR: ${CHOREONOID_VERSION_MINOR}")
message("CHOREONOID_VERSION_PATCH: ${CHOREONOID_VERSION_PATCH}")
message("CHOREONOID_INCLUDE_DIRS: ${CHOREONOID_INCLUDE_DIRS}")
message("CHOREONOID_LIBRARY_DIRS: ${CHOREONOID_LIBRARY_DIRS}")
message("CHOREONOID_LIBRARIES: ${CHOREONOID_LIBRARIES}")
message("CHOREONOID_PLUGIN_DIR: ${CHOREONOID_PLUGIN_DIR}")
message("CHOREONOID_ADDITIONAL_LIBRARIES: ${CHOREONOID_ADDITIONAL_LIBRARIES}")

# for python module
catkin_python_setup()

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME})
catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME})

# -I
message("CATKIN_PACKAGE_INCLUDE_DESTINATION: ${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# for -std=c++11
# set(CMAKE_CXX_STANDARD 11)# cannot?
add_compile_options(-std=c++11)

message("Build 3rdparty")
file(GLOB _3rdparty_dirs "${PROJECT_SOURCE_DIR}/3rdparty/*")
foreach(_3rdparty_dir ${_3rdparty_dirs})
  message("Build: ${_3rdparty_dir}")
  add_subdirectory(${_3rdparty_dir})
endforeach()

message("Build plugins")
# file(GLOB plugins RELATIVE  ${PROJECT_SOURCE_DIR}/plugins "${PROJECT_SOURCE_DIR}/plugins/*Plugin")
file(GLOB _plugin_dirs "${PROJECT_SOURCE_DIR}/plugins/*Plugin")
foreach(_plugin_dir ${_plugin_dirs})
  message("Build: ${_plugin_dir}")
  add_subdirectory(${_plugin_dir})
endforeach()
