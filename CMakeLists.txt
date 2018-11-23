cmake_minimum_required(VERSION 3.1.0)
project(jsk_choreonoid)

## include cmake util function
include(${PROJECT_SOURCE_DIR}/cmake/util.cmake)

message("********************find pkg********************")
message("PKG_CONFIG_PATH: $ENV{PKG_CONFIG_PATH}")
# find_package(catkin REQUIRED COMPONENTS mk choreonoid)
# find_package(catkin REQUIRED COMPONENTS mk)
include(FindPkgConfig)
include(CheckIncludeFiles)

find_package(catkin REQUIRED cmake_modules)
find_package(choreonoid)
if(NOT ${choreonoid_FOUND})
	find_package(PkgConfig)
	pkg_check_modules(choreonoid choreonoid REQUIRED)
endif(NOT ${choreonoid_FOUND})

pkg_check_modules(choreonoid-body-plugin choreonoid-body-plugin REQUIRED)

# for python wrapper
## python package
find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})
## boost package
find_package(Boost 1.47.0 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

string(REGEX REPLACE ".0" "" choreonoid_VER ${choreonoid_VERSION})
execute_process(COMMAND pkg-config --variable=plugindir choreonoid
  OUTPUT_VARIABLE choreonoid_PLUGINDIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME})
catkin_package(
	INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}
	DEPENDS Eigen)

message("********************choreonoid pkg infomation********************")
# pkg_search_module(choreonoid choreonoid REQUIRED)
# message("choreonoid_FOUND: ${choreonoid_FOUND}")
# message("choreonoid_PREFIX: ${choreonoid_PREFIX}")
# message("choreonoid_INCLUDEDIR: ${choreonoid_INCLUDEDIR}")
# message("choreonoid_LIBDIR: ${choreonoid_LIBDIR}")
# message("choreonoid_SOURCE_DIR: ${choreonoid_SOURCE_DIR}")
message("choreonoid_CFLAGS: ${choreonoid_CFLAGS}")
message("choreonoid_LDFLAGS :${choreonoid_LDFLAGS}")
message("choreonoid_INCLUDE_DIRS: ${choreonoid_INCLUDE_DIRS}")
message("choreonoid_LIBRARY_DIRS: ${choreonoid_LIBRARY_DIRS}")
message("choreonoid_LIBRARIES: ${choreonoid_LIBRARIES}")
# message("choreonoid_LDFLAGS_OTHER :${choreonoid_LDFLAGS_OTHER}")
message("choreonoid_VERSION: ${choreonoid_VERSION}")
# message("choreonoid_plugindir: ${choreonoid_plugindir}")
message("choreonoid_VER: ${choreonoid_VER}")

message("********************choreonoid-body-plugin infomation********************")
# message("choreonoid-body-plugin_SOURCE_DIR: ${choreonoid-body-plugin_SOURCE_DIR}")
message("choreonoid-body-plugin_CFLAGS: ${choreonoid-body-plugin_CFLAGS}")
message("choreonoid-body-plugin_LDFLAGS :${choreonoid-body-plugin_LDFLAGS}")
message("choreonoid-body-plugin_INCLUDE_DIRS: ${choreonoid-body-plugin_INCLUDE_DIRS}")
message("choreonoid-body-plugin_LIBRARY_DIRS: ${choreonoid-body-plugin_LIBRARY_DIRS}")
message("choreonoid-body-plugin_LIBRARIES: ${choreonoid-body-plugin_LIBRARIES}")
message("EIGEN_INCLUDE_DIRS: ${EIGEN_INCLUDE_DIRS}")
include_directories(${Eigen_INCLUDE_DIRS})

# -I
include_directories(${choreonoid_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/plugins)
# -L
link_directories(${choreonoid_LIBRARY_DIRS})
set(choreonoid_ADDITIONAL_LIBRARIES CnoidPoseSeqPlugin)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib/choreonoid-${choreonoid_VER})

# for -std=c++11
# set(CMAKE_CXX_STANDARD 11)# できない?
add_compile_options(-std=c++11)

option(CLEAN "Clean" OFF)
if(CLEAN)
	message("clean")
	execute_process(
		COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}
		make clean -f Makefile.plugins
		)
	return()
endif(CLEAN)

message("Build 3rdparty")
file(GLOB _3rdparty_dirs "${PROJECT_SOURCE_DIR}/3rdparty/*")
foreach(_3rdparty_dir ${_3rdparty_dirs})
	message("Build: ${_3rdparty_dir}")
	add_subdirectory(${_3rdparty_dir})
endforeach()

message("Build plugins")
# file(GLOB plugins RELATIVE  ${PROJECT_SOURCE_DIR}/plugins "${PROJECT_SOURCE_DIR}/plugins/*Plugin")
file(GLOB _plugin_dirs "${PROJECT_SOURCE_DIR}/plugins/*Plugin")
# message("Found Plugins: ${plugins}")	
foreach(_plugin_dir ${_plugin_dirs})
	message("Build: ${_plugin_dir}")
	add_subdirectory(${_plugin_dir})
endforeach()


# for python module
catkin_python_setup()
