cmake_minimum_required(VERSION 2.8.3)
project(jsk_choreonoid)

# find_package(catkin REQUIRED COMPONENTS mk choreonoid)
# find_package(catkin REQUIRED COMPONENTS mk)

find_package(catkin REQUIRED)
find_package(choreonoid)
if(NOT ${choreonoid_FOUND})
	find_package(PkgConfig)
	pkg_check_modules(choreonoid choreonoid REQUIRED)
endif(NOT ${choreonoid_FOUND})

message("PKG_CONFIG_PATH: $ENV{PKG_CONFIG_PATH}")
# pkg_search_module(choreonoid choreonoid REQUIRED)
message("choreonoid_FOUND: ${choreonoid_FOUND}")
message("choreonoid_PREFIX: ${choreonoid_PREFIX}")
message("choreonoid_INCLUDEDIR: ${choreonoid_INCLUDEDIR}")
message("choreonoid_LIBDIR: ${choreonoid_LIBDIR}")
message("choreonoid_SOURCE_DIR: ${choreonoid_SOURCE_DIR}")
message("choreonoid_INCLUDE_DIRS: ${choreonoid_INCLUDE_DIRS}")
message("choreonoid_LIBRARIES: ${choreonoid_LIBRARIES}")
message("choreonoid_CFLAGS: ${choreonoid_CFLAGS}")
message("choreonoid_LIBRARY_DIRS: ${choreonoid_LIBRARY_DIRS}")
message("choreonoid_LDFLAGS :${choreonoid_LDFLAGS}")
message("choreonoid_LDFLAGS_OTHER :${choreonoid_LDFLAGS_OTHER}")
message("choreonoid_VERSION: ${choreonoid_VERSION}")
# message("choreonoid_plugindir: ${choreonoid_plugindir}")
string(REGEX REPLACE ".0" "" choreonoid_VER ${choreonoid_VERSION})
message("choreonoid_VER: ${choreonoid_VER}")

pkg_check_modules(choreonoid-body-plugin choreonoid-body-plugin REQUIRED)
message("choreonoid-body-plugin_SOURCE_DIR: ${choreonoid-body-plugin_SOURCE_DIR}")
message("choreonoid-body-plugin_INCLUDE_DIRS: ${choreonoid-body-plugin_INCLUDE_DIRS}")
message("choreonoid-body-plugin_LIBRARIES: ${choreonoid-body-plugin_LIBRARIES}")
message("choreonoid-body-plugin_CFLAGS: ${choreonoid-body-plugin_CFLAGS}")
message("choreonoid-body-plugin_LIBRARY_DIRS: ${choreonoid-body-plugin_LIBRARY_DIRS}")

execute_process(COMMAND pkg-config --variable=plugindir choreonoid
  OUTPUT_VARIABLE choreonoid_PLUGINDIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

include_directories(${choreonoid_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/plugins)
link_directories(${choreonoid_LIBRARY_DIRS})
set(choreonoid_ADDITIONAL_LIBRARIES CnoidPoseSeqPlugin)

# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib/choreonoid-${choreonoid_VER})

if ( CLEAN )
	message("clean")
	execute_process(
		COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}
		make clean -f Makefile.plugins
		)
	return()
endif( CLEAN )


message("Build by cmake")
# file(GLOB plugins RELATIVE  ${PROJECT_SOURCE_DIR}/plugins "${PROJECT_SOURCE_DIR}/plugins/*Plugin")
file(GLOB _plugin_dirs "${PROJECT_SOURCE_DIR}/plugins/*Plugin")
# message("Found Plugins: ${plugins}")	
foreach(_plugin_dir ${_plugin_dirs})

	message("Build: ${_plugin_dir}")
	add_subdirectory(${_plugin_dir})

endforeach()

# install(FILES jsk_choreonoid.pc
#   DESTINATION lib/pkgconfig)
execute_process(
	COMMAND cmake -E copy ${CMAKE_CURRENT_SOURCE_DIR}/jsk_choreonoid.pc ${CATKIN_DEVEL_PREFIX}/lib/pkgconfig
	RESULT_VARIABLE _copy_failed)

# execute_process(
# 	  COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}
# 		make -f Makefile.plugins
#     # INSTALL_DIR=${CATKIN_DEVEL_PREFIX} # CATKIN_DEVEL_PREFIX=devel (for making choreonoid(binary) in devel/bin)
#     RESULT_VARIABLE _make_failed)
# if (_make_failed)
#   message(FATAL_ERROR "Build of jsk_choreonoid plugins failed")
# endif(_make_failed)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
