cmake_minimum_required(VERSION 2.8.3)
message("This is SlideFrictionControlPlugin")

if(NOT ONLYTEST)
	set(TARGET CnoidSlideFrictionControlPlugin)

	message("Build qpOASES")
	link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/bin)
	include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/include)

	# for openhrp3
	# using openhrp3's Link class
	pkg_check_modules(openhrp3 REQUIRED openhrp3.1)
	include_directories(${openhrp3_INCLUDE_DIRS})
	link_directories(${openhrp3_LIBRARY_DIRS})
	execute_process(
		COMMAND pkg-config --variable=link_shared_files openhrp3.1
		OUTPUT_VARIABLE openhrp3_link_shared_files
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
	string(REGEX REPLACE "-l" "" openhrp3_link_shared_files ${openhrp3_link_shared_files})# remove -l
	string(REGEX REPLACE " " ";" openhrp3_link_shared_files ${openhrp3_link_shared_files})# change string to list

	add_library(${TARGET} SHARED SlideFrictionControlPlugin SlideFrictionControl SlideFrictionControlBar)

	set(libs CnoidUtilPlugin qpOASES CnoidMultiContactStabilizerPlugin ${openhrp3_link_shared_files})
	target_link_libraries(${TARGET} ${libs})
endif(NOT ONLYTEST)

# include(test.cmake)

set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/choreonoid-${choreonoid_VER})

set(PLUGIN_NAME SlideFrictionControlPlugin)
set(headers
	SlideFrictionControlPlugin.h
	SlideFrictionControlBar.h
	SlideFrictionControl.h)
set(INCLUDE_DEST_DIR ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/${PLUGIN_NAME})
execute_process(
  COMMAND cmake -E make_directory ${INCLUDE_DEST_DIR}
  RESULT_VARIABLE _make_failed)
foreach(header ${headers})
	execute_process(
		COMMAND cmake -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${header} ${INCLUDE_DEST_DIR}
		RESULT_VARIABLE _copy_failed)
endforeach()

if(BUILD_PREVIEWCONTROL)

endif()
