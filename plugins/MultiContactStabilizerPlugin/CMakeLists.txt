cmake_minimum_required(VERSION 2.8.3)
option(BUILD_MULTICONTACTSTABILIZER_PLUGIN "Build MultiContactStabilizer Plugin" ON)
set(TARGET CnoidMultiContactStabilizerPlugin)
message("TARGET: ${TARGET}")
if(BUILD_MULTICONTACTSTABILIZER_PLUGIN)

  if(NOT ONLYTEST)

	  message("Build qpOASES")
	  add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0 ${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0)
	  link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/bin)
	  include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/include)
	  # add_definitions(-DUSE_QPOASES)

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

	  add_library(${TARGET} SHARED MultiContactStabilizerPlugin MultiContactStabilizer ModelPredictiveController ContactConstraint QP MultiContactStabilizerBar)

	  set(libs CnoidUtilPlugin qpOASES ${openhrp3_link_shared_files})
	  target_link_libraries(${TARGET} ${libs})
  endif(NOT ONLYTEST)

  include(test.cmake)

  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

  set(PLUGIN_NAME MultiContactStabilizerPlugin)
  set(headers
    MultiContactStabilizerPlugin.h
    MultiContactStabilizerBar.h
    ModelPredictiveController.h
    MultiContactStabilizer.h
    ContactConstraint.h
    QP.h)
  set(INCLUDE_DEST_DIR ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/${PLUGIN_NAME})
  execute_process(
    COMMAND cmake -E make_directory ${INCLUDE_DEST_DIR}
    RESULT_VARIABLE _make_failed)
  foreach(header ${headers})
	  execute_process(
		  COMMAND cmake -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${header} ${INCLUDE_DEST_DIR}
		  RESULT_VARIABLE _copy_failed)
  endforeach()

else()
  message("Skip building ${TARGET}")
endif()
