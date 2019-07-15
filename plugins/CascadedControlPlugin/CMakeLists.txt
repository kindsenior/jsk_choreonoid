cmake_minimum_required(VERSION 2.8.3)
option(BUILD_CASCADEDCONTROL_PLUGIN "Build CascatedControl Plugin" ON)
set(TARGET CnoidCascadedControlPlugin)
message("TARGET: ${TARGET}")
if(BUILD_CASCADEDCONTROL_PLUGIN)

  if(NOT ONLYTEST)

    # for qpoases
	  include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/include)

	  # for openhrp3
	  # using openhrp3's Link class
	  pkg_check_modules(openhrp3 REQUIRED openhrp3.1)
	  include_directories(${openhrp3_INCLUDE_DIRS})
	  link_directories(${openhrp3_LIBRARY_DIRS})

	  add_library(${TARGET} SHARED CascadedControlPlugin CascadedControlBar)

	  set(libs CnoidUtilPlugin CnoidSlideFrictionControlPlugin CnoidMultiContactStabilizerPlugin)
	  target_link_libraries(${TARGET} ${libs})
  endif(NOT ONLYTEST)

  # include(test.cmake)

  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

else()
  message("Skip building ${TARGET}")
endif()
