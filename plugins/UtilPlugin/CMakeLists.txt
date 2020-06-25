option(BUILD_UTIL_PLUGIN "Build Util Plugin" ON)
set(PLUGIN_NAME UtilPlugin)
set(TARGET Cnoid${PLUGIN_NAME})
message("TARGET: ${TARGET}")
if(BUILD_UTIL_PLUGIN)

  set(headers
    UtilPlugin.h
    Interpolator.h
    InverseKinematics.h
    DrawInterface.h
    Filter.h
    GeometricUtil.h
    ParamNode.h
    ParamWidget.h
    ParamSetupLayout.h
    ParamSetupDialog.h
    SetupToolBar.h
    )

  add_library(${TARGET} SHARED UtilPlugin GeometricUtil Interpolator InverseKinematics)

  # -l
  target_link_libraries(${TARGET} CnoidPoseSeqPlugin ${CHOREONOID_BASE_LIBRARIES})
  target_compile_definitions(${TARGET} PUBLIC ${CHOREONOID_COMPILE_DEFINITIONS})

  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

  set(INCLUDE_DEST_DIR ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/${PLUGIN_NAME})
  execute_process(
    COMMAND cmake -E make_directory ${INCLUDE_DEST_DIR}
    RESULT_VARIABLE _make_failed)
  # install(FILES ${headers} DESTINATION ${INCLUDE_DEST_DIR})
  foreach(header ${headers})
	  execute_process(
		  COMMAND cmake -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${header} ${INCLUDE_DEST_DIR}
		  RESULT_VARIABLE _copy_failed)
  endforeach()

  add_subdirectory(python)
else()
  message("Skip building ${TARGET}")
endif()
