message("This is UtilPlugin")

set(PLUGIN_NAME UtilPlugin)
set(TARGET Cnoid${PLUGIN_NAME})
set(headers UtilPlugin.h)

add_library(${TARGET} SHARED UtilPlugin)

set(INCLUDE_DEST_DIR ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/${PLUGIN_NAME})
execute_process(
  COMMAND cmake -E make_directory ${INCLUDE_DEST_DIR}
  RESULT_VARIABLE _make_failed)
# install(FILES ${headers} DESTINATION ${INCLUDE_DEST_DIR})
foreach(header ${headers})
	execute_process(
		COMMAND cmake -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${headers} ${INCLUDE_DEST_DIR}
		RESULT_VARIABLE _copy_failed)
endforeach()

if(BUILD_PREVIEWCONTROL)

endif()
