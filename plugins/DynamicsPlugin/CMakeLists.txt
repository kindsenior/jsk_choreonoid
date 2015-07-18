message("This is DynamicsPlugin")

set(PLUGIN_NAME DynamicsPlugin)
set(TARGET Cnoid${PLUGIN_NAME})
set(headers DynamicsPlugin.h)

add_library(${TARGET} SHARED DynamicsPlugin Jacobian)

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

if(BUILD_DYNAMICSPLUGIN)

endif()
