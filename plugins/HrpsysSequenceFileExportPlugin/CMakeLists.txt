message("This is HrpsysSequenceFileExportPlugin")

set(TARGET CnoidHrpsysSequenceFileExportPlugin)

add_library(${TARGET} SHARED HrpsysSequenceFileExportPlugin)

target_link_libraries(${TARGET} CnoidUtilPlugin)

set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/choreonoid-${choreonoid_VER})

if(BUILD_PREVIEWCONTROL)

endif()
