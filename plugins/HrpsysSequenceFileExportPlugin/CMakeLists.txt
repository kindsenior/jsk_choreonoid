message("This is HrpsysSequenceFileExportPlugin")

set(TARGET CnoidHrpsysSequenceFileExportPlugin)

add_library(${TARGET} SHARED HrpsysSequenceFileExportPlugin)

target_link_libraries(${TARGET} CnoidUtilPlugin)

set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

if(BUILD_PREVIEWCONTROL)

endif()
