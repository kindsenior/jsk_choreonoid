message("This is HrpsysSequenceFileExportPlugin")
option(BUILD_HRPSYSSEQUENCEFILEEXPORT_PLUGIN "Build HrpsysSequenceFileExport Plugin" ON)
if(BUILD_HRPSYSSEQUENCEFILEEXPORT_PLUGIN)

  set(TARGET CnoidHrpsysSequenceFileExportPlugin)

  add_library(${TARGET} SHARED HrpsysSequenceFileExportPlugin)

  target_link_libraries(${TARGET} CnoidUtilPlugin)

  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

endif()
