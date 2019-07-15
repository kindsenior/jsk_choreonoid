option(BUILD_HRPSYSSEQUENCEFILEEXPORT_PLUGIN "Build HrpsysSequenceFileExport Plugin" ON)
set(TARGET CnoidHrpsysSequenceFileExportPlugin)
message("TARGET: ${TARGET}")
if(BUILD_HRPSYSSEQUENCEFILEEXPORT_PLUGIN)

  add_library(${TARGET} SHARED HrpsysSequenceFileExportPlugin)

  target_link_libraries(${TARGET} CnoidUtilPlugin)

  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

else()
  message("Skip building ${TARGET}")
endif()
