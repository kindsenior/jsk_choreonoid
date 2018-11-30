message("This is RMControlPlugin")
option(BUILD_RMCONTROL_PLUGIN "Build RMcontrol Plugin" ON)
if(BUILD_RMCONTROL_PLUGIN)

  set(TARGET CnoidRMControlPlugin)

  add_library(${TARGET} SHARED RMControlPlugin RMControlBar)

  target_link_libraries(${TARGET} CnoidUtilPlugin)

  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

endif()
