option(BUILD_RMCONTROL_PLUGIN "Build RMcontrol Plugin" ON)
set(TARGET CnoidRMControlPlugin)
message("TARGET: ${TARGET}")
if(BUILD_RMCONTROL_PLUGIN)

  add_library(${TARGET} SHARED RMControlPlugin RMControlBar)

  target_link_libraries(${TARGET} CnoidUtilPlugin)

  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

else()
  message("Skip building ${TARGET}")
endif()
