option(BUILD_SANDBOX_PLUGIN "Build Sandbox Plugin" ON)
set(TARGET CnoidSandboxPlugin)
message("TARGET: ${TARGET}")
if(BUILD_SANDBOX_PLUGIN)

  add_library(${TARGET} SHARED SandboxPlugin)

  target_link_libraries(${TARGET} CnoidUtilPlugin boost_thread)

  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

else()
  message("Skip building ${TARGET}")
endif()
