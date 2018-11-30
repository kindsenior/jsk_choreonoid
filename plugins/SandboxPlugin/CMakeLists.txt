message("This is SandboxPlugin")
option(BUILD_SANDBOX_PLUGIN "Build Sandbox Plugin" ON)
if(BUILD_SANDBOX_PLUGIN)

  set(TARGET CnoidSandboxPlugin)

  add_library(${TARGET} SHARED SandboxPlugin)

  target_link_libraries(${TARGET} CnoidUtilPlugin boost_thread)

  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

endif()
