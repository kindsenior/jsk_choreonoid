message("This is SandboxPlugin")

set(TARGET CnoidSandboxPlugin)

add_library(${TARGET} SHARED SandboxPlugin)

target_link_libraries(${TARGET} CnoidUtilPlugin boost_thread)

set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

if(BUILD_PREVIEWCONTROL)

endif()
