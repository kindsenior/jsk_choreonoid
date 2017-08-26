message("This is RMControlPlugin")

set(TARGET CnoidRMControlPlugin)

add_library(${TARGET} SHARED RMControlPlugin RMControlBar)

target_link_libraries(${TARGET} CnoidUtilPlugin)

set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

if(BUILD_PREVIEWCONTROL)

endif()
