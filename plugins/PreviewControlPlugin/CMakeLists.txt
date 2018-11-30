message("This is PreviewControlPlugin")
option(BUILD_PREVIEWCONTROL_PLUGIN "Build PreviewControl Plugin" ON)
if(BUILD_PREVIEWCONTROL_PLUGIN)

  set(TARGET CnoidPreviewControlPlugin)

  pkg_check_modules(openhrp3 REQUIRED openhrp3.1)
  include_directories(${openhrp3_INCLUDE_DIRS})

  add_library(${TARGET} SHARED PreviewControlPlugin PreviewControlBar PreviewController)

  target_link_libraries(${TARGET} CnoidUtilPlugin)

  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

endif()
