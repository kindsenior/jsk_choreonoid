option(BUILD_PREVIEWCONTROL_PLUGIN "Build PreviewControl Plugin" ON)
set(TARGET CnoidPreviewControlPlugin)
message("TARGET: ${TARGET}")
if(BUILD_PREVIEWCONTROL_PLUGIN)

  pkg_check_modules(openhrp3 REQUIRED openhrp3.1)
  include_directories(${openhrp3_INCLUDE_DIRS})

  pkg_check_modules(hrpsys hrpsys-base REQUIRED)
  include_directories(${hrpsys_INCLUDE_DIRS})

  add_library(${TARGET} SHARED PreviewControlPlugin PreviewControlBar PreviewController)

  target_link_libraries(${TARGET} CnoidUtilPlugin)

  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

else()
  message("Skip building ${TARGET}")
endif()
