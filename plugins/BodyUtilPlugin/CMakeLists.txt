option(BUILD_BODYUTIL_PLUGIN "Build BodyUtil Plugin" ON)
set(TARGET CnoidBodyUtilPlugin)
message("TARGET: ${TARGET}")
if(BUILD_BODYUTIL_PLUGIN)

  add_subdirectory(python)

else()
  message("Skip building ${TARGET}")
endif()
