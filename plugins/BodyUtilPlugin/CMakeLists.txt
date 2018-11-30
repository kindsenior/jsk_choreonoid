message("This is BodyUtilPlugin")
option(BUILD_BODYUTIL_PLUGIN "Build BodyUtil Plugin" ON)
if(BUILD_BODYUTIL_PLUGIN)

  add_subdirectory(python)

endif()
