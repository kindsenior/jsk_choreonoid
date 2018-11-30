message("This is CnoidTestPlugin")
set(TARGET CnoidTestPlugin)
option(BUILD_TEST_PLUGIN "Build Test Plugin" ON)
if(BUILD_TEST_PLUGIN)

  add_library(${TARGET} SHARED TestPlugin.cpp)
  target_link_libraries(${TARGET} CnoidBase)
  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

  add_subdirectory(python)

endif()
