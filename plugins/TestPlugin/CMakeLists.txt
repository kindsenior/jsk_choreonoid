option(BUILD_TEST_PLUGIN "Build Test Plugin" ON)
set(TARGET CnoidTestPlugin)
message("TARGET ${TARGET}")
if(BUILD_TEST_PLUGIN)

  add_library(${TARGET} SHARED TestPlugin.cpp)
  target_link_libraries(${TARGET} CnoidBase ${CHOREONOID_BASE_LIBRARIES})
  target_compile_definitions(${TARGET} PUBLIC ${CHOREONOID_COMPILE_DEFINITIONS})
  set_target_properties(${TARGET}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

  add_subdirectory(python)

else()
  message("Skip building ${TARGET}")
endif()
