message("This is CnoidTestPlugin")
set(TARGET CnoidTestPlugin)

add_library(${TARGET} SHARED TestPlugin.cpp)
target_link_libraries(${TARGET} CnoidBase)
set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PLUGINDIR})

add_subdirectory(python)
