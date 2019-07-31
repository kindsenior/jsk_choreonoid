set(TARGET PyTestPlugin)
add_cnoid_python_module(${TARGET} PyTestPluginModule.cpp)
target_link_libraries(${TARGET})
set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PYTHON_DIR})# copy the library for calling 'import cnoid.hoge'
