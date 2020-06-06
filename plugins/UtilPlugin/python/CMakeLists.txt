set(TARGET PyUtilPlugin)
add_cnoid_python_module(${TARGET} PyUtilPluginModule.cpp)
target_link_libraries(${TARGET} CnoidBase ${CHOREONOID_BASE_LIBRARIES})
target_compile_definitions(${TARGET} PUBLIC ${CHOREONOID_COMPILE_DEFINITIONS})
set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PYTHON_DIR})# copy the library for calling 'import cnoid.hoge'
