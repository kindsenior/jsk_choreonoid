set(TARGET PyBodyUtil)
add_cnoid_python_module(${TARGET} PyBodyUtilModule.cpp)
target_link_libraries(${TARGET} CnoidBody)
set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${choreonoid_PYTHON_DIR})# copy the library for calling 'import cnoid.hoge'
