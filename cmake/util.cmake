function(add_cnoid_python_module)

  set(choreonoid_python_dir ${choreonoid_PLUGINDIR}/python/cnoid) # TODO

  set(target ${ARGV0})
  string(REGEX REPLACE "^Py(.+)$" "\\1" module ${target})
  set(sources ${ARGV})
  list(REMOVE_AT sources 0)

  add_library(${target} SHARED ${sources})

  if(NOT WIN32)
    set_target_properties(${target}  PROPERTIES
      COMPILE_DEFINITIONS "BOOST_PYTHON_USE_GCC_SYMBOL_VISIBILITY" )
  else()
    set_target_properties(${target}  PROPERTIES SUFFIX .pyd)
  endif()

  set_target_properties(${target}  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${choreonoid_python_dir}
    LIBRARY_OUTPUT_DIRECTORY ${choreonoid_python_dir}
    RUNTIME_OUTPUT_NAME ${module}
    LIBRARY_OUTPUT_NAME ${module}
    PREFIX "")

  install(TARGETS ${target}
    RUNTIME DESTINATION ${choreonoid_python_dir} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
    LIBRARY DESTINATION ${choreonoid_python_dir} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)

endfunction()
