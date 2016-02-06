message("This is MultiContactStabilizerPlugin")

set(TARGET CnoidMultiContactStabilizerPlugin)

add_library(${TARGET} SHARED MultiContactStabilizerPlugin)

set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/choreonoid-${choreonoid_VER})

if(BUILD_PREVIEWCONTROL)

endif()
