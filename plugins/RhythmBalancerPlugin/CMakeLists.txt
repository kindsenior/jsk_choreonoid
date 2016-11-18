message("This is RhythmBalancerPlugin")

set(TARGET CnoidRhythmBalancerPlugin)

add_library(${TARGET} SHARED RhythmBalancerPlugin)

target_link_libraries(${TARGET} CnoidUtilPlugin boost_thread)

set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/choreonoid-${choreonoid_VER})

if(BUILD_PREVIEWCONTROL)

endif()
