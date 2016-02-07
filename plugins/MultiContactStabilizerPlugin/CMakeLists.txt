message("This is MultiContactStabilizerPlugin")

set(TARGET CnoidMultiContactStabilizerPlugin)

message("Build qpOASES")
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0 ${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/bin)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/include)
# add_definitions(-DUSE_QPOASES)

add_library(${TARGET} SHARED MultiContactStabilizerPlugin Controller)

set(libs CnoidUtilPlugin qpOASES)
target_link_libraries(${TARGET} ${libs})

set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/choreonoid-${choreonoid_VER})

if(BUILD_PREVIEWCONTROL)

endif()
