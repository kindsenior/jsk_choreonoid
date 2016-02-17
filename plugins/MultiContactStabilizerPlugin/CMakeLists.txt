message("This is MultiContactStabilizerPlugin")

set(TARGET CnoidMultiContactStabilizerPlugin)

message("Build qpOASES")
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0 ${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/bin)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/../../3rdparty/qpOASES/qpOASES-3.0/include)
# add_definitions(-DUSE_QPOASES)

# for openhrp3
# using openhrp3's Link class
pkg_check_modules(openhrp3 REQUIRED openhrp3.1)
include_directories(${openhrp3_INCLUDE_DIRS})
link_directories(${openhrp3_LIBRARY_DIRS})
execute_process(
	COMMAND pkg-config --variable=link_shared_files openhrp3.1
	OUTPUT_VARIABLE openhrp3_link_shared_files
	OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REGEX REPLACE "-l" "" openhrp3_link_shared_files ${openhrp3_link_shared_files})# remove -l
string(REGEX REPLACE " " ";" openhrp3_link_shared_files ${openhrp3_link_shared_files})# change string to list

add_library(${TARGET} SHARED MultiContactStabilizerPlugin Controller ContactConstraint QP test MultiContactStabilizerBar)

set(libs CnoidUtilPlugin qpOASES ${openhrp3_link_shared_files})
target_link_libraries(${TARGET} ${libs})

set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/choreonoid-${choreonoid_VER})

if(BUILD_PREVIEWCONTROL)

endif()
