message("This is PreviewControlPlugin")

set(TARGET CnoidPreviewControlPlugin)

# add openhrp3 include directories
# execute_process(
# 	COMMAND pkg-config --variable=includedir openhrp3.1
# 	OUTPUT_VARIABLE openhrp3.1_includedir
# 	OUTPUT_STRIP_TRAILING_WHITESPACE
# )
# message("openhrp3.1_includedir: ${openhrp3.1_includedir}")
# string(REGEX REPLACE "-I" "" openhrp3.1_includedir ${openhrp3.1_includedir})# remove -I
# string(REGEX REPLACE " " ";" openhrp3.1_includedir ${openhrp3.1_includedir})# change string to list
# include_directories(${openhrp3_includedir})
find_package(PkgConfig)
pkg_check_modules(openhrp3 REQUIRED openhrp3.1)
include_directories(${openhrp3_INCLUDE_DIRS})

add_library(${TARGET} SHARED PreviewControlPlugin PreviewControlBar PreviewController)

target_link_libraries(${TARGET} CnoidUtilPlugin)

set_target_properties(${TARGET}
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/choreonoid-${choreonoid_VER})

if(BUILD_PREVIEWCONTROL)

endif()
