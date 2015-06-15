message("This is PreviewControlPlugin")

set(TARGET CnoidPreviewControlPlugin)

# add openhrp3 include directories
execute_process(
	COMMAND pkg-config --variable=includedir openhrp3.1
	OUTPUT_VARIABLE openhrp3.1_includedir
	OUTPUT_STRIP_TRAILING_WHITESPACE
)
message("openhrp3.1_includedir: ${openhrp3.1_includedir}")
string(REGEX REPLACE "-I" "" openhrp3.1_includedir ${openhrp3.1_includedir})# remove -I
string(REGEX REPLACE " " ";" openhrp3.1_includedir ${openhrp3.1_includedir})# change string to list
include_directories(${openhrp3.1_includedir})

add_library(${TARGET} SHARED PreviewControlPlugin PreviewController)

if(BUILD_PREVIEWCONTROL)

endif()
