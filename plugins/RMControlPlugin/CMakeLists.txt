message("This is RMControlPlugin")

set(TARGET CnoidRMControlPlugin)

add_library(${TARGET} SHARED RMControlPlugin)

if(BUILD_PREVIEWCONTROL)

endif()
