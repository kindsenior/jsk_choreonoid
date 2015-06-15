message("This is DynamicsPlugin")

set(TARGET CnoidDynamicsPlugin)

add_library(${TARGET} SHARED DynamicsPlugin Jacobian)

if(BUILD_PREVIEWCONTROL)

endif()
