# create an executable
ADD_EXECUTABLE (controller_standalone
	${CS225A_COMMON_SOURCE}
	controller_standalone.cpp
)

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_standalone
        ${CS225A_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/controller_standalone)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY jaco.urdf mico.urdf world_mico.urdf world_jaco.urdf DESTINATION ${APP_RESOURCE_DIR})
