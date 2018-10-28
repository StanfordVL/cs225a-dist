# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${SAI2-COMMON_INCLUDE_DIRS})

# create an executable
ADD_EXECUTABLE (01-sim_urdfmodel 01-sim_urdfmodel.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (01-sim_urdfmodel
	${SAI2-COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY world.urdf pbot.urdf DESTINATION ${RESOURCE_DIR})
