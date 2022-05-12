# XSTypes lib provides native MVN Animate types
set(NEURON_HEADERS
    "${CMAKE_CURRENT_LIST_DIR}/NeuronStreamSource.h"
	"${CMAKE_CURRENT_LIST_DIR}/NeuronSegments.h"
	"${CMAKE_CURRENT_LIST_DIR}/BVH.h"
)
set(NEURON_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/NeuronStreamSource.cpp"
  	"${CMAKE_CURRENT_LIST_DIR}/BVH.cpp"
)

source_group(Headers FILES${NEURON_HEADERS})
source_group(Sources FILE ${NEURON_SOURCES})

# MVN sources
set(DRIVER_SOURCES 
	${NEURON_HEADERS} 
	${NEURON_SOURCES}
)

# Create static lib to hold our MVN stream library
set(NEURONANIMATE_TARGET AxisStudioStream)
add_library(${NEURONANIMATE_TARGET} STATIC ${DRIVER_SOURCES})

# Propagate source folder up to the main library
target_include_directories(${NEURONANIMATE_TARGET} PUBLIC 
    ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/../../Common
    ${DEPENDANT_LIB_DIR}/linalg 
    ${DEPENDANT_LIB_DIR}/openvr/headers 
)

# Find the NeuronDataReader lib from the Neuron SDK at https://neuronmocap.com/system/files/software/NeuronDataReaderSDK%20b18_0.zip
find_package(NeuronMocapAPI)
target_link_libraries(${NEURONANIMATE_TARGET} PUBLIC NeuronMocapAPI)
set(MOCAP_PRIVATE_LIBS 
	${MOCAP_PRIVATE_LIBS}
	${NEURONANIMATE_TARGET}
	PARENT_SCOPE
)

# Copy NeuronDataReader DLL dependencies to driver
set(DRIVER_DLL_DEST ${DRIVER_RESOURCE_DEST}/bin/${PLATFORM_NAME}${PROCESSOR_ARCH})
set(NEURONMOCAPAPI_DLL_DEST ${DRIVER_DLL_DEST}/MocapApi.dll)
add_custom_command(
	POST_BUILD
	TARGET ${NEURONANIMATE_TARGET}
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:NeuronMocapAPI>" "${NEURONMOCAPAPI_DLL_DEST}"
    DEPENDS ${MVNAnimateStream} 
)
