set(DRIVER_IMP_HEADERS 
	"${CMAKE_CURRENT_LIST_DIR}/cmath_fix.h"
	"${CMAKE_CURRENT_LIST_DIR}/ControllerDevice.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/HMDDevice.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/Key.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/TrackerDevice.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/TrackingReferenceDevice.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/VRDriver.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/DriverFactory.hpp"
)
set(DRIVER_IMP_SOURCES
	"${CMAKE_CURRENT_LIST_DIR}/ControllerDevice.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/HMDDevice.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/Key.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/TrackerDevice.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/TrackingReferenceDevice.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/VRDriver.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/DriverFactory.cpp"
)

set(COMMON_HEADERS
	"${CMAKE_CURRENT_LIST_DIR}/../Common/DeviceType.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/../Common/IMocapStreamSource.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/../Common/PoseMath.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/../Common/IVRDevice.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/../Common/IVRDriver.hpp"
)

set(DRIVER_SOURCES  
	${DRIVER_SOURCES}
	${DRIVER_IMP_HEADERS} 
	${DRIVER_IMP_SOURCES}
	${COMMON_HEADERS}
	PARENT_SCOPE
)

source_group(Common FILES ${COMMON_HEADERS})
source_group(Headers FILES ${DRIVER_IMP_HEADERS})
source_group(Source FILES ${DRIVER_IMP_SOURCES})

# Find OpenVR lib
find_library(OPENVR_LIB openvr_api HINTS "${DEPENDANT_LIB_DIR}/openvr/lib/${PLATFORM_NAME}${PROCESSOR_ARCH}/" NO_DEFAULT_PATH )
set(MOCAP_PRIVATE_LIBS 
	${MOCAP_PRIVATE_LIBS} 
	${OPENVR_LIB}
	PARENT_SCOPE
)

set(DRIVER_INCLUDES
	${DRIVER_INCLUDES} 
	${DEPENDANT_LIB_DIR}/linalg 
	${DEPENDANT_LIB_DIR}/openvr/headers 
	PARENT_SCOPE
)
