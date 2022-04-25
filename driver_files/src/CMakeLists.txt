message(STATUS "Configuring ${DRIVER_TARGET}")

set(DRIVER_HEADERS 
	"${CMAKE_CURRENT_LIST_DIR}/Driver/cmath_fix.h"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/ControllerDevice.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/DeviceType.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/HMDDevice.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/IVRDevice.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/IVRDriver.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/Key.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/TrackerDevice.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/TrackingReferenceDevice.hpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/VRDriver.hpp"
)
set(DRIVER_SOURCES
	"${CMAKE_CURRENT_LIST_DIR}/Driver/ControllerDevice.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/HMDDevice.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/Key.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/TrackerDevice.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/TrackingReferenceDevice.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/Driver/VRDriver.cpp"
)
set(NATIVE_HEADERS 	"${CMAKE_CURRENT_LIST_DIR}/Native/DriverFactory.hpp")
set(NATIVE_SOURCES 	"${CMAKE_CURRENT_LIST_DIR}/Native/DriverFactory.cpp")
set(MVN_HEADERS
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/angularsegmentkinematicsdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/centerofmassdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/datagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/eulerdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/jointanglesdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/linearsegmentkinematicsdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/metadatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/parsermanager.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/positiondatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/quaterniondatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/scaledatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/streamer.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/timecodedatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/trackerkinematicsdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/udpserver.h"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/segments.h"
)
set(MVN_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/angularsegmentkinematicsdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/centerofmassdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/datagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/eulerdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/jointanglesdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/linearsegmentkinematicsdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/metadatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/parsermanager.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/positiondatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/quaterniondatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/scaledatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/streamer.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/timecodedatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/trackerkinematicsdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Driver/MVN/udpserver.cpp"
)


# Driver
set(DRIVER_NAME "MVN")
set(DRIVER_TARGET "driver_${DRIVER_NAME}")
add_library("${DRIVER_TARGET}" SHARED "")

set_property(TARGET "${DRIVER_TARGET}" PROPERTY CXX_STANDARD 17)
set_property(TARGET "${DRIVER_TARGET}" PROPERTY PREFIX "")
source_group(driver FILES 
	${DRIVER_HEADERS}
	${DRIVER_SOURCES}
)
source_group(native FILES 
	${NATIVE_HEADERS}
	${NATIVE_SOURCES}
)
source_group(MVN FILES
	${MVN_HEADERS}
	${MVN_SOURCES}
)

target_sources(${DRIVER_TARGET} 
	PRIVATE 
	${DRIVER_HEADERS} 
	${DRIVER_SOURCES}
	${NATIVE_HEADERS}
	${NATIVE_SOURCES}
	${MVN_HEADERS}
	${MVN_SOURCES}
)
target_include_directories(${DRIVER_TARGET} PUBLIC 
	"${CMAKE_CURRENT_LIST_DIR}"
	"${SOURCE_DIR}/libraries/linalg"
	"${OPENVR_INCLUDE_DIR}"
)
target_link_libraries(${DRIVER_TARGET} 
	PUBLIC 
	${OPENVR_LIB}
	PRIVATE
	xstypes
)

# Copy driver assets to output folder
set(DRIVER_RESOURCE_DEST $<TARGET_FILE_DIR:${DRIVER_TARGET}>/driver)
add_custom_command(
    TARGET ${DRIVER_TARGET}
    PRE_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_LIST_DIR}/../driver" "${DRIVER_RESOURCE_DEST}"
    DEPENDS ${DRIVER_RESOURCE_DEST}
)

# Copy dll to output folder
set(DRIVER_DLL_DEST ${DRIVER_RESOURCE_DEST}/${DRIVER_NAME}/bin/${PLATFORM_NAME}${PROCESSOR_ARCH})
add_custom_command(
    TARGET ${DRIVER_TARGET} 
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${DRIVER_TARGET}>" "${DRIVER_DLL_DEST}"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:xstypes>" "${DRIVER_DLL_DEST}"
    DEPENDS ${DRIVER_DLL_SRC} ${DRIVER_DLL_DEST}/$<TARGET_FILE_NAME:${DRIVER_TARGET}>
)
