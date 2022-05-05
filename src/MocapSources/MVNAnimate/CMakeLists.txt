# XSTypes lib provides native MVN Animate types
set(MVN_HEADERS
    "${CMAKE_CURRENT_LIST_DIR}/angularsegmentkinematicsdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/centerofmassdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/datagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/eulerdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/jointanglesdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/linearsegmentkinematicsdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/metadatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/parsermanager.h"
    "${CMAKE_CURRENT_LIST_DIR}/positiondatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/quaterniondatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/scaledatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/streamer.h"
    "${CMAKE_CURRENT_LIST_DIR}/timecodedatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/trackerkinematicsdatagram.h"
    "${CMAKE_CURRENT_LIST_DIR}/udpserver.h"
    "${CMAKE_CURRENT_LIST_DIR}/segments.h"
    "${CMAKE_CURRENT_LIST_DIR}/MVNStreamSource.h"
)
set(MVN_SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/angularsegmentkinematicsdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/centerofmassdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/datagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/eulerdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/jointanglesdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/linearsegmentkinematicsdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/metadatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/parsermanager.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/positiondatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/quaterniondatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/scaledatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/streamer.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/timecodedatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/trackerkinematicsdatagram.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/udpserver.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/MVNStreamSource.cpp"
)

source_group(Headers FILES${MVN_HEADERS})
source_group(Sources FILE ${MVN_SOURCES})

# MVN sources
set(DRIVER_SOURCES 
	${MVN_HEADERS} 
	${MVN_SOURCES}
)

# Create static lib to hold our MVN stream library
set(MVNANIMATE_TARGET MVNAnimateStream)
add_library(${MVNANIMATE_TARGET} STATIC ${DRIVER_SOURCES})

# Propagate source folder up to the main library
target_include_directories(${MVNANIMATE_TARGET} PUBLIC 
    ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/../../Common
    ${DEPENDANT_LIB_DIR}/linalg 
    ${DEPENDANT_LIB_DIR}/openvr/headers 
)

# Find the xstypes lib from the MVN Toolkit at https://www.xsens.com/software-downloads
find_package(xstypes)
target_link_libraries(${MVNANIMATE_TARGET} PUBLIC xstypes)
set(MOCAP_PRIVATE_LIBS 
	${MOCAP_PRIVATE_LIBS}
	${MVNANIMATE_TARGET}
	PARENT_SCOPE
)

# Copy XSens DLL dependencies to driver
set(DRIVER_DLL_DEST ${DRIVER_RESOURCE_DEST}/bin/${PLATFORM_NAME}${PROCESSOR_ARCH})
set(XSENSE_DLL_DEST ${DRIVER_DLL_DEST}/xstypes.dll)
add_custom_command(
	POST_BUILD
	TARGET ${MVNANIMATE_TARGET}
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:xstypes>" "${XSENSE_DLL_DEST}"
    DEPENDS ${MVNAnimateStream} 
)
add_custom_target(task_MVNCopyLibs ALL
    DEPENDS ${XSENSE_DLL_DEST}
)