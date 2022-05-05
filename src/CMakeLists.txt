message(STATUS "Configuring ${DRIVER_TARGET}")

# Driver
set(DRIVER_NAME "Mocap")
set(DRIVER_TARGET "driver_${DRIVER_NAME}")

add_library("${DRIVER_TARGET}" SHARED "")
set_property(TARGET "${DRIVER_TARGET}" PROPERTY PREFIX "")

set(DRIVER_SOURCES "")
set(MOCAP_PRIVATE_LIBS "")
set(DRIVER_INCLUDES "")

set(COMMON_SOURCES 
    "${CMAKE_CURRENT_LIST_DIR}/Common/IMocapStreamSource.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/Common/IVRDevice.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/Common/IVRDriver.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/Common/concurrentqueue.h"
)
 
# Common driver core
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/Driver)

# MVN Animate suit source
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/MocapSources/MVNAnimate)

# Set all sources
target_sources(${DRIVER_TARGET} PRIVATE ${DRIVER_SOURCES} ${COMMON_SOURCES})

# Set include directories
target_include_directories(${DRIVER_TARGET} PUBLIC 
	${DRIVER_INCLUDES} 
	${CMAKE_CURRENT_LIST_DIR}/Common
)

# Set linked libraries
target_link_libraries(${DRIVER_TARGET} PRIVATE "${MOCAP_PRIVATE_LIBS}")

# Copy driver assets to output folder
add_custom_command(
    TARGET ${DRIVER_TARGET}
    PRE_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${SOURCE_DIR}/resources" "${DRIVER_RESOURCE_DEST}"
    DEPENDS ${DRIVER_RESOURCE_DEST}
)

# Copy dll to output folder
set(DRIVER_DLL_DEST ${DRIVER_RESOURCE_DEST}/bin/${PLATFORM_NAME}${PROCESSOR_ARCH})
set(PDB_COPY_COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_PDB_FILE:${DRIVER_TARGET}> ${DRIVER_DLL_DEST}/$<TARGET_PDB_FILE_NAME:${DRIVER_TARGET}>)
add_custom_command(
    TARGET ${DRIVER_TARGET} 
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${DRIVER_TARGET}>" "${DRIVER_DLL_DEST}/$<TARGET_FILE_NAME:${DRIVER_TARGET}>"
    COMMAND "$<$<CONFIG:Debug,RelWithDebInfo>:${PDB_COPY_COMMAND}>"
    DEPENDS ${DRIVER_DLL_SRC} ${DRIVER_DLL_DEST}/$<TARGET_FILE_NAME:${DRIVER_TARGET}>
    VERBATIM
    COMMAND_EXPAND_LISTS
)
