cmake_minimum_required(VERSION "3.13.1")

cmake_policy(SET CMP0079 NEW)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake;${CMAKE_MODULE_PATH}")
set(CMAKE_CXX_STANDARD 17)

# Solution
project("MocapSuit_SteamVR_Driver")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(SIZEOF_VOIDP ${CMAKE_SIZEOF_VOID_P})
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PROCESSOR_ARCH "64")
else()
    set(PROCESSOR_ARCH "32")
endif()

if(WIN32)
    set(PLATFORM_NAME "win")
elseif(UNIX AND NOT APPLE)
    set(PLATFORM_NAME "linux")
elseif(APPLE)
    set(PLATFORM_NAME "osx")
endif()

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DEPENDANT_LIB_DIR ${SOURCE_DIR}/libraries)
set(DRIVER_RESOURCE_DEST ${CMAKE_CURRENT_BINARY_DIR}/driver/$<CONFIG>)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")
