cmake_minimum_required(VERSION "3.7.1")

cmake_policy(SET CMP0079 NEW)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake;${CMAKE_MODULE_PATH}")

# Solution
project("MVNAnimate_SteamVR_Driver")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Deps
set(OPENVR_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libraries/openvr/headers")

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

# Xsens types library
find_package(xstypes)
find_library(OPENVR_LIB openvr_api HINTS "${CMAKE_CURRENT_SOURCE_DIR}/libraries/openvr/lib/${PLATFORM_NAME}${PROCESSOR_ARCH}/" NO_DEFAULT_PATH )

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/driver_files/src")
