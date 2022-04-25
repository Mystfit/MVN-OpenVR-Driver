set(XSENS_DIR "C:/Program Files/Xsens" CACHE PATH "XSens root folder")
if(NOT DEFINED MVN_TOOLKIT_DIR)
	file(GLOB toolkit_dir LIST_DIRECTORIES true "${XSENS_DIR}/Xsens MVN Developer Toolkit*")
	list(GET ${toolkit_dir} -1 MVN_SDK_DIR)
	set(MVN_TOOLKIT_DIR ${toolkit_dir} CACHE PATH "MVN Toolkit folder")
	message(STATUS "Found MVN Toolkit at ${MVN_TOOLKIT_DIR}")
endif()

find_library(xstypes_LIBRARY NAMES xstypes64 HINTS "${MVN_TOOLKIT_DIR}/x64/lib")
find_file(xstypes_BINARY NAMES xstypes64.dll HINTS "${MVN_TOOLKIT_DIR}/x64/lib")

if(xstypes_LIBRARY)
    set(xstypes_FOUND ON)
endif()

add_library(xstypes SHARED IMPORTED)
set_property(TARGET xstypes PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${MVN_TOOLKIT_DIR}/x64/include)
set_property(TARGET xstypes PROPERTY IMPORTED_IMPLIB ${xstypes_LIBRARY})
set_property(TARGET xstypes PROPERTY IMPORTED_LOCATION ${xstypes_BINARY})
