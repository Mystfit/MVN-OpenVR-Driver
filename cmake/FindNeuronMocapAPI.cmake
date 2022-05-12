# set(NeuronMocapAPI_DIR "" CACHE PATH "NeuronMocapAPI folder")
set(NeuronMocapAPI_DIR "${DEPENDANT_LIB_DIR}/PerceptionNeuronSDK")

find_library(NeuronMocapAPI_LIBRARY NAMES MocapApi HINTS "${NeuronMocapAPI_DIR}/lib/win32/x64")
find_file(NeuronMocapAPI_BINARY NAMES MocapApi.dll HINTS "${NeuronMocapAPI_DIR}/bin/win32/x64")

if(NeuronMocapAPI_LIBRARY)
    set(NeuronMocapAPI_FOUND ON)
endif()

add_library(NeuronMocapAPI SHARED IMPORTED)
set_property(TARGET NeuronMocapAPI PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NeuronMocapAPI_DIR}/include)
set_property(TARGET NeuronMocapAPI PROPERTY IMPORTED_IMPLIB ${NeuronMocapAPI_LIBRARY})
set_property(TARGET NeuronMocapAPI PROPERTY IMPORTED_LOCATION ${NeuronMocapAPI_BINARY})
