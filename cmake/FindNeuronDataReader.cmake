set(NEURONREADER_DIR "" CACHE PATH "NeuronDataReader folder")

find_library(NeuronDataReader_LIBRARY NAMES NeuronDataReader HINTS "${NEURONREADER_DIR}/lib/win/desktop/x64")
find_file(NeuronDataReader_BINARY NAMES NeuronDataReader.dll HINTS "${NEURONREADER_DIR}/lib/win/desktop/x64")

if(NeuronDataReader_LIBRARY)
    set(NeuronDataReader_FOUND ON)
endif()

add_library(NeuronDataReader SHARED IMPORTED)
set_property(TARGET NeuronDataReader PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NEURONREADER_DIR}/include/NeuronDataReader)
set_property(TARGET NeuronDataReader PROPERTY IMPORTED_IMPLIB ${NeuronDataReader_LIBRARY})
set_property(TARGET NeuronDataReader PROPERTY IMPORTED_LOCATION ${NeuronDataReader_BINARY})
