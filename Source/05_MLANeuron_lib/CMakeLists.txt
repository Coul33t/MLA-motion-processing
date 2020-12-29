PROJECT(05_MLANeuron_lib)

INCLUDE_DIRECTORIES(${NEURON_INCLUDE_DIR}
)

ADD_LIBRARY(05_MLANeuron_lib STATIC
    MLANeuron_lib.h

    MLANeuronConnection/NeuronConnection.h
    MLANeuronConnection/NeuronConnection.cpp
)

SOURCE_GROUP("headers" FILES MLANeuron_lib.h)

SOURCE_GROUP("MLANeuronConnection\\headers"    FILES MLANeuronConnection/NeuronConnection.h)
SOURCE_GROUP("MLANeuronConnection\\src"      FILES MLANeuronConnection/NeuronConnection.cpp)