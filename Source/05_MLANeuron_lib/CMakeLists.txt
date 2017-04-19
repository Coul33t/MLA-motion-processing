PROJECT(05_MLANeuron_lib)

INCLUDE_DIRECTORIES(${NEURON_INCLUDE_DIR}
)

ADD_LIBRARY(05_MLANeuron_lib STATIC
    MLANeuron_lib.h

    MLANeuronConnection/NeuronConnection.h
    MLANeuronConnection/NeuronConnection.cpp
)

SOURCE_GROUP("Fichiers d'en-tete" FILES MLANeuron_lib.h)

SOURCE_GROUP("MLANeuronConnection\\Fichiers d'en-tete"    FILES MLANeuronConnection/NeuronConnection.h)
SOURCE_GROUP("MLANeuronConnection\\Fichiers sources"      FILES MLANeuronConnection/NeuronConnection.cpp)