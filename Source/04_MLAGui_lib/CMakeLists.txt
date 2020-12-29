PROJECT(04_MLAGui_lib)

INCLUDE_DIRECTORIES(
    ${01_MLATools_lib_SOURCE_DIR}
    ${02_MLAMotion_lib_SOURCE_DIR}
    ${03_MLAParser_lib_SOURCE_DIR}  
    ${04_MLAGui_lib_SOURCE_DIR}
    ${05_MLANeuron_lib_SOURCE_DIR}
    ${06_MLARender_lib_SOURCE_DIR}
    ${GLM_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
    ${SDL_INCLUDE_DIR}
	${NEURON_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
)

ADD_LIBRARY(04_MLAGui_lib STATIC
    MLAGui_lib.h

    MLACamera/MLACamera.h
    MLACamera/MLACamera.cpp

    MLAInput/MLAInput.h
    MLAInput/MLAInput.cpp

    MLAMainWindow/MLAMainWindow.h
    MLAMainWindow/MLAMainWindow.cpp
)

SOURCE_GROUP("headers" FILES MLAGui_lib.h)

SOURCE_GROUP("MLACamera\\headers"    FILES MLACamera/MLACamera.h)
SOURCE_GROUP("MLACamera\\src"      FILES MLACamera/MLACamera.cpp)

SOURCE_GROUP("MLAInput\\headers"    FILES MLAInput/MLAInput.h)
SOURCE_GROUP("MLAInput\\src"      FILES MLAInput/MLAInput.cpp)

SOURCE_GROUP("MLAMainWindow\\headers"    FILES MLAMainWindow/MLAMainWindow.h)
SOURCE_GROUP("MLAMainWindow\\src"      FILES MLAMainWindow/MLAMainWindow.cpp)