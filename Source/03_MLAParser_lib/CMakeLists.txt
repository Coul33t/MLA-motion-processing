PROJECT(03_MLAParser_lib)

INCLUDE_DIRECTORIES(${02_MLAMotion_lib_SOURCE_DIR}
    ${01_MLATools_lib_SOURCE_DIR}
    ${GLM_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
    ${SDL_INCLUDE_DIR}
)

ADD_LIBRARY(03_MLAParser_lib STATIC
    MLAParser_lib.h

    MLABvhParser/MLABvhParser.h
    MLABvhParser/MLABvhParser.cpp
)

SOURCE_GROUP("headers" FILES MLAParser_lib.h)

SOURCE_GROUP("MLABvhParser\\headers"    FILES MLABvhParser/MLABvhParser.h)
SOURCE_GROUP("MLABvhParser\\src"      FILES MLABvhParser/MLABvhParser.cpp)