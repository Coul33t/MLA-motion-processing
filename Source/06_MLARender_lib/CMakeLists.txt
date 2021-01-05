PROJECT(06_MLARender_lib)

INCLUDE_DIRECTORIES(
    ${01_MLATools_lib_SOURCE_DIR}
    ${02_MLAMotion_lib_SOURCE_DIR}
    ${06_MLARender_lib_SOURCE_DIR}
    ${GLM_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
    ${SDL_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
)

ADD_LIBRARY(06_MLARender_lib STATIC
    MLARender_lib.h

    MLARender/FrameRender.h
    MLARender/FrameRender.cpp
)

SOURCE_GROUP("headers" FILES MLARender_lib.h)

SOURCE_GROUP("MLARender\\headers"    FILES MLARender/FrameRender.h)
SOURCE_GROUP("MLARender\\src"      FILES MLARender/FrameRender.cpp)