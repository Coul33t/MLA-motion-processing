PROJECT(06_MLARender_lib)

INCLUDE_DIRECTORIES(${RENDER_INCLUDE_DIR}
    ${01_MLATools_lib_SOURCE_DIR}
    ${02_MLAMotion_lib_SOURCE_DIR}
    ${GLM_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
    ${SDL_INCLUDE_DIR}
)

ADD_LIBRARY(06_MLARender_lib STATIC
    MLARender_lib.h

    MLARender/FrameRender.h
    MLARender/FrameRender.cpp
)

SOURCE_GROUP("Fichiers d'en-tete" FILES MLARender_lib.h)

SOURCE_GROUP("MLARender\\Fichiers d'en-tete"    FILES MLARender/FrameRender.h)
SOURCE_GROUP("MLARender\\Fichiers sources"      FILES MLARender/FrameRender.cpp)