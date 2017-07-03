PROJECT(01_MLATools_lib)

INCLUDE_DIRECTORIES(${01_MLATools_lib_SOURCE_DIR}
    ${02_MLAMotion_lib_SOURCE_DIR}
	${GLM_INCLUDE_DIR}
	${GLEW_INCLUDE_DIR}
	${SDL_INCLUDE_DIR}
)

ADD_LIBRARY(01_MLATools_lib STATIC
	MLATools_lib.h 
	MLACommonInclude.h

	MLAShader/MLAShader.cpp
	MLAShader/MLAShader.h

    MLAMotionOperation/MLAMotionOperation.cpp
    MLAMotionOperation/MLAMotionOperation.h
)

SOURCE_GROUP("Fichiers d'en-tete" FILES MLATools_lib.h MLACommonInclude.h)

SOURCE_GROUP("MLAShader\\Fichiers d'en-tete" FILES MLAShader/MLAShader.h)
SOURCE_GROUP("MLAShader\\Fichiers sources" FILES MLAShader/MLAShader.cpp)

SOURCE_GROUP("MLAMotionOperation\\Fichiers d'en-tete" FILES MLAMotionOperation/MLAMotionOperation.h)
SOURCE_GROUP("MLAMotionOperation\\Fichiers sources" FILES MLAMotionOperation/MLAMotionOperation.cpp)