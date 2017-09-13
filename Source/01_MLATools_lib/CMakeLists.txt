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

	MLAShader/MLAShader.h
	MLAShader/MLAShader.cpp

    MLAMotionOperation/MLAMotionOperation.h
    MLAMotionOperation/MLAMotionOperation.cpp

    MLADataExport/CSVExport.h
    MLADataExport/CSVExport.cpp

    MLADataExport/PipeExport.h
    MLADataExport/PipeExport.cpp)

SOURCE_GROUP("Fichiers d'en-tete" FILES MLATools_lib.h MLACommonInclude.h)

SOURCE_GROUP("MLAShader\\Fichiers d'en-tete" FILES MLAShader/MLAShader.h)
SOURCE_GROUP("MLAShader\\Fichiers sources" FILES MLAShader/MLAShader.cpp)

SOURCE_GROUP("MLAMotionOperation\\Fichiers d'en-tete" FILES MLAMotionOperation/MLAMotionOperation.h)
SOURCE_GROUP("MLAMotionOperation\\Fichiers sources" FILES MLAMotionOperation/MLAMotionOperation.cpp)

SOURCE_GROUP("MLADataExport\\Fichiers d'en-tete" FILES MLADataExport/CSVExport.h)
SOURCE_GROUP("MLADataExport\\Fichiers sources" FILES MLADataExport/CSVExport.cpp)

SOURCE_GROUP("MLADataExport\\Fichiers d'en-tete" FILES MLADataExport/PipeExport.h)
SOURCE_GROUP("MLADataExport\\Fichiers sources" FILES MLADataExport/PipeExport.cpp)
