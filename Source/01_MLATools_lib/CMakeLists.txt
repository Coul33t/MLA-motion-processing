PROJECT(01_MLATools_lib)

INCLUDE_DIRECTORIES(${01_MLATools_lib_SOURCE_DIR}
    ${02_MLAMotion_lib_SOURCE_DIR}
	${GLM_INCLUDE_DIR}
	${GLEW_INCLUDE_DIR}
	${SDL_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
)

ADD_LIBRARY(01_MLATools_lib STATIC
	MLATools_lib.h
	MLACommonInclude.h

	MLAShader/MLAShader.h
	MLAShader/MLAShader.cpp

    MLAMotionOperation/MLAMotionOperation.h
    MLAMotionOperation/MLAMotionOperation.cpp

    MLAMotionOperation/MLASpeedData.h
    MLAMotionOperation/MLASpeedData.cpp
    MLAMotionOperation/MLAAccData.h
    MLAMotionOperation/MLAAccData.cpp

    MLAData/MLAData.h
    MLAData/MLAData.cpp

    MLAFilters/MLAFilters.h
    MLAFilters/MLAFilters.cpp
    MLAFilters/MLASavgol.h
    MLAFilters/MLASavgol.cpp

    MLAUtility/MLAUtility.h
    MLAUtility/MLAUtility.cpp
    MLAUtility/MLAUtility_impl.tcc

    MLADataExport/CSVExport.h
    MLADataExport/CSVExport.cpp
    MLADataExport/PipeExport.h
    MLADataExport/PipeExport.cpp
    MLADataExport/JsonExport.h
    MLADataExport/JsonExport.cpp)

SOURCE_GROUP("headers" FILES MLATools_lib.h MLACommonInclude.h)

SOURCE_GROUP("MLAShader\\headers" FILES MLAShader/MLAShader.h)
SOURCE_GROUP("MLAShader\\src" FILES MLAShader/MLAShader.cpp)

SOURCE_GROUP("MLAMotionOperation\\headers" FILES MLAMotionOperation/MLAMotionOperation.h)
SOURCE_GROUP("MLAMotionOperation\\src" FILES MLAMotionOperation/MLAMotionOperation.cpp)

SOURCE_GROUP("MLAData\\headers" FILES MLAData/MLAData.h)
SOURCE_GROUP("MLAData\\src" FILES MLAData/MLAData.cpp)

SOURCE_GROUP("MLAMotionOperation\\headers" FILES MLAMotionOperation/MLASpeedData.h)
SOURCE_GROUP("MLAMotionOperation\\src" FILES MLAMotionOperation/MLASpeedData.cpp)
SOURCE_GROUP("MLAMotionOperation\\headers" FILES MLAMotionOperation/MLAAccData.h)
SOURCE_GROUP("MLAMotionOperation\\src" FILES MLAMotionOperation/MLAAccData.cpp)

SOURCE_GROUP("MLAFilters\\headers" FILES MLAFilters/MLAFilters.h)
SOURCE_GROUP("MLAFilters\\src" FILES MLAFilters/MLAFilters.cpp)
SOURCE_GROUP("MLAFilters\\headers" FILES MLAFilters/MLASavgol.h)
SOURCE_GROUP("MLAFilters\\src" FILES MLAFilters/MLASavgol.cpp)

SOURCE_GROUP("MLAUtility\\headers" FILES MLAUtility/MLAUtility.h)
SOURCE_GROUP("MLAUtility\\headers" FILES MLAUtility/MLAUtility_impl.tcc)
SOURCE_GROUP("MLAUtility\\src" FILES MLAUtility/MLAUtility.cpp)

SOURCE_GROUP("MLADataExport\\headers" FILES MLADataExport/CSVExport.h)
SOURCE_GROUP("MLADataExport\\src" FILES MLADataExport/CSVExport.cpp)
SOURCE_GROUP("MLADataExport\\headers" FILES MLADataExport/PipeExport.h)
SOURCE_GROUP("MLADataExport\\src" FILES MLADataExport/PipeExport.cpp)
SOURCE_GROUP("MLADataExport\\headers" FILES MLADataExport/JsonExport.h)
SOURCE_GROUP("MLADataExport\\src" FILES MLADataExport/JsonExport.cpp)