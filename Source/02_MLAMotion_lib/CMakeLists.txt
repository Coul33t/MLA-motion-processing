PROJECT(02_MLAMotion_lib)

INCLUDE_DIRECTORIES(${01_MLATools_lib_SOURCE_DIR}
		    ${GLM_INCLUDE_DIR}
 		    ${GLEW_INCLUDE_DIR}
		    ${SDL_INCLUDE_DIR}
)

ADD_LIBRARY(02_MLAMotion_lib STATIC
		MLAMotion_lib.h
	
		MLAMotion/MLAMotion.h
        MLAMotion/MLAMotion.cpp

        MLAMotion/MLAFrame.h
        MLAMotion/MLAFrame.cpp

        MLAMotion/MLAJoint.h
        MLAMotion/MLAJoint.cpp	
)

SOURCE_GROUP("Fichiers d'en-tete" FILES MLAMotion_lib.h)

SOURCE_GROUP("MLAMotion\\Fichiers d'en-tete"    FILES MLAMotion/MLAMotion.h)
SOURCE_GROUP("MLAMotion\\Fichiers sources"      FILES MLAMotion/MLAMotion.cpp)

SOURCE_GROUP("MLAMotion\\Fichiers d'en-tete"    FILES MLAMotion/MLAFrame.h)
SOURCE_GROUP("MLAMotion\\Fichiers sources"      FILES MLAMotion/MLAFrame.cpp)

SOURCE_GROUP("MLAMotion\\Fichiers d'en-tete"    FILES MLAMotion/MLAJoint.h)
SOURCE_GROUP("MLAMotion\\Fichiers sources"      FILES MLAMotion/MLAJoint.cpp)



