
##---------------------------------------------------------------------------
## $RCSfile:        CMakeLists.txt $
## $Source:         CMakeLists.txt $
## $Revision:       1.00 $
## $Date:           Jan 27, 2017 3:41:00 PM $
##---------------------------------------------------------------------------
## Author:          Quentin Couland
## Subject;         This is the root Cmake file of MLA PROJECT
## Copyright:       (c) Jorgen Bodde
##---------------------------------------------------------------------------



# define minimum cmake version
cmake_minimum_required(VERSION 3.6.2)

# Our project is called 'MLA' this is how it will be called in
# visual studio, and in our makefiles. 
PROJECT(MLA)


# Create verbose makefiles if on.
SET(CMAKE_VERBOSE_MAKEFILE False)

# Set the target for generating (MSVC12 -> VS2013, MSVC15 -> VS2017)
SET(MSVC_VERSION 15)

##---------------------------------------------------------------------------
# Set build type
##---------------------------------------------------------------------------


# Specifies the build type for make based generators (Debug or Release)
SET(CMAKE_BUILD_TYPE Debug)


IF(NOT ((CMAKE_BUILD_TYPE STREQUAL "Debug") OR (CMAKE_BUILD_TYPE STREQUAL "Release")))
    MESSAGE("Error: build type invalid, please choose between Debug and Release mode")
ELSE(NOT ((CMAKE_BUILD_TYPE STREQUAL "Debug") OR (CMAKE_BUILD_TYPE STREQUAL "Release")))
    SET(TOUPPER_BUILD_TYPE ${CMAKE_BUILD_TYPE})
    string(TOUPPER ${TOUPPER_BUILD_TYPE} TOUPPER_BUILD_TYPE)
ENDIF(NOT ((CMAKE_BUILD_TYPE STREQUAL "Debug") OR (CMAKE_BUILD_TYPE STREQUAL "Release")))

##---------------------------------------------------------------------------
# Set directory path
##---------------------------------------------------------------------------


#Location where cmake first looks for cmake modules.
set(CMAKE_MODULE_PATH ${MLA_SOURCE_DIR})

#Path in which to build LIBRARY target files
SET(LIBRARY_OUTPUT_PATH ${MLA_SOURCE_DIR}/Lib)  

#Path in which to build RUNTIME target files
SET(ARCHIVE_OUTPUT_PATH     ${MLA_SOURCE_DIR}/Lib)

#Path in which to build RUNTIME target file
SET(EXECUTABLE_OUTPUT_PATH  ${MLA_SOURCE_DIR}/Bin)


#Directory in which to build RUNTIME target file for the current build mode
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${TOUPPER_BUILD_TYPE} ${MLA_SOURCE_DIR}/Bin )

#Directory in which to build LIBRARY target files the current build mode
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${TOUPPER_BUILD_TYPE} ${MLA_SOURCE_DIR}/Lib )

#Directory in which to build RUNTIME target files the current build mode
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${TOUPPER_BUILD_TYPE} ${MLA_SOURCE_DIR}/Lib )


##---------------------------------------------------------------------------
# Check OS
##---------------------------------------------------------------------------


IF(APPLE)
    
    SET(APPLE_OS 1)
    
    MESSAGE(STATUS "Platform: Mac")

ELSE(APPLE)
    
    SET(APPLE_OS 0)

ENDIF(APPLE)



IF((UNIX) AND NOT (APPLE))
    
    SET(LINUX_OS 1)
    
    MESSAGE(STATUS "Platform: Linux")
ELSE((UNIX) AND NOT (APPLE))
    
    SET(LINUX_OS 0)

ENDIF((UNIX) AND NOT (APPLE))


IF(WIN32)
    
    SET(WIN_OS 1)
    
    MESSAGE(STATUS "Platform: Windows")

ELSE(WIN32)
    
    SET(WIN_OS 0)

ENDIF(WIN32)



MESSAGE(STATUS "OS: " ${CMAKE_SYSTEM_NAME})



IF(NOT(LINUX_OS) AND NOT (APPLE_OS) AND NOT (WIN_OS))
    
    message(FATAL_ERROR "Error: OS not recognized, please reconfigure CmakeLists.txt")

ENDIF(NOT(LINUX_OS) AND NOT (APPLE_OS) AND NOT (WIN_OS))



##---------------------------------------------------------------------------
# Check/set Achitecture
##---------------------------------------------------------------------------



if((LINUX_OS) OR (APPLE_OS))
    
    message(FATAL_ERROR "Error: project not ready for Unix OS, please reconfigure CmakeLists.txt")

endif((LINUX_OS) OR (APPLE_OS))



IF(WIN_OS)

    # TODO: find how to do it now
    # IF (NOT(MSVC12) OR NOT(MSVC15))
    #     message(FATAL_ERROR "Error: project valid only for Visual studio 2013 or 2017 version, please reconfigure CmakeLists.txt" )
    # endif(NOT(MSVC12) OR NOT(MSVC15))
    

    IF (MSVC_VERSION EQUAL 12)
        SET(CMAKE_VERSION_VISUAL_STUDIO_NAME "Visual Studio 12 2013")
    ENDIF (MSVC_VERSION EQUAL 12)

    IF (MSVC_VERSION EQUAL 15)
        SET(CMAKE_VERSION_VISUAL_STUDIO_NAME "Visual Studio 15 2017")
    ENDIF (MSVC_VERSION EQUAL 15) 
    

    SET(WIN_IDE_ARCH 0)
    IF(CMAKE_GENERATOR STREQUAL "${CMAKE_VERSION_VISUAL_STUDIO_NAME} Win64")
        SET(CMAKE_ARCHITECTURE_TYPE x64)        
        SET(WIN_IDE_ARCH 1) 
    ENDIF(CMAKE_GENERATOR STREQUAL "${CMAKE_VERSION_VISUAL_STUDIO_NAME} Win64")
 

    IF(CMAKE_GENERATOR STREQUAL ${CMAKE_VERSION_VISUAL_STUDIO_NAME})
        SET(CMAKE_ARCHITECTURE_TYPE x86)        
        SET(WIN_IDE_ARCH 1)
    ENDIF(CMAKE_GENERATOR STREQUAL ${CMAKE_VERSION_VISUAL_STUDIO_NAME})


    IF(NOT (WIN_IDE_ARCH))
        
        message(FATAL_ERROR "Error: the following Visual Studio architecture is unknown: ${CMAKE_GENERATOR}")

    ELSE(NOT (WIN_IDE_ARCH))    
        IF(MSVC_VERSION EQUAL 12)
            MESSAGE(STATUS "Configuration: Visual Studio 2013")
        ENDIF(MSVC_VERSION EQUAL 12)

        IF(MSVC_VERSION EQUAL 15)
            MESSAGE(STATUS "Configuration: Visual Studio 2017")
        ENDIF(MSVC_VERSION EQUAL 15)

    ENDIF(NOT (WIN_IDE_ARCH))


ENDIF(WIN_OS)


# Manually specifies the build architecture: 32 (please type "x86") or 64 (please type "x64") bits.
SET(CMAKE_ARCHITECTURE_TYPE x86)


##---------------------------------------------------------------------------
# Common settings 
##---------------------------------------------------------------------------


    #-------------------------------------
    # OPENGL
    #-------------------------------------  
    find_package(OpenGL QUIET)

    IF(CMAKE_ARCHITECTURE_TYPE STREQUAL "x86" )

        IF (CMAKE_BUILD_TYPE STREQUAL "Debug")
            MESSAGE(STATUS "Architecture: Debug, 32 bits")
        ELSE(CMAKE_BUILD_TYPE STREQUAL "Debug")
            MESSAGE(STATUS "Architecture: Release, 32 bits")
        ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")

    ENDIF(CMAKE_ARCHITECTURE_TYPE STREQUAL "x86" )


    IF(CMAKE_ARCHITECTURE_TYPE STREQUAL "x64" )

        IF (CMAKE_BUILD_TYPE STREQUAL "Debug")
            MESSAGE(STATUS "Architecture: Debug, 64 bits")
        ELSE(CMAKE_BUILD_TYPE STREQUAL "Debug")
            MESSAGE(STATUS "Architecture: Release, 64 bits")
        ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")
    ENDIF(CMAKE_ARCHITECTURE_TYPE STREQUAL "x64" )

##---------------------------------------------------------------------------
# Special settings for Linux or Apple systems
##---------------------------------------------------------------------------

##---------------------------------------------------------------------------
# Special settings for Linux systems
##---------------------------------------------------------------------------

##---------------------------------------------------------------------------
# Special settings for Apple systems
##---------------------------------------------------------------------------

##---------------------------------------------------------------------------
# Special settings for Windows systems
##---------------------------------------------------------------------------
IF(WIN_OS)


    #-------------------------------------
    # OpenGL
    #-------------------------------------  
    if(NOT OPENGL_FOUND)
        message(FATAL_ERROR "Error OpenGL not found: please install OpenGL package")
    endif(NOT OPENGL_FOUND)
    SET(OPENGL_LIB ${OPENGL_LIBRARIES})


    #-------------------------------------
    # GLEW
    #-------------------------------------  
    SET(GLEW_INCLUDE_DIR        ${MLA_SOURCE_DIR}/ThirdParty/Windows/Glew/include)
    SET(GLEW_LIBRARIES_DIR      ${MLA_SOURCE_DIR}/ThirdParty/Windows/Glew/lib)


    #-------------------------------------
    # SDL
    #-------------------------------------                  
    SET(SDL_INCLUDE_DIR         ${MLA_SOURCE_DIR}/ThirdParty/Windows/SDL/include)
    SET(SDL_LIBRARIES_DIR       ${MLA_SOURCE_DIR}/ThirdParty/Windows/SDL/lib)


    #-------------------------------------
    # GLM
    #-------------------------------------                  
    SET(GLM_INCLUDE_DIR         ${MLA_SOURCE_DIR}/ThirdParty/CrossPlatform/glm)


    #-------------------------------------
    # NEURON SDK
    #-------------------------------------  
    SET(NEURON_INCLUDE_DIR         ${MLA_SOURCE_DIR}/ThirdParty/Windows/NeuronDataReaderSDK/include)
    SET(NEURON_LIBRARIES_DIR       ${MLA_SOURCE_DIR}/ThirdParty/Windows/NeuronDataReaderSDK/lib/x86)


    #-------------------------------------
    # JSON
    #-------------------------------------
    SET(JSON_INCLUDE_DIR         ${MLA_SOURCE_DIR}/ThirdParty/CrossPlatform/json)


    #-------------------------------------
    # LIBS
    #-------------------------------------  
    
    IF(CMAKE_ARCHITECTURE_TYPE STREQUAL "x86" )

        SET(GLEW_LIB        GLEW32_x86.lib
                    GLEW32s_x86.lib)

        SET(SDL_LIB         SDL2_x86.lib
                    SDL2main_x86.lib)

        SET(NEURON_LIB      NeuronDataReader.lib)

        SET(GLEW_DYNAMIC_LIB        ${MLA_SOURCE_DIR}/ThirdParty/Windows/Glew/dll/x86/glew32.dll)
        SET(SDL_DYNAMIC_LIB         ${MLA_SOURCE_DIR}/ThirdParty/Windows/SDL/dll/x86/SDL2.dll)
        SET(NEURON_DYNAMIC_LIB      ${MLA_SOURCE_DIR}/ThirdParty/Windows/NeuronDataReaderSDK/lib/x86/NeuronDataReader.dll)


        ADD_DEFINITIONS(-D_ALLOW_ITERATOR_DEBUG_LEVEL_MISMATCH)
        ADD_DEFINITIONS(/W4)       
    
    ENDIF(CMAKE_ARCHITECTURE_TYPE STREQUAL "x86" )

    
    IF(CMAKE_ARCHITECTURE_TYPE STREQUAL "x64" )

        SET(GLEW_LIB        GLEW32_x64.lib
                    GLEW32s_x64.lib)

        SET(SDL_LIB         SDL2_x64.lib
                    SDL2main_x64.lib)

        SET(NEURON_LIB      NeuronDataReader.lib)

        SET(GLEW_DYNAMIC_LIB        ${MLA_SOURCE_DIR}/ThirdParty/Windows/Glew/dll/x64/glew32.dll)
        SET(SDL_DYNAMIC_LIB         ${MLA_SOURCE_DIR}/ThirdParty/Windows/SDL/dll/x64/SDL2.dll)
        SET(NEURON_DYNAMIC_LIB      ${MLA_SOURCE_DIR}/ThirdParty/Windows/NeuronDataReaderSDK/lib/x64/NeuronDataReader.dll)

    ENDIF(CMAKE_ARCHITECTURE_TYPE STREQUAL "x64" )
    
    IF (CMAKE_BUILD_TYPE STREQUAL "Release")
        MESSAGE("Do not forget to set Visual Studio configuration to Release mode")
    ENDIF(CMAKE_BUILD_TYPE STREQUAL "Release")

ENDIF(WIN_OS)




# Add a list of subdirectories to the build
add_subdirectory(Source)