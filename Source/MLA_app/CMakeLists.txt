


project(MLA_app)
    INCLUDE_DIRECTORIES(
        ${MLA_SOURCE_DIR}/Source
        ${01_MLATools_lib_SOURCE_DIR}
            ${02_MLAMotion_lib_SOURCE_DIR}  
        ${SDL_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIR}
        ${GLM_INCLUDE_DIR}
    )

    LINK_DIRECTORIES(
         ${LIBRARY_OUTPUT_PATH}
         ${SDL_LIBRARIES_DIR}
         ${GLEW_LIBRARIES_DIR}
    )



    #Create exe 
    add_executable(MLA_app
        ../MLA.h
        MLA_app.cpp
    )

    SOURCE_GROUP("Fichiers d'en-tete" REGULAR_EXPRESSION "[.]h$")
    SOURCE_GROUP("Fichiers sources" REGULAR_EXPRESSION "[.]cpp$")


    IF(WIN_OS)

    
        TARGET_LINK_LIBRARIES(MLA_app
            02_MLAMotion_lib.lib
            01_MLATools_lib.lib 
            ${SDL_LIB}
            ${GLEW_LIB}
            ${OPENGL_LIB} 
        )
    
        #-------------------------------------
        # SDL
        #-------------------------------------

    
        add_custom_command(TARGET MLA_app   POST_BUILD      # Adds a post-build event to MyTest
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different   # which executes "cmake - E copy_if_different..."
                    ${SDL_DYNAMIC_LIB}              # <--this is in-file
                    ${EXECUTABLE_OUTPUT_PATH})          # <--this is out-file path

        #-------------------------------------
        # Glew
        #-------------------------------------
        add_custom_command(TARGET MLA_app   POST_BUILD      # Adds a post-build event to MyTest
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different   # which executes "cmake - E copy_if_different..."
                    ${GLEW_DYNAMIC_LIB}             # <--this is in-file
                    ${EXECUTABLE_OUTPUT_PATH})          # <--this is out-file path


    ENDIF(WIN_OS)



                    
    IF(LINUX_OS)

    
        TARGET_LINK_LIBRARIES(MLA_app
                02_MLAlaMotion_lib.a
                01_MLATools_lib.a
                ${SDL_LIB}
                ${GLEW_LIB}
                ${OPENGL_LIB}
                ${THREAD_LIBRARY}
                ${CMAKE_DL_LIBS}                    
        )

    ENDIF(LINUX_OS)


    IF(APPLE_OS)
    
        TARGET_LINK_LIBRARIES(MLA_app
                02_MLAMotion_lib.a
                01_MLATools_lib.a
                ${SDL_LIB}
                ${GLEW_LIB}
                ${OPENGL_LIB}
                ${THREAD_LIBRARY}
                ${CMAKE_DL_LIBS}
                "-framework Opengl"
                "-framework Cocoa"
                "-framework Carbon"
                "-framework IOKit"
                ${ICONV_LIBRARY} 
        )

    ENDIF(APPLE_OS)

