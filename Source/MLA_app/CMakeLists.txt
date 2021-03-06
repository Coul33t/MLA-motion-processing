project(MLA_app)
    INCLUDE_DIRECTORIES(
        ${MLA_SOURCE_DIR}/Source
        ${01_MLATools_lib_SOURCE_DIR}
        ${02_MLAMotion_lib_SOURCE_DIR}
		${03_MLAParser_lib_SOURCE_DIR}
        ${04_MLAGui_lib_SOURCE_DIR}
        ${05_MLANeuron_lib_SOURCE_DIR}
        ${06_MLARender_lib_SOURCE_DIR}
		${SDL_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIR}
        ${GLM_INCLUDE_DIR}
        ${NEURON_INCLUDE_DIR}
        ${JSON_INCLUDE_DIR}
    )

    LINK_DIRECTORIES(
        ${LIBRARY_OUTPUT_PATH}
        ${SDL_LIBRARIES_DIR}
        ${GLEW_LIBRARIES_DIR}
        ${NEURON_LIBRARIES_DIR}
    )



    #Create exe
    add_executable(MLA_app
        ../MLA.h
        MLA_app.cpp
    )

    SOURCE_GROUP("headers" REGULAR_EXPRESSION "[.]h$")
    SOURCE_GROUP("src" REGULAR_EXPRESSION "[.]cpp$")


    IF(WIN_OS)

        TARGET_LINK_LIBRARIES(MLA_app
            06_MLARender_lib.lib
            05_MLANeuron_lib.lib
            04_MLAGui_lib.lib
			03_MLAParser_lib.lib
            02_MLAMotion_lib.lib
            01_MLATools_lib.lib
            ${SDL_LIB}
            ${GLEW_LIB}
            ${OPENGL_LIB}
            ${NEURON_LIB}
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


        #-------------------------------------
        # Neuron
        #-------------------------------------
        add_custom_command(TARGET MLA_app   POST_BUILD      # Adds a post-build event to MyTest
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different   # which executes "cmake - E copy_if_different..."
                    ${NEURON_DYNAMIC_LIB}             # <--this is in-file
                    ${EXECUTABLE_OUTPUT_PATH})          # <--this is out-file path

    ENDIF(WIN_OS)




    IF(LINUX_OS)


        TARGET_LINK_LIBRARIES(MLA_app
                06_MLARender_lib.a
                05_MLANeuron_lib.a
                04_MLAGui_lib.a
				03_MLAParser_lib.a
                02_MLAMotion_lib.a
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
                06_MLARender_lib.a
                05_MLANeuron_lib.a
                04_MLAGui_lib.a
				03_MLAParser_lib.a
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

