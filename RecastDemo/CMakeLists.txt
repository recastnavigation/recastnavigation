project(RecastDemo)
file(GLOB SOURCES Source/*.cpp Contrib/fastlz/fastlz.c)
add_executable(RecastDemo ${SOURCES})
add_dependencies(RecastDemo DebugUtils Detour DetourCrowd DetourTileCache Recast)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

target_include_directories(${PROJECT_NAME} PRIVATE
    SYSTEM ${OPENGL_INCLUDE_DIR}
    SYSTEM Contrib/fastlz
    SYSTEM Contrib
    ../DebugUtils/Include
    ../Detour/Include
    ../DetourCrowd/Include
    ../DetourTileCache/Include
    ../Recast/Include
    Include
    Contrib/SDL/include
    $<$<PLATFORM_ID:Darwin>:${SDL2_LIBRARY}/Headers>
)
target_compile_features(
        ${PROJECT_NAME} PRIVATE
        cxx_std_20
)

include(FetchContent)
FetchContent_Declare(
        SDL2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL
        GIT_TAG release-2.30.0
)
FetchContent_MakeAvailable(SDL2)

target_link_libraries(
        ${PROJECT_NAME} PRIVATE
        ${OPENGL_LIBRARIES}
        DebugUtils
        Detour
        DetourCrowd
        DetourTileCache
        Recast

        $<$<PLATFORM_ID:Darwin>:${SDL2_LIBRARY}>
        $<$<PLATFORM_ID:Windows>:SDL2::SDL2main>
)


if(WIN32)
    if("${CMAKE_MAKE_PROGRAM}" MATCHES "MSBuild")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy "${SDL2_RUNTIME_LIBRARY}" ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/$(ConfigurationName)/
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/$(ConfigurationName)/Meshes
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/$(ConfigurationName)/TestCases
            COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/$(ConfigurationName)/)
    elseif(MINGW)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy "${SDL2_RUNTIME_LIBRARY}" ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/Meshes
            COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/TestCases
            COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/)
    endif()
else()
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes ${CMAKE_CURRENT_BINARY_DIR}/Meshes
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases ${CMAKE_CURRENT_BINARY_DIR}/TestCases
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf ${CMAKE_CURRENT_BINARY_DIR}/)
endif()

install(TARGETS RecastDemo
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin)
install(DIRECTORY Bin/Meshes DESTINATION bin)
install(DIRECTORY Bin/TestCases DESTINATION bin)
install(FILES Bin/DroidSans.ttf DESTINATION bin)

if (WIN32)
    install(FILES "${SDL2_RUNTIME_LIBRARY}" DESTINATION bin)
endif()