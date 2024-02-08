project(RecastDemo)
file(GLOB SOURCES Source/*.cpp Contrib/fastlz/fastlz.c)
add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} DebugUtils Detour DetourCrowd DetourTileCache Recast)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

target_include_directories(${PROJECT_NAME} PRIVATE
        ../DebugUtils/Include
        ../Detour/Include
        ../DetourCrowd/Include
        ../DetourTileCache/Include
        ../Recast/Include
        Include
)

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
         ${OPENGL_INCLUDE_DIR}
        Contrib/fastlz
        Contrib
)

find_package(OpenGL REQUIRED)

include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
        SDL2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL
        GIT_TAG release-2.30.0
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
)
set(BUILD_SHARED_LIBS CASH BOOL ON)
FetchContent_MakeAvailable(SDL2)

add_dependencies(${PROJECT_NAME} SDL2::SDL2main)
target_include_directories(${PROJECT_NAME} PRIVATE
        ${SDL2_BINARY_DIR}/include-config-$<LOWER_CASE:$<CONFIG>>/SDL2
        ${SDL2_BINARY_DIR}/include/SDL2
        ${SDL2_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        ${OPENGL_LIBRARIES}
        DebugUtils
        Detour
        DetourCrowd
        DetourTileCache
        Recast

        SDL2::SDL2main
        SDL2::SDL2
)


add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:SDL2::SDL2>
        ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/Bin
        ${CMAKE_CURRENT_BINARY_DIR}
)

install(TARGETS RecastDemo
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin)
install(DIRECTORY Bin/Meshes DESTINATION bin)
install(DIRECTORY Bin/TestCases DESTINATION bin)
install(FILES Bin/DroidSans.ttf DESTINATION bin)