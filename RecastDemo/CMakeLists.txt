file(GLOB SOURCES Source/*.cpp Contrib/fastlz/fastlz.c)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(NOT SDL2_ROOT_DIR)
    set(SDL2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Contrib/SDL")
endif()

find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(SYSTEM ${OPENGL_INCLUDE_DIR})
include_directories(SYSTEM Contrib/fastlz)
include_directories(SYSTEM Contrib)
include_directories(../DebugUtils/Include)
include_directories(../Detour/Include)
include_directories(../DetourCrowd/Include)
include_directories(../DetourTileCache/Include)
include_directories(../Recast/Include)
include_directories(Include)

if (WIN32)
    add_executable(RecastDemo WIN32 ${SOURCES})
elseif(APPLE)
    add_executable(RecastDemo MACOSX_BUNDLE ${SOURCES})
else()
    add_executable(RecastDemo ${SOURCES})
endif()

file(COPY Bin/Meshes DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY Bin/TestCases DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY Bin/DroidSans.ttf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

if (WIN32)
    file(COPY "${SDL2_RUNTIME_LIBRARY}" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

add_dependencies(RecastDemo DebugUtils Detour DetourCrowd DetourTileCache Recast)
target_link_libraries(RecastDemo ${OPENGL_LIBRARIES} SDL2::SDL2main DebugUtils Detour DetourCrowd DetourTileCache Recast)

install(TARGETS RecastDemo
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin)
install(DIRECTORY Bin/Meshes DESTINATION bin)
install(DIRECTORY Bin/TestCases DESTINATION bin)
install(FILES Bin/DroidSans.ttf DESTINATION bin)

if (WIN32)
    install(FILES "${SDL2_RUNTIME_LIBRARY}" DESTINATION bin)
endif()