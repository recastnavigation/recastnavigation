file(GLOB SOURCES Source/*.cpp Contrib/fastlz/fastlz.c)

include(cmake/FindSDL2.cmake)

find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(SYSTEM ${OPENGL_INCLUDE_DIR})
include_directories(SYSTEM ${SDL2_INCLUDE_DIRS})
include_directories(SYSTEM Contrib/fastlz)
include_directories(SYSTEM Contrib)
include_directories(../DebugUtils/Include)
include_directories(../Detour/Include)
include_directories(../DetourCrowd/Include)
include_directories(../DetourTileCache/Include)
include_directories(../Recast/Include)
include_directories(Include)

add_executable(RecastDemo ${SOURCES})
file(COPY Bin/Meshes DESTINATION ${CMAKE_BINARY_DIR}/RecastDemo)
file(COPY Bin/TestCases DESTINATION ${CMAKE_BINARY_DIR}/RecastDemo)
file(COPY Bin/DroidSans.ttf DESTINATION ${CMAKE_BINARY_DIR}/RecastDemo)

add_dependencies(RecastDemo DebugUtils Detour DetourCrowd DetourTileCache Recast)
target_link_libraries(RecastDemo ${OPENGL_LIBRARIES} ${SDL2_LIBRARY} DebugUtils Detour DetourCrowd DetourTileCache Recast)

install(TARGETS RecastDemo RUNTIME DESTINATION bin)
