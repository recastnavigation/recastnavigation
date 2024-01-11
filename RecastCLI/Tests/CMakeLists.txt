# TEST SETUP
file(GLOB TESTS_SOURCES ../Source/*.cpp ../Contrib/fastlz/fastlz.c Tests_Recast.cpp)
set(FILE_TO_REMOVE "${CMAKE_CURRENT_SOURCE_DIR}/../Source/main.cpp")
list(REMOVE_ITEM TESTS_SOURCES ${FILE_TO_REMOVE})
add_executable(CLITests ${TESTS_SOURCES})

set_property(TARGET CLITests PROPERTY CXX_STANDARD 20)

add_dependencies(CLITests  DebugUtils Detour DetourCrowd DetourTileCache Recast)
target_link_libraries(CLITests  DebugUtils Detour DetourCrowd DetourTileCache Recast)

find_package(Catch2 QUIET)
if (Catch2_FOUND)
    target_link_libraries(CLITests Catch2::Catch2WithMain)
else()
    include_directories(../Contrib)
    target_sources(CLITests PRIVATE ../Contrib/catch2/catch_amalgamated.cpp)
endif()

add_test(CLITests CLITests)
