cmake_minimum_required(VERSION 3.0)

project(RecastNavigation)

option(RECASTNAVIGATION_DEMO "Build demo" ON)
option(RECASTNAVIGATION_TESTS "Build tests" ON)
option(RECASTNAVIGATION_EXAMPLES "Build examples" ON)

add_subdirectory(DebugUtils)
add_subdirectory(Detour)
add_subdirectory(DetourCrowd)
add_subdirectory(DetourTileCache)
add_subdirectory(Recast)

if(RECASTNAVIGATION_DEMO)
    add_subdirectory(RecastDemo)
endif()

if(RECASTNAVIGATION_TESTS)
    enable_testing()
    add_subdirectory(Tests)
endif()
