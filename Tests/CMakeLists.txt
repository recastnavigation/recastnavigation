include_directories(../DebugUtils/Include)
include_directories(../Detour/Include)
include_directories(../Recast/Include)
include_directories(../RecastCLI/Include)

file(GLOB EXTERNAL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../RecastCLI/Source/*.cpp")
set(FILE_TO_REMOVE "${CMAKE_CURRENT_SOURCE_DIR}/../RecastCLI/Source/main.cpp")
list(REMOVE_ITEM EXTERNAL_SOURCES ${FILE_TO_REMOVE})

add_executable(Tests ${EXTERNAL_SOURCES}
        Detour/Tests_Detour.cpp
        DetourCrowd/Tests_DetourPathCorridor.cpp
        Recast/Bench_rcVector.cpp
        Recast/Tests_Alloc.cpp
        Recast/Tests_Recast.cpp
        Recast/Tests_RecastFilter.cpp
        RecastLCM/Tests_Recast_LCM.h
        RecastLCM/Tests_Recast_LCM_2D.cpp
        RecastLCM/Tests_Recast_LCM_3D.cpp
)

set_property(TARGET Tests PROPERTY CXX_STANDARD 17)

add_dependencies(Tests Recast Detour DetourCrowd)
target_link_libraries(Tests DebugUtils Recast Detour DetourCrowd)

find_package(Catch2 QUIET)
if (Catch2_FOUND)
    target_link_libraries(Tests Catch2::Catch2WithMain)
else ()
    include_directories(./Contrib)
    target_sources(Tests PRIVATE Contrib/catch2/catch_amalgamated.cpp)
endif ()

add_test(Tests Tests)
