include_directories(../Detour/Include)
include_directories(../Recast/Include)

add_executable(Tests
	Detour/Tests_Detour.cpp
	Recast/Bench_rcVector.cpp
	Recast/Tests_Alloc.cpp
	Recast/Tests_Recast.cpp
	Recast/Tests_RecastFilter.cpp
	DetourCrowd/Tests_DetourPathCorridor.cpp
)

set_property(TARGET Tests PROPERTY CXX_STANDARD 17)

add_dependencies(Tests Recast Detour DetourCrowd)
target_link_libraries(Tests Recast Detour DetourCrowd)

find_package(Catch2 QUIET)
if (Catch2_FOUND)
	target_link_libraries(Tests Catch2::Catch2WithMain)
else()
	include_directories(./Contrib)
	target_sources(Tests PRIVATE Contrib/catch2/catch_amalgamated.cpp)
endif()

add_test(Tests Tests)
