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

set_target_properties(Tests PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF  # Disable compiler-specific extensions
)

# Remove warnings-as-errors for third-party code
if(MSVC)
	target_compile_options(Tests PRIVATE /W3 /WX-)
else()
	target_compile_options(Tests PRIVATE -Wno-error)
endif()

add_dependencies(Tests Recast Detour DetourCrowd)
target_link_libraries(Tests Recast Detour DetourCrowd)

# Use bundled catch2 rather than a possibly incompatible version installed elsewhere.
include_directories(./Contrib)
target_sources(Tests PRIVATE Contrib/catch2/catch_amalgamated.cpp)

add_test(Tests Tests)
