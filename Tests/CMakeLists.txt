add_executable(Tests)

set_target_properties(Tests PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF  # Disable compiler-specific extensions
)

# Remove warnings-as-errors
if(MSVC)
	target_compile_options(Tests PRIVATE /W3 /WX-)
else()
	target_compile_options(Tests PRIVATE -Wno-error)
endif()

target_include_directories(Tests PRIVATE ./Contrib)

target_sources(Tests PRIVATE 
	Contrib/catch2/catch_amalgamated.cpp
	Detour/Tests_Detour.cpp
	DetourCrowd/Tests_DetourPathCorridor.cpp
	Recast/Tests_Alloc.cpp
	Recast/Tests_Recast.cpp
	Recast/Tests_RecastFilter.cpp
)

target_link_libraries(Tests PRIVATE Recast Detour DetourCrowd)

add_test(NAME Tests COMMAND Tests)
