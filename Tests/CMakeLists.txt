project(Tests)
file(GLOB TESTS_SOURCES Detour/*.cpp Recast/*.cpp)
file(GLOB EXTERNAL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../RecastCLI/Source/*.cpp")
set(FILE_TO_REMOVE "${CMAKE_CURRENT_SOURCE_DIR}/../RecastCLI/Source/main.cpp")
list(REMOVE_ITEM EXTERNAL_SOURCES ${FILE_TO_REMOVE})

add_executable(${PROJECT_NAME} ${TESTS_SOURCES} ${EXTERNAL_SOURCES}
	Detour/Tests_Detour.cpp
	Recast/Bench_rcVector.cpp
	Recast/Tests_Alloc.cpp
	Recast/Tests_Recast.cpp
	Recast/Tests_RecastFilter.cpp
	DetourCrowd/Tests_DetourPathCorridor.cpp
)
add_dependencies(${PROJECT_NAME}
        Recast
        Detour
        DebugUtils
	   DetourCrowd
)
# Use target_include_directories instead of include_directories
target_include_directories(${PROJECT_NAME} PRIVATE
        ../DebugUtils/Include
        ../Detour/Include
        ../Recast/Include
        ../RecastCLI/Include
)

# Link libraries using target_link_libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
        DebugUtils
        Detour
        Recast
        DetourCrowd
)

# Use FetchContent module to handle Catch2 dependency
include(FetchContent)
FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.2
)
FetchContent_MakeAvailable(Catch2)

target_link_libraries(${PROJECT_NAME} PRIVATE
        Catch2::Catch2WithMain
)

# Use Catch2's own functions to declare tests
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(${PROJECT_NAME})

# Copy meshes directory to build directory
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/Bin
        ${CMAKE_CURRENT_BINARY_DIR}
)

# Install targets and directories
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin
)
install(DIRECTORY Meshes DESTINATION bin)