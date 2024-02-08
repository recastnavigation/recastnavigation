project(RecastCLI)
file(GLOB SOURCES Source/*.cpp)




add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(RecastCLI
        DebugUtils
        DetourTileCache
        Recast
)

# Use target_include_directories instead of include_directories
target_include_directories(${PROJECT_NAME} PRIVATE
        ../DebugUtils/Include
        ../Recast/Include
        Include
)

# Link libraries using target_link_libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
        DebugUtils
        Recast
)

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