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

# Set C++ standard directly on the target
target_compile_features(
        ${PROJECT_NAME} PRIVATE
        cxx_std_20
)

# Copy meshes directory to build directory
if (WIN32)
    if ("${CMAKE_MAKE_PROGRAM}" MATCHES "MSBuild")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes
                ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/$(ConfigurationName)/Meshes)
    elseif (MINGW)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes
                ${CMAKE_BINARY_DIR}/${PROJECT_NAME}/Meshes)
    endif ()
else ()
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes
            ${CMAKE_CURRENT_BINARY_DIR}/Meshes)
endif ()

# Install targets and directories
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin
)
install(DIRECTORY Meshes DESTINATION bin)