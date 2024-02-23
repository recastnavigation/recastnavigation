project(DetourTileCache)
file(GLOB SOURCES Source/*.cpp)
add_library(${PROJECT_NAME} ${SOURCES})

add_library(RecastNavigation::DetourTileCache ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX -d)

set(DetourTileCache_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_compile_options(${PROJECT_NAME} PRIVATE
        $<$<NOT:$<CONFIG:Debug>>:
        $<$<CXX_COMPILER_ID:Clang,GNU>:-Ofast>
        $<$<CXX_COMPILER_ID:MSVC>:/Ox>
        >
)

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${DetourTileCache_INCLUDE_DIR}>"
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Detour
)

set_target_properties(${PROJECT_NAME} PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${LIB_VERSION}
        COMPILE_PDB_OUTPUT_DIRECTORY .
        COMPILE_PDB_NAME "DetourTileCache-d"
        )


install(TARGETS ${PROJECT_NAME}
        EXPORT recastnavigation-targets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT library
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation
        )

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation)
if(MSVC)
    install(FILES "$<TARGET_FILE_DIR:DetourTileCache>/DetourTileCache-d.pdb" CONFIGURATIONS "Debug" DESTINATION "lib" OPTIONAL)
endif()
