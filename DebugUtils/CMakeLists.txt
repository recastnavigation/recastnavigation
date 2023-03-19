file(GLOB SOURCES Source/*.cpp)
add_library(DebugUtils ${SOURCES})

add_library(RecastNavigation::DebugUtils ALIAS DebugUtils)
set_target_properties(DebugUtils PROPERTIES DEBUG_POSTFIX -d)

set(DebugUtils_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(DebugUtils PUBLIC
    "$<BUILD_INTERFACE:${DebugUtils_INCLUDE_DIR}>"
)

target_link_libraries(DebugUtils
    Recast
    Detour
    DetourTileCache
)

set_target_properties(DebugUtils PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${LIB_VERSION}
        COMPILE_PDB_OUTPUT_DIRECTORY .
        COMPILE_PDB_NAME "DebugUtils-d"
        )

install(TARGETS DebugUtils
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
    install(FILES "$<TARGET_FILE_DIR:DebugUtils>/DebugUtils-d.pdb" CONFIGURATIONS "Debug" DESTINATION "lib" OPTIONAL)
endif()
