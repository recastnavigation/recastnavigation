add_library(DebugUtils)
add_library(RecastNavigation::DebugUtils ALIAS DebugUtils)

set_target_properties(DebugUtils PROPERTIES
    DEBUG_POSTFIX -d
    SOVERSION ${SOVERSION}
    VERSION ${LIB_VERSION}
    COMPILE_PDB_OUTPUT_DIRECTORY .
    COMPILE_PDB_NAME "DebugUtils-d"
    CXX_STANDARD 98
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF  # Disable compiler-specific extensions
)

target_include_directories(DebugUtils PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation>"
)

target_sources(DebugUtils PRIVATE
    Source/DebugDraw.cpp
    Source/DetourDebugDraw.cpp
    Source/RecastDebugDraw.cpp
    Source/RecastDump.cpp
)

target_link_libraries(DebugUtils Recast Detour DetourTileCache)

install(TARGETS DebugUtils
    EXPORT recastnavigation-targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT library
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation
)

install(DIRECTORY Include/ 
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation
    FILES_MATCHING PATTERN "*.h"
)

if (MSVC)
    install(FILES "$<TARGET_FILE_DIR:DebugUtils>/DebugUtils-d.pdb" CONFIGURATIONS "Debug" DESTINATION "lib" OPTIONAL)
endif()
