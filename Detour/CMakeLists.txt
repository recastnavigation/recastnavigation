add_library(Detour)
add_library(RecastNavigation::Detour ALIAS Detour)

set_target_properties(Detour PROPERTIES
    DEBUG_POSTFIX -d
    SOVERSION ${SOVERSION}
    VERSION ${LIB_VERSION}
    COMPILE_PDB_OUTPUT_DIRECTORY .
    COMPILE_PDB_NAME "Detour-d"
    CXX_STANDARD 98
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF  # Disable compiler-specific extensions
)

# Set build options
if (RECASTNAVIGATION_DT_POLYREF64)
    target_compile_definitions(Detour PUBLIC DT_POLYREF64)
endif()

if (RECASTNAVIGATION_DT_VIRTUAL_QUERYFILTER)
    target_compile_definitions(Detour PUBLIC DT_VIRTUAL_QUERYFILTER)
endif()

target_compile_definitions(Detour PUBLIC "$<$<NOT:${RECASTNAVIGATION_ENABLE_ASSERTS}>:RC_DISABLE_ASSERTS>")

target_include_directories(Detour PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation>"
)

target_sources(Detour PRIVATE
    Source/DetourAlloc.cpp
    Source/DetourAssert.cpp
    Source/DetourCommon.cpp
    Source/DetourNavMesh.cpp
    Source/DetourNavMeshBuilder.cpp
    Source/DetourNavMeshQuery.cpp
    Source/DetourNode.cpp
)

install(TARGETS Detour
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

if(MSVC)
    install(FILES "$<TARGET_FILE_DIR:Detour>/Detour-d.pdb"
        CONFIGURATIONS "Debug"
        DESTINATION "lib"
        OPTIONAL)
endif()
