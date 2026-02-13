
add_library(Recast)
add_library(RecastNavigation::Recast ALIAS Recast)

set_target_properties(Recast PROPERTIES
    DEBUG_POSTFIX -d
    SOVERSION ${SOVERSION}
    VERSION ${LIB_VERSION}
    COMPILE_PDB_OUTPUT_DIRECTORY .
    COMPILE_PDB_NAME "Recast-d"
    CXX_STANDARD 98
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF  # Disable compiler-specific extensions
)

target_compile_definitions(Recast PUBLIC "$<$<NOT:${RECASTNAVIGATION_ENABLE_ASSERTS}>:RC_DISABLE_ASSERTS>")

target_include_directories(Recast PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation>"
)

target_sources(Recast PRIVATE
    Source/Recast.cpp
    Source/RecastAlloc.cpp
    Source/RecastArea.cpp
    Source/RecastAssert.cpp
    Source/RecastContour.cpp
    Source/RecastFilter.cpp
    Source/RecastLayers.cpp
    Source/RecastMesh.cpp
    Source/RecastMeshDetail.cpp
    Source/RecastRasterization.cpp
    Source/RecastRegion.cpp
)

install(TARGETS Recast
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
    install(FILES "$<TARGET_FILE_DIR:Recast>/Recast-d.pdb"
        CONFIGURATIONS "Debug"
        DESTINATION "lib"
        OPTIONAL)
endif()
