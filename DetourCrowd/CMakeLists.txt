add_library(DetourCrowd)
add_library(RecastNavigation::DetourCrowd ALIAS DetourCrowd)

set_target_properties(DetourCrowd PROPERTIES
    DEBUG_POSTFIX -d
    SOVERSION ${SOVERSION}
    VERSION ${LIB_VERSION}
    COMPILE_PDB_OUTPUT_DIRECTORY .
    COMPILE_PDB_NAME "DetourCrowd-d"
    CXX_STANDARD 98
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF  # Disable compiler-specific extensions
)

target_include_directories(DetourCrowd PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation>"
)

target_sources(DetourCrowd PRIVATE
    Source/DetourCrowd.cpp
    Source/DetourLocalBoundary.cpp
    Source/DetourObstacleAvoidance.cpp
    Source/DetourPathCorridor.cpp
    Source/DetourPathQueue.cpp
    Source/DetourProximityGrid.cpp
)

target_link_libraries(DetourCrowd Detour)

install(TARGETS DetourCrowd
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
    install(FILES "$<TARGET_FILE_DIR:DetourCrowd>/DetourCrowd-d.pdb"
        CONFIGURATIONS "Debug"
        DESTINATION "lib"
        OPTIONAL)
endif()
