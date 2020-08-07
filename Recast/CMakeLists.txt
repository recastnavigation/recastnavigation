file(GLOB SOURCES Source/*.cpp)
add_library(Recast ${SOURCES})

add_library(RecastNavigation::Recast ALIAS Recast)
set_target_properties(Recast PROPERTIES DEBUG_POSTFIX -d)

set(Recast_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(Recast PUBLIC
    "$<BUILD_INTERFACE:${Recast_INCLUDE_DIR}>"
)

set_target_properties(Recast PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${VERSION}
        COMPILE_PDB_OUTPUT_DIRECTORY .
        COMPILE_PDB_NAME "Recast-d"
        )

install(TARGETS Recast
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT library
        )

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation)
install(FILES "$<TARGET_FILE_DIR:Recast>/Recast-d.pdb" CONFIGURATIONS "Debug" DESTINATION "lib") 
