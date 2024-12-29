file(GLOB SOURCES Source/*.cpp)
add_library(Detour ${SOURCES})

add_library(RecastNavigation::Detour ALIAS Detour)
set_target_properties(Detour PROPERTIES DEBUG_POSTFIX -d)

set(Detour_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

if(RECASTNAVIGATION_DT_POLYREF64)
    target_compile_definitions(Detour PUBLIC DT_POLYREF64)
endif()
if(RECASTNAVIGATION_DT_VIRTUAL_QUERYFILTER)
    target_compile_definitions(Detour PUBLIC DT_VIRTUAL_QUERYFILTER)
endif()

if(NOT RECASTNAVIGATION_ENABLE_ASSERTS)
    target_compile_definitions(Detour PUBLIC RC_DISABLE_ASSERTS)
endif()

target_include_directories(Detour PUBLIC
    "$<BUILD_INTERFACE:${Detour_INCLUDE_DIR}>"
)

set_target_properties(Detour PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${LIB_VERSION}
        COMPILE_PDB_OUTPUT_DIRECTORY .
        COMPILE_PDB_NAME "Detour-d"
        )

install(TARGETS Detour
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
    install(FILES "$<TARGET_FILE_DIR:Detour>/Detour-d.pdb" CONFIGURATIONS "Debug" DESTINATION "lib" OPTIONAL)
endif()
