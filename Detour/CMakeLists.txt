project(Detour)
file(GLOB SOURCES Source/*.cpp)
add_library(${PROJECT_NAME} ${SOURCES})

add_library(RecastNavigation::Detour ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX -d)

set(Detour_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_compile_options(${PROJECT_NAME} PRIVATE
        $<$<NOT:$<CONFIG:Debug>>:
        $<$<CXX_COMPILER_ID:Clang,GNU>:-Ofast>
        $<$<CXX_COMPILER_ID:MSVC>:/Ox>
        >
)

if(RECASTNAVIGATION_DT_POLYREF64)
    target_compile_definitions(${PROJECT_NAME} PUBLIC DT_POLYREF64)
endif()
if(RECASTNAVIGATION_DT_VIRTUAL_QUERYFILTER)
    target_compile_definitions(${PROJECT_NAME} PUBLIC DT_VIRTUAL_QUERYFILTER)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${Detour_INCLUDE_DIR}>"
)

set_target_properties(${PROJECT_NAME} PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${LIB_VERSION}
        COMPILE_PDB_OUTPUT_DIRECTORY .
        COMPILE_PDB_NAME "Detour-d"
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
    install(FILES "$<TARGET_FILE_DIR:Detour>/Detour-d.pdb" CONFIGURATIONS "Debug" DESTINATION "lib" OPTIONAL)
endif()
