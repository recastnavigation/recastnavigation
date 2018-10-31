file(GLOB SOURCES Source/*.cpp)

if (RECASTNAVIGATION_STATIC)
    add_library(DebugUtils STATIC ${SOURCES})
else()
    add_library(DebugUtils SHARED ${SOURCES})
endif()

add_library(RecastNavigation::DebugUtils ALIAS DebugUtils)

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
        VERSION ${VERSION}
        )

install(TARGETS DebugUtils
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library
        )

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION include)
