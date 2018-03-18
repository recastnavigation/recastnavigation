file(GLOB SOURCES Source/*.cpp)

include_directories(../Detour/Include)
include_directories(Include)

if (RECASTNAVIGATION_STATIC)
    add_library(DetourTileCache STATIC ${SOURCES})
else ()
    add_library(DetourTileCache SHARED ${SOURCES})
endif ()

set_target_properties(DetourTileCache PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${VERSION}
        )


install(TARGETS DetourTileCache
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library
        )

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION include)
