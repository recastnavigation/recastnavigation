file(GLOB SOURCES Source/*.cpp)

include_directories(../Recast/Include)
include_directories(../Detour/Include)
include_directories(../DetourTileCache/Include)
include_directories(Include)

if (RECASTNAVIGATION_STATIC)
    add_library(DebugUtils STATIC ${SOURCES})
else()
    add_library(DebugUtils SHARED ${SOURCES})
endif()

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
