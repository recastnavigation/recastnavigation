file(GLOB SOURCES Source/*.cpp)

include_directories(Include)

if (RECASTNAVIGATION_STATIC)
    add_library(Recast STATIC ${SOURCES})
else ()
    add_library(Recast SHARED ${SOURCES})
endif ()

set_target_properties(Recast PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${VERSION}
        )

install(TARGETS Recast
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library
        )

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION include)
