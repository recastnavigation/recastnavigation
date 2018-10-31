file(GLOB SOURCES Source/*.cpp)

if (RECASTNAVIGATION_STATIC)
    add_library(Recast STATIC ${SOURCES})
else ()
    add_library(Recast SHARED ${SOURCES})
endif ()

add_library(RecastNavigation::Recast ALIAS Recast)

set(Recast_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(Recast PUBLIC
    "$<BUILD_INTERFACE:${Recast_INCLUDE_DIR}>"
)

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
