file(GLOB SOURCES Source/*.cpp)

if(RECASTNAVIGATION_STATIC)
    add_library(Detour STATIC ${SOURCES})
else()
    add_library(Detour SHARED ${SOURCES})
endif()

add_library(RecastNavigation::Detour ALIAS Detour)

set(Detour_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(Detour PUBLIC
    "$<BUILD_INTERFACE:${Detour_INCLUDE_DIR}>"
)

set_target_properties(Detour PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${VERSION}
        )

install(TARGETS Detour
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library
        )

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION include)
