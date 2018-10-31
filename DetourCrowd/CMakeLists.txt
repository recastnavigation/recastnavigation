file(GLOB SOURCES Source/*.cpp)

if (RECASTNAVIGATION_STATIC)
    add_library(DetourCrowd STATIC ${SOURCES})
else ()
    add_library(DetourCrowd SHARED ${SOURCES})
endif ()

add_library(RecastNavigation::DetourCrowd ALIAS DetourCrowd)

set(DetourCrowd_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(DetourCrowd PUBLIC
    "$<BUILD_INTERFACE:${DetourCrowd_INCLUDE_DIR}>"
)

target_link_libraries(DetourCrowd
    Detour
)

set_target_properties(DetourCrowd PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${VERSION}
        )

install(TARGETS DetourCrowd
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        COMPONENT library
        )

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION include)
