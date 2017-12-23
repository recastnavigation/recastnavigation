file(GLOB SOURCES Source/*.cpp)

include_directories(Include)

add_library(Recast STATIC ${SOURCES})

install(TARGETS Recast ARCHIVE DESTINATION lib)

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION include)
