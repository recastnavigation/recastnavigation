file(GLOB SOURCES Source/*.cpp)

include_directories(Include)

add_library(Detour STATIC ${SOURCES})

install(TARGETS Detour ARCHIVE DESTINATION lib)

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION include)
