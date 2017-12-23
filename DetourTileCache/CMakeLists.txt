file(GLOB SOURCES Source/*.cpp)

include_directories(../Detour/Include)
include_directories(Include)

add_library(DetourTileCache STATIC ${SOURCES})

install(TARGETS DetourTileCache ARCHIVE DESTINATION lib)

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION include)
