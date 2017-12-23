file(GLOB SOURCES Source/*.cpp)

include_directories(../Recast/Include)
include_directories(../Detour/Include)
include_directories(../DetourTileCache/Include)
include_directories(Include)

add_library(DebugUtils STATIC ${SOURCES})

install(TARGETS DebugUtils ARCHIVE DESTINATION lib)

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION include)
