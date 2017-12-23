file(GLOB SOURCES Source/*.cpp)

include_directories(../Detour/Include)
include_directories(Include)

add_library(DetourCrowd STATIC ${SOURCES})

install(TARGETS DetourCrowd ARCHIVE DESTINATION lib)

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION include)
