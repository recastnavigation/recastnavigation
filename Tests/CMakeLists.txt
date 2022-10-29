file(GLOB TESTS_SOURCES *.cpp Detour/*.cpp Recast/*.cpp Contrib/Catch/*.cpp)

include_directories(../Detour/Include)
include_directories(../Recast/Include)
include_directories(./Contrib/Catch)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(Tests ${TESTS_SOURCES})
add_dependencies(Tests Recast Detour)
target_link_libraries(Tests Recast Detour)
add_test(Tests Tests)
