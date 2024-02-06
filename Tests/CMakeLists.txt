file(GLOB TESTS_SOURCES Detour/*.cpp Recast/*.cpp)

include_directories(../Detour/Include)
include_directories(../Recast/Include)

add_executable(Tests ${TESTS_SOURCES})

set_property(TARGET Tests PROPERTY CXX_STANDARD 20)

add_dependencies(Tests Recast Detour)
target_link_libraries(Tests Recast Detour)

find_package(Catch2 QUIET)
if (Catch2_FOUND)
	target_link_libraries(Tests Catch2::Catch2WithMain)
	target_include_directories(Tests PRIVATE ${Catch2_DIR}/src)
	message("found")
else()
	add_subdirectory(./contrib/Catch2)
	target_include_directories(Tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/contrib/Catch2/src)
	message("yeh, no")
endif()

add_test(Tests Tests)
