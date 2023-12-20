file(GLOB SOURCES Source/*.cpp Contrib/fastlz/fastlz.c)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#include_directories(SYSTEM ${OPENGL_INCLUDE_DIR})
include_directories(SYSTEM Contrib/fastlz)
include_directories(SYSTEM Contrib)
#include_directories(../DebugUtils/Include)
include_directories(../Detour/Include)
include_directories(../DetourCrowd/Include)
include_directories(../DetourTileCache/Include)
include_directories(../Recast/Include)
include_directories(Include)
#include_directories(Contrib/SDL/include)

add_executable(RecastCLI ${SOURCES})

if( WIN32 )
  if ( "${CMAKE_MAKE_PROGRAM}" MATCHES "MSBuild" )
    add_custom_command(TARGET RecastCLI
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes ${CMAKE_BINARY_DIR}/RecastCLI/$(ConfigurationName)/Meshes
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases ${CMAKE_BINARY_DIR}/RecastCLI/$(ConfigurationName)/TestCases
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf ${CMAKE_BINARY_DIR}/RecastCLI/$(ConfigurationName)/
    )
  elseif ( MINGW )
    add_custom_command(TARGET RecastCLI
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes ${CMAKE_BINARY_DIR}/RecastCLI/Meshes
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases ${CMAKE_BINARY_DIR}/RecastCLI/TestCases
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf ${CMAKE_BINARY_DIR}/RecastCLI/
    )
  endif()
else()
    file(COPY Bin/Meshes DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    file(COPY Bin/TestCases DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    file(COPY Bin/DroidSans.ttf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()


add_dependencies(RecastCLI  DebugUtils Detour DetourCrowd DetourTileCache Recast)
target_link_libraries(RecastCLI  DebugUtils Detour DetourCrowd DetourTileCache Recast)

install(TARGETS RecastCLI
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin)
install(DIRECTORY Bin/Meshes DESTINATION bin)
install(DIRECTORY Bin/TestCases DESTINATION bin)
install(FILES Bin/DroidSans.ttf DESTINATION bin)

add_subdirectory(Tests)
