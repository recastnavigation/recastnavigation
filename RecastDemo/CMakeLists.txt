file(GLOB SOURCES Source/*.cpp Contrib/fastlz/fastlz.c)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(NOT SDL2_ROOT_DIR)
  if(WIN32)
    set(SDL2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Contrib/SDL")
  elseif(APPLE)
    set(SDL2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Bin")
  endif()
endif()

find_package(OpenGL REQUIRED)
if(APPLE)
  find_library(SDL2_LIBRARY 
    NAMES SDL2
    PATHS ${SDL2_ROOT_DIR}
    REQUIRED)
else()
  find_package(SDL2 REQUIRED)
endif()

include_directories(SYSTEM ${OPENGL_INCLUDE_DIR})
include_directories(SYSTEM Contrib/fastlz)
include_directories(SYSTEM Contrib)
include_directories(../DebugUtils/Include)
include_directories(../Detour/Include)
include_directories(../DetourCrowd/Include)
include_directories(../DetourTileCache/Include)
include_directories(../Recast/Include)
include_directories(Include)
include_directories(Contrib/SDL/include)
if(APPLE)
  include_directories(${SDL2_LIBRARY}/Headers)
endif()

if (WIN32)
    add_executable(RecastDemo WIN32 ${SOURCES})
elseif(APPLE)
    add_executable(RecastDemo MACOSX_BUNDLE ${SOURCES})
else()
    add_executable(RecastDemo ${SOURCES})
endif()

if( WIN32 )
  if ( "${CMAKE_MAKE_PROGRAM}" MATCHES "MSBuild" )
    add_custom_command(TARGET RecastDemo
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy "${SDL2_RUNTIME_LIBRARY}" ${CMAKE_BINARY_DIR}/RecastDemo/$(ConfigurationName)/
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes ${CMAKE_BINARY_DIR}/RecastDemo/$(ConfigurationName)/Meshes
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases ${CMAKE_BINARY_DIR}/RecastDemo/$(ConfigurationName)/TestCases
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf ${CMAKE_BINARY_DIR}/RecastDemo/$(ConfigurationName)/
    )
  elseif ( MINGW )
    add_custom_command(TARGET RecastDemo
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy "${SDL2_RUNTIME_LIBRARY}" ${CMAKE_BINARY_DIR}/RecastDemo/
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes ${CMAKE_BINARY_DIR}/RecastDemo/Meshes
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases ${CMAKE_BINARY_DIR}/RecastDemo/TestCases
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf ${CMAKE_BINARY_DIR}/RecastDemo/
    )
  endif()
else()
    file(COPY Bin/Meshes DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    file(COPY Bin/TestCases DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    file(COPY Bin/DroidSans.ttf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()


add_dependencies(RecastDemo DebugUtils Detour DetourCrowd DetourTileCache Recast)
if(APPLE)
  target_link_libraries(RecastDemo ${OPENGL_LIBRARIES} ${SDL2_LIBRARY} DebugUtils Detour DetourCrowd DetourTileCache Recast)
else()
  target_link_libraries(RecastDemo ${OPENGL_LIBRARIES} SDL2::SDL2main DebugUtils Detour DetourCrowd DetourTileCache Recast)
endif()


install(TARGETS RecastDemo
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin)
install(DIRECTORY Bin/Meshes DESTINATION bin)
install(DIRECTORY Bin/TestCases DESTINATION bin)
install(FILES Bin/DroidSans.ttf DESTINATION bin)

if (WIN32)
    install(FILES "${SDL2_RUNTIME_LIBRARY}" DESTINATION bin)
endif()