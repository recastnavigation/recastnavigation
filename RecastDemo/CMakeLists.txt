file(GLOB SOURCES Source/*.cpp)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if (NOT SDL2_ROOT_DIR)
  if (WIN32)
    set(SDL2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Contrib/SDL")
  elseif(APPLE)
    set(SDL2_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Bin")
  endif()
endif()

find_package(OpenGL REQUIRED)
if (APPLE)
  set(SDL2_FRAMEWORK_PATH "${SDL2_ROOT_DIR}/SDL2.framework")
  set(SDL2_INCLUDE_DIR "${SDL2_FRAMEWORK_PATH}/Headers")
else()
  find_package(SDL2 REQUIRED)
endif()

# Contrib static library (third-party code)
add_library(Contrib STATIC
	Contrib/fastlz/fastlz.c
	Contrib/imgui/imgui.cpp
	Contrib/imgui/imgui_demo.cpp
	Contrib/imgui/imgui_draw.cpp
	Contrib/imgui/imgui_tables.cpp
	Contrib/imgui/imgui_widgets.cpp
	Contrib/implot/implot.cpp
	Contrib/implot/implot_demo.cpp
	Contrib/implot/implot_items.cpp
	Contrib/imgui/backends/imgui_impl_sdl2.cpp
	Contrib/imgui/backends/imgui_impl_opengl2.cpp
)

set_target_properties(Contrib PROPERTIES
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED ON
	CXX_EXTENSIONS OFF
)

# Remove warnings-as-errors for third-party code
if(MSVC)
	target_compile_options(Contrib PRIVATE /W3 /WX-)
else()
	target_compile_options(Contrib PRIVATE -Wno-error)
endif()

target_include_directories(Contrib PRIVATE
	Contrib/imgui
	Contrib/implot
	Contrib/imgui/backends
)

if (WIN32)
	target_include_directories(Contrib PRIVATE Contrib/SDL/include)
elseif (APPLE)
	target_include_directories(Contrib PRIVATE ${SDL2_INCLUDE_DIR})
else()
	target_include_directories(Contrib PRIVATE ${SDL2_INCLUDE_DIRS})
endif()

include_directories(SYSTEM ${OPENGL_INCLUDE_DIR})
include_directories(SYSTEM Contrib/fastlz)
include_directories(SYSTEM Contrib)
include_directories(SYSTEM Contrib/imgui)
include_directories(SYSTEM Contrib/implot)
include_directories(SYSTEM Contrib/imgui/backends)
include_directories(../DebugUtils/Include)
include_directories(../Detour/Include)
include_directories(../DetourCrowd/Include)
include_directories(../DetourTileCache/Include)
include_directories(../Recast/Include)
include_directories(Include)
include_directories(Contrib/SDL/include)
if (APPLE)
  include_directories(${SDL2_INCLUDE_DIR})
endif()

if (WIN32)
    add_executable(RecastDemo WIN32 ${SOURCES})
elseif (APPLE)
    add_executable(RecastDemo MACOSX_BUNDLE ${SOURCES})
else()
    add_executable(RecastDemo ${SOURCES})
endif()

set_target_properties(RecastDemo PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF  # Disable compiler-specific extensions
)

# Suppress deprecated OpenGL warnings on macOS (gluPerspective, gluProject, etc.)
if (APPLE)
    target_compile_options(RecastDemo PRIVATE -Wno-deprecated-declarations)
endif()

if (WIN32)
  if ("${CMAKE_MAKE_PROGRAM}" MATCHES "MSBuild")
    target_compile_definitions(RecastDemo PUBLIC _CRT_SECURE_NO_WARNINGS)
    add_custom_command(TARGET RecastDemo
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy "${SDL2_RUNTIME_LIBRARY}" ${CMAKE_BINARY_DIR}/RecastDemo/$(ConfigurationName)/
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes ${CMAKE_BINARY_DIR}/RecastDemo/$(ConfigurationName)/Meshes
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases ${CMAKE_BINARY_DIR}/RecastDemo/$(ConfigurationName)/TestCases
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf ${CMAKE_BINARY_DIR}/RecastDemo/$(ConfigurationName)/
    )
  elseif (MINGW)
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

add_dependencies(RecastDemo DebugUtils Detour DetourCrowd DetourTileCache Recast Contrib)
if (APPLE)
  target_link_libraries(
	RecastDemo
	${OPENGL_LIBRARIES}
	"-F${SDL2_ROOT_DIR} -framework SDL2"
	DebugUtils
	Detour
	DetourCrowd
	DetourTileCache
	Recast
	Contrib)
else()
  target_link_libraries(
	RecastDemo
	${OPENGL_LIBRARIES}
	SDL2::SDL2main
	DebugUtils
	Detour
	DetourCrowd
	DetourTileCache
	Recast
	Contrib)
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
