add_library(Contrib STATIC
	fastlz/fastlz.c
	imgui/imgui.cpp
	imgui/imgui_demo.cpp
	imgui/imgui_draw.cpp
	imgui/imgui_tables.cpp
	imgui/imgui_widgets.cpp
	implot/implot.cpp
	implot/implot_demo.cpp
	implot/implot_items.cpp
	imgui/backends/imgui_impl_sdl2.cpp
	imgui/backends/imgui_impl_opengl2.cpp
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
	target_compile_options(Contrib PRIVATE -Wno-error -w)
endif()

# Public include directories so consumers can use these headers
target_include_directories(Contrib PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/fastlz
	${CMAKE_CURRENT_SOURCE_DIR}/imgui
	${CMAKE_CURRENT_SOURCE_DIR}/implot
	${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
)

# SDL2 includes needed for imgui_impl_sdl2.cpp
if(WIN32)
	target_include_directories(Contrib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/SDL/include)
elseif(APPLE)
	# SDL2_INCLUDE_DIR and SDL2_INCLUDE_DIR_PARENT are set by the parent CMakeLists.txt
	target_include_directories(Contrib PRIVATE ${SDL2_INCLUDE_DIR} ${SDL2_INCLUDE_DIR_PARENT})
else()
	# On Linux, link against SDL2 to get the include directories from the imported target
	target_link_libraries(Contrib PRIVATE SDL2::SDL2)
endif()
