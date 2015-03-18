--
-- premake4 file to build RecastDemo
-- http://industriousone.com/premake
--

local action = _ACTION or ""
local todir = "Build/" .. action

solution "recastnavigation"
	configurations { 
		"Debug",
		"Release"
	}
	location (todir)

	-- extra warnings, no exceptions or rtti
	flags { 
		"ExtraWarnings",
		"FloatFast",
		"NoExceptions",
		"NoRTTI",
		"Symbols"
	}

	-- debug configs
	configuration "Debug*"
		defines { "DEBUG" }
		targetdir ( todir .. "/lib/Debug" )
 
 	-- release configs
	configuration "Release*"
		defines { "NDEBUG" }
		flags { "Optimize" }
		targetdir ( todir .. "/lib/Release" )

	-- windows specific
	configuration "windows"
		defines { "WIN32", "_WINDOWS", "_CRT_SECURE_NO_WARNINGS" }


project "DebugUtils"
	language "C++"
	kind "StaticLib"
	includedirs { 
		"../DebugUtils/Include",
		"../Detour/Include",
		"../DetourTileCache/Include",
		"../Recast/Include"
	}
	files { 
		"../DebugUtils/Include/*.h",
		"../DebugUtils/Source/*.cpp"
	}

project "Detour"
	language "C++"
	kind "StaticLib"
	includedirs { 
		"../Detour/Include" 
	}
	files { 
		"../Detour/Include/*.h", 
		"../Detour/Source/*.cpp" 
	}

project "DetourCrowd"
	language "C++"
	kind "StaticLib"
	includedirs {
		"../DetourCrowd/Include",
		"../Detour/Include",
		"../Recast/Include"
	}
	files {
		"../DetourCrowd/Include/*.h",
		"../DetourCrowd/Source/*.cpp"
	}

project "DetourTileCache"
	language "C++"
	kind "StaticLib"
	includedirs {
		"../DetourTileCache/Include",
		"../Detour/Include",
		"../Recast/Include"
	}
	files {
		"../DetourTileCache/Include/*.h",
		"../DetourTileCache/Source/*.cpp"
	}

project "Recast"
	language "C++"
	kind "StaticLib"
	includedirs { 
		"../Recast/Include" 
	}
	files { 
		"../Recast/Include/*.h",
		"../Recast/Source/*.cpp" 
	}

project "RecastDemo"
	language "C++"
	kind "WindowedApp"
	includedirs { 
		"../RecastDemo/Include",
		"../RecastDemo/Contrib",
		"../RecastDemo/Contrib/fastlz",
		"../DebugUtils/Include",
		"../Detour/Include",
		"../DetourCrowd/Include",
		"../DetourTileCache/Include",
		"../Recast/Include"
	}
	files	{ 
		"../RecastDemo/Include/*.h",
		"../RecastDemo/Source/*.cpp",
		"../RecastDemo/Contrib/fastlz/*.h",
		"../RecastDemo/Contrib/fastlz/*.c"
	}

	-- project dependencies
	links { 
		"DebugUtils",
		"Detour",
		"DetourCrowd",
		"DetourTileCache",
		"Recast"
	}

	-- distribute executable in RecastDemo/Bin directory
	targetdir "Bin"

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags glew`",
			"`pkg-config --cflags glfw3`",
			"`pkg-config --cflags gl`",
		}
		linkoptions { 
			"`pkg-config --libs glew`",
			"`pkg-config --libs glfw3`",
			"`pkg-config --libs gl`",
		}
		links {
			"pthread",
			"X11",
			"Xrandr",
			"Xinerama",
			"Xi",
			"Xxf86vm",
			"Xcursor"
		}

	-- windows library cflags and libs
	configuration { "windows" }
		links { "glfw3", "gdi32", "winmm", "user32", "glew32", "opengl32", "kernel32" }
		libdirs { "contrib/libs" }
		debugdir "Bin"

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		links { "glfw3" }
		linkoptions { "-framework OpenGL", "-framework Cocoa", "-framework IOKit", "-framework CoreVideo" }
