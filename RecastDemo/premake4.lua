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
			"`pkg-config --cflags sdl`",
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"`pkg-config --libs sdl`",
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}

	-- windows library cflags and libs
	configuration { "windows" }
		includedirs { "../RecastDemo/Contrib/SDL/include" }
		libdirs { "../RecastDemo/Contrib/SDL/lib/x86" }
		links { 
			"opengl32",
			"glu32",
			"sdlmain",
			"sdl"
		}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		links { 
			"OpenGL.framework", 
			"/Library/Frameworks/SDL.framework", 
			"Cocoa.framework",
		}

		files {
			"../RecastDemo/Include/SDLMain.h", 
			"../RecastDemo/Source/SDLMain.m",
-- These don't seem to work in xcode4 target yet.
--			"Info.plist",
--			"Icon.icns",
--			"English.lproj/InfoPlist.strings",
--			"English.lproj/MainMenu.xib",
		}
