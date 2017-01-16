--
-- premake5 file to build RecastDemo
-- http://premake.github.io/
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
		"Symbols"
	}
	exceptionhandling "Off"
	rtti "Off"

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

	-- linux specific
	configuration { "linux", "gmake" }
		buildoptions {
			"-Wall",
			"-Werror"
		}

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
			"`pkg-config --cflags sdl2`",
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"`pkg-config --libs sdl2`",
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}

	-- windows library cflags and libs
	configuration { "windows" }
		includedirs { "../RecastDemo/Contrib/SDL/include" }
		libdirs { "../RecastDemo/Contrib/SDL/lib/x86" }
		debugdir "../RecastDemo/Bin/"
		links { 
			"glu32",
			"opengl32",
			"SDL2",
			"SDL2main",
		}
		postbuildcommands {
			-- Copy the SDL2 dll to the Bin folder.
			'{COPY} "%{wks.location}../../Contrib/SDL/lib/x86/SDL2.dll" "%{cfg.targetdir}"'
		}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		includedirs { "/Library/Frameworks/SDL2.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		links { 
			"OpenGL.framework", 
			"SDL2.framework",
			"Cocoa.framework",
		}

project "Tests"
	language "C++"
	kind "ConsoleApp"

	-- Catch requires RTTI and exceptions
	exceptionhandling "On"
	rtti "On"

	includedirs { 
		"../DebugUtils/Include",
		"../Detour/Include",
		"../DetourCrowd/Include",
		"../DetourTileCache/Include",
		"../Recast/Include",
		"../Recast/Source",
		"../Tests/Recast",
		"../Tests",
	}
	files	{ 
		"../Tests/*.h",
		"../Tests/*.hpp",
		"../Tests/*.cpp",
		"../Tests/Recast/*.h",
		"../Tests/Recast/*.cpp",
	}

	-- project dependencies
	links { 
		"DebugUtils",
		"Detour",
		"DetourCrowd",
		"DetourTileCache",
		"Recast",
	}

	-- distribute executable in RecastDemo/Bin directory
	targetdir "Bin"

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags sdl2`",
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`",
			"-Wno-parentheses" -- Disable parentheses warning for the Tests target, as Catch's macros generate this everywhere.
		}
		linkoptions { 
			"`pkg-config --libs sdl2`",
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}

	-- windows library cflags and libs
	configuration { "windows" }
		includedirs { "../RecastDemo/Contrib/SDL/include" }
		libdirs { "../RecastDemo/Contrib/SDL/lib/x86" }
		debugdir "../RecastDemo/Bin/"
		links { 
			"glu32",
			"opengl32",
			"SDL2",
			"SDL2main",
		}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp"
		includedirs { "/Library/Frameworks/SDL2.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		links { 
			"OpenGL.framework", 
			"SDL2.framework",
			"Cocoa.framework",
		}
