--
-- premake5 file to build RecastCLI
-- http://premake.github.io/
--

local action = _ACTION or ""
local todir = "Build/" .. action

workspace "recastnavigation"
	configurations {
		"Debug",
		"Release"
	}

	location (todir)

	floatingpoint "Fast"
	exceptionhandling "Off"
	rtti "Off"
	symbols "On"
	flags { "FatalCompileWarnings" }
	cppdialect "C++98"

	-- debug configs
	filter "configurations:Debug"
		defines { "DEBUG" }
		targetdir ( todir .. "/lib/Debug" )

 	-- release configs
	filter "configurations:Release"
		defines { "RC_DISABLE_ASSERTS" }
		optimize "On"
		targetdir ( todir .. "/lib/Release" )

	filter "system:not windows"
		warnings "Extra"

	-- windows specific
	filter "system:windows"
		platforms { "Win32", "Win64" }
		defines { "WIN32", "_WINDOWS", "_CRT_SECURE_NO_WARNINGS", "_HAS_EXCEPTIONS=0" }
		-- warnings "Extra" uses /W4 which is too aggressive for us, so use W3 instead.
		-- Disable:
		-- * C4351: new behavior for array initialization
		buildoptions { "/W3", "/wd4351" }

	filter "platforms:Win32"
		architecture "x32"

	filter "platforms:Win64"
		architecture "x64"

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
	-- linux library cflags and libs
	filter {"system:linux", "toolset:gcc"}
		buildoptions {
			"-Wno-error=class-memaccess",
			"-Wno-error=maybe-uninitialized"
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

project "RecastCLI"
	language "C++"
	kind "ConsoleApp"
	includedirs {
		"../RecastCLI/Include",
		"../RecastCLI/Contrib",
		"../RecastCLI/Contrib/fastlz",
		"../DebugUtils/Include",
		"../Detour/Include",
		"../DetourCrowd/Include",
		"../DetourTileCache/Include",
		"../Recast/Include"
	}
	files {
		"../RecastCLI/Include/*.h",
		"../RecastCLI/Source/*.cpp",
		"../RecastCLI/Contrib/fastlz/*.h",
		"../RecastCLI/Contrib/fastlz/*.c"
	}

	-- project dependencies
	links {
		"DebugUtils",
		"Detour",
		"DetourCrowd",
		"DetourTileCache",
		"Recast"
	}

	-- distribute executable in RecastCLI/Bin directory
	targetdir "Bin"

	-- linux library cflags and libs
	filter "system:linux"
		buildoptions {
			"-Wno-ignored-qualifiers",
		}

	filter { "system:linux", "toolset:gcc", "files:*.c" }
		buildoptions {
			"-Wno-class-memaccess"
		}

	-- windows library cflags and libs
	filter "system:windows"
		debugdir "../RecastCLI/Bin/"

	-- mac includes and libs
	filter "system:macosx"
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp

project "Tests"
	language "C++"
	kind "ConsoleApp"
	cppdialect "C++14" -- Catch requires newer C++ features

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
		"../Tests/Contrib"
	}
	files {
		"../Tests/*.h",
		"../Tests/*.hpp",
		"../Tests/*.cpp",
		"../Tests/Recast/*.h",
		"../Tests/Recast/*.cpp",
		"../Tests/Detour/*.h",
		"../Tests/Detour/*.cpp",
		"../Tests/Contrib/catch2/*.cpp"
	}

	-- project dependencies
	links {
		"DebugUtils",
		"Detour",
		"DetourCrowd",
		"DetourTileCache",
		"Recast",
	}

	-- distribute executable in RecastCLI/Bin directory
	targetdir "Bin"

	-- enable ubsan and asan when compiling with clang
	filter "toolset:clang"
			buildoptions { "-fsanitize=undefined", "-fsanitize=address" } -- , "-fsanitize=memory" }
			linkoptions { "-fsanitize=undefined", "-fsanitize=address" } --, "-fsanitize=memory" }

	-- linux library cflags and libs
	filter "system:linux"
		buildoptions {
			"-Wno-parentheses" -- Disable parentheses warning for the Tests target, as Catch's macros generate this everywhere.
		}
		linkoptions {
			"-lpthread"
		}
project "TestsCLI"
	language "C++"
	kind "ConsoleApp"
	cppdialect "C++14" -- Catch requires newer C++ features

	-- Catch requires RTTI and exceptions
	exceptionhandling "On"
	rtti "On"

	includedirs {
		"../Detour/Include",
		"../DetourCrowd/Include",
		"../DetourTileCache/Include",
		"../DebugUtils/Include",
		"../Recast/Include",
		"../Recast/Source",
		"Tests",
		"Contrib",
		"Include",
	}
	files {
		"Tests/*.cpp",
		"Source/*.cpp",
		"Contrib/catch2/*.cpp"
	}

	-- project dependencies
	links {
		"DebugUtils",
		"Detour",
		"DetourCrowd",
		"DetourTileCache",
		"Recast",
	}

	-- distribute executable in RecastCLI/Bin directory
	targetdir "Bin"

	-- enable ubsan and asan when compiling with clang
	filter "toolset:clang"
			buildoptions { "-fsanitize=undefined", "-fsanitize=address" } -- , "-fsanitize=memory" }
			linkoptions { "-fsanitize=undefined", "-fsanitize=address" } --, "-fsanitize=memory" }

    filter "system:linux"
		buildoptions {
			"-Wno-parentheses" -- Disable parentheses warning for the Tests target, as Catch's macros generate this everywhere.
		}
		linkoptions {
			"-lpthread"
		}