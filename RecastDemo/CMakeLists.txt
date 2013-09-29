# 2.8.5 required for CMAKE_LIBRARY_ARCHITECTURE
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.5)

SET(recastdemo_SRCS 
	Source/ChunkyTriMesh.cpp
	Source/ConvexVolumeTool.cpp
	Source/CrowdTool.cpp
	Source/Filelist.cpp
	Source/imgui.cpp
	Source/imguiRenderGL.cpp
	Source/InputGeom.cpp
	Source/main.cpp
	Source/MeshLoaderObj.cpp
	Source/NavMeshPruneTool.cpp
	Source/NavMeshTesterTool.cpp
	Source/OffMeshConnectionTool.cpp
	Source/PerfTimer.cpp
	Source/Sample.cpp
	Source/Sample_Debug.cpp
	Source/Sample_SoloMesh.cpp
	Source/Sample_TileMesh.cpp
	Source/Sample_TempObstacles.cpp
	Source/SampleInterfaces.cpp
	Source/SlideShow.cpp
	Source/TestCase.cpp
	Source/ValueHistory.cpp
	Contrib/fastlz/fastlz.c
)

SET(recastdemo_HDRS
	Include/ChunkyTriMesh.h
	Include/ConvexVolumeTool.h
	Include/CrowdTool.h
	Include/Filelist.h
	Include/imgui.h
	Include/imguiRenderGL.h
	Include/InputGeom.h
	Include/MeshLoaderObj.h
	Include/NavMeshPruneTool.h
	Include/NavMeshTesterTool.h
	Include/OffMeshConnectionTool.h
	Include/PerfTimer.h
	Include/Sample.h
	Include/Sample_Debug.h
	Include/Sample_SoloMesh.h
	Include/Sample_TileMesh.h
	Include/Sample_TempObstacles.h
	Include/SampleInterfaces.h
	Include/SDLMain.h
	Include/SlideShow.h
	Include/TestCase.h
	Include/ValueHistory.h
)

IF(MSVC)
	# Look in the Contrib directory for SDL when building with MSVC
	SET(CMAKE_PREFIX_PATH Contrib/SDL)
	# Look in arch dependendent lib subfolder for libsdl
	IF(CMAKE_GENERATOR MATCHES Win64)
		SET(CMAKE_LIBRARY_ARCHITECTURE x64)
	ELSE()
		SET(CMAKE_LIBRARY_ARCHITECTURE x86)
	ENDIF()
ENDIF()

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(SDL REQUIRED)

INCLUDE_DIRECTORIES(Include 
	Contrib
	Contrib/fastlz
	../DebugUtils/Include
	../Detour/Include
	../DetourCrowd/Include
	../DetourTileCache/Include
	../Recast/Include
	${SDL_INCLUDE_DIR}
)

IF(XCODE)
	ADD_EXECUTABLE(RecastDemo MACOSX_BUNDLE Source/SDLMain.m Include/SDLMain.h ${recastdemo_SRCS} ${recastdemo_HDRS})
ELSE()
	ADD_EXECUTABLE(RecastDemo WIN32 ${recastdemo_SRCS} ${recastdemo_HDRS})
ENDIF()

TARGET_LINK_LIBRARIES(RecastDemo DebugUtils Detour DetourCrowd DetourTileCache Recast ${SDL_LIBRARY} ${OPENGL_LIBRARIES})

IF(MSVC)
	# Enable some linker optimisations
	SET(recastdemo_LINK_OPTS "/OPT:REF /OPT:ICF")
	SET_TARGET_PROPERTIES(RecastDemo PROPERTIES LINK_FLAGS_RELEASE ${recastdemo_LINK_OPTS})
	SET_TARGET_PROPERTIES(RecastDemo PROPERTIES LINK_FLAGS_MINSIZEREL ${recastdemo_LINK_OPTS})
	SET_TARGET_PROPERTIES(RecastDemo PROPERTIES LINK_FLAGS_RELWITHDEBINFO ${recastdemo_LINK_OPTS})
ENDIF()

SET_TARGET_PROPERTIES(RecastDemo PROPERTIES DEBUG_POSTFIX "_Debug")
SET_TARGET_PROPERTIES(RecastDemo PROPERTIES MINSIZEREL_POSTFIX "_MinSizeRel")
SET_TARGET_PROPERTIES(RecastDemo PROPERTIES RELWITHDEBINFO_POSTFIX "_RelWithDebInfo")

SET(recastdemo_OUTPUT_DIR "${RecastNavigation_SOURCE_DIR}/RecastDemo/Bin")
SET_TARGET_PROPERTIES(RecastDemo PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${recastdemo_OUTPUT_DIR})
SET_TARGET_PROPERTIES(RecastDemo PROPERTIES RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${recastdemo_OUTPUT_DIR})
SET_TARGET_PROPERTIES(RecastDemo PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${recastdemo_OUTPUT_DIR})
SET_TARGET_PROPERTIES(RecastDemo PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${recastdemo_OUTPUT_DIR})


