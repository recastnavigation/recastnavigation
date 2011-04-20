CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(debugutils_SRCS 
	Source/DebugDraw.cpp
	Source/DetourDebugDraw.cpp
	Source/RecastDebugDraw.cpp
	Source/RecastDump.cpp
)

SET(debugutils_HDRS
	Include/DebugDraw.h
	Include/DetourDebugDraw.h
	Include/RecastDebugDraw.h
	Include/RecastDump.h
)

INCLUDE_DIRECTORIES(Include 
	../Detour/Include
	../DetourTileCache/Include
	../Recast/Include
)

ADD_LIBRARY(DebugUtils ${debugutils_SRCS} ${debugutils_HDRS})
