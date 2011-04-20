CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(detour_SRCS
	Source/DetourAlloc.cpp
	Source/DetourCommon.cpp
	Source/DetourNavMesh.cpp
	Source/DetourNavMeshBuilder.cpp
	Source/DetourNavMeshQuery.cpp
	Source/DetourNode.cpp
)

SET(detour_HDRS
	Include/DetourAlloc.h
	Include/DetourAssert.h
	Include/DetourCommon.h
	Include/DetourNavMesh.h
	Include/DetourNavMeshBuilder.h
	Include/DetourNavMeshQuery.h
	Include/DetourNode.h
)

INCLUDE_DIRECTORIES(Include)

ADD_LIBRARY(Detour ${detour_SRCS} ${detour_HDRS})
