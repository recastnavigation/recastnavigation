NAME = Detour

SOURCES = \
	DetourAlloc.cpp \
	DetourCommon.cpp \
	DetourNavMesh.cpp \
	DetourNavMeshBuilder.cpp \
	DetourNavMeshQuery.cpp \
	DetourNode.cpp \
	DetourObstacleAvoidance.cpp

HEADERS = \
	DetourAlloc.h \
	DetourAssert.h \
	DetourCommon.h \
	DetourNavMesh.h \
	DetourNavMeshBuilder.h \
	DetourNavMeshQuery.h \
	DetourNode.h \
	DetourObstacleAvoidance.h

include $(BUILD)/Library.mk