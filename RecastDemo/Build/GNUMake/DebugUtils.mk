NAME = DebugUtils

SOURCES = \
	DebugDraw.cpp \
	DetourDebugDraw.cpp \
	RecastDebugDraw.cpp \
	RecastDump.cpp

HEADERS = \
	DebugDraw.h \
	DetourDebugDraw.h \
	RecastDebugDraw.h \
	RecastDump.h

CPPFLAGS = \
	-I Detour/Include \
	-I Recast/Include

include $(BUILD)/HelperLibrary.mk