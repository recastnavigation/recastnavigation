NAME = Recast

OBJECTS = \
	Recast.cpp \
	RecastAlloc.cpp \
	RecastArea.cpp \
	RecastFilter.cpp \
	RecastMesh.cpp \
	RecastMeshDetail.cpp \
	RecastRasterization.cpp \
	RecastRegion.cpp

HEADERS = \
	Recast.h \
	RecastAlloc.h \
	RecastAssert.h

include $(BUILD)/Library.mk