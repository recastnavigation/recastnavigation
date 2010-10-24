NAME = RecastDemo

SOURCES = \
	ChunkyTriMesh.cpp \
	ConvexVolumeTool.cpp \
	CrowdManager.cpp \
	CrowdTool.cpp \
	Filelist.cpp \
	foo \
	imgui.cpp \
	imguiRenderGL.cpp \
	InputGeom.cpp \
	main.cpp \
	MeshLoaderObj.cpp \
	NavMeshTesterTool.cpp \
	OffMeshConnectionTool.cpp \
	PerfTimer.cpp \
	Sample.cpp \
	Sample_Debug.cpp \
	SampleInterfaces.cpp \
	Sample_SoloMeshSimple.cpp \
	Sample_SoloMeshTiled.cpp \
	Sample_TileMesh.cpp \
	SDLMain.m \
	SlideShow.cpp \
	TestCase.cpp \
	ValueHistory.cpp

HEADERS = \
	ChunkyTriMesh.h \
	ConvexVolumeTool.h \
	CrowdManager.h \
	CrowdTool.h \
	Filelist.h \
	foo \
	imgui.h \
	imguiRenderGL.h \
	InputGeom.h \
	MeshLoaderObj.h \
	NavMeshTesterTool.h \
	OffMeshConnectionTool.h \
	PerfTimer.h \
	Sample_Debug.h \
	Sample.h \
	SampleInterfaces.h \
	Sample_SoloMeshSimple.h \
	Sample_SoloMeshTiled.h \
	Sample_TileMesh.h \
	SDLMain.h \
	SlideShow.h \
	TestCase.h \
	ValueHistory.h

CPPFLAGS = \
	-I $(NAME)/Contrib \
	-I DebugUtils/Include \
	-I Detour/Include \
	-I Recast/Include \
	`pkg-config --cflags sdl`
  
LDFLAGS = \
	-L $(BIN) \
	-lDetour \
	-lRecast \
	-lGL -lGLU \
	`pkg-config --libs sdl`

LIBS = $(BIN)/DebugUtils.a

include $(BUILD)/Program.mk