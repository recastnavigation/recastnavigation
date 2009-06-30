#ifndef RECASTBUILDERSTATMESHSIMPLE_H
#define RECASTBUILDERSTATMESHSIMPLE_H

#include "BuilderStatMesh.h"
#include "DetourStatNavMesh.h"
#include "Recast.h"
#include "RecastLog.h"

class BuilderStatMeshSimple : public BuilderStatMesh
{
protected:

	bool m_keepInterResults;
	rcBuildTimes m_buildTimes; 

	unsigned char* m_triflags;
	rcHeightfield* m_solid;
	rcCompactHeightfield* m_chf;
	rcContourSet* m_cset;
	rcPolyMesh* m_polyMesh;
	rcConfig m_cfg;	
	
	enum DrawMode
	{
		DRAWMODE_NAVMESH,
		DRAWMODE_NAVMESH_TRANS,
		DRAWMODE_NAVMESH_BVTREE,
		DRAWMODE_NAVMESH_INVIS,
		DRAWMODE_MESH,
		DRAWMODE_VOXELS,
		DRAWMODE_VOXELS_WALKABLE,
		DRAWMODE_COMPACT,
		DRAWMODE_COMPACT_DISTANCE,
		DRAWMODE_COMPACT_REGIONS,
		DRAWMODE_REGION_CONNECTIONS,
		DRAWMODE_RAW_CONTOURS,
		DRAWMODE_BOTH_CONTOURS,
		DRAWMODE_CONTOURS,
		DRAWMODE_POLYMESH,
		MAX_DRAWMODE
	};
	
	DrawMode m_drawMode;
	
	void cleanup();
		
public:
	BuilderStatMeshSimple();
	virtual ~BuilderStatMeshSimple();
	
	virtual void handleSettings();
	virtual void handleDebugMode();
	
	virtual void handleRender();
	virtual void handleMeshChanged(const float* verts, int nverts,
								   const int* tris, const float* trinorms, int ntris,
								   const float* bmin, const float* bmax);
	virtual bool handleBuild();
};


#endif // RECASTBUILDERSTATMESHSIMPLE_H
