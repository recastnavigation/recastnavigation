#ifndef RECASTSAMPLESTATMESHTILED_H
#define RECASTSAMPLESTATMESHTILED_H

#include "Sample_StatMesh.h"
#include "DetourStatNavMesh.h"
#include "Recast.h"
#include "RecastLog.h"
#include "ChunkyTriMesh.h"

class Sample_StatMeshTiled : public Sample_StatMesh
{
protected:

	struct Tile
	{
		inline Tile() : chf(0), solid(0), cset(0), pmesh(0), dmesh(0), buildTime(0) {}
		inline ~Tile() { delete chf; delete cset; delete solid; delete pmesh; delete dmesh; }
		rcCompactHeightfield* chf;
		rcHeightfield* solid;
		rcContourSet* cset;
		rcPolyMesh* pmesh;
		rcPolyMeshDetail* dmesh;
		int buildTime;
	};
	
	struct TileSet
	{
		inline TileSet() : width(0), height(0), tiles(0) {}
		inline ~TileSet() { delete [] tiles; }
		int width, height;
		float bmin[3], bmax[3];
		float cs, ch;
		Tile* tiles;
	};
	
	bool m_measurePerTileTimings;
	bool m_keepInterResults;
	float m_tileSize;
	rcBuildTimes m_buildTimes; 
	
	rcChunkyTriMesh* m_chunkyMesh;
	rcPolyMesh* m_pmesh;
	rcPolyMeshDetail* m_dmesh;
	rcConfig m_cfg;	
	TileSet* m_tileSet;

	static const int MAX_STAT_BUCKETS = 1000;
	int m_statPolysPerTile[MAX_STAT_BUCKETS];
	int m_statPolysPerTileSamples;
	int m_statTimePerTile[MAX_STAT_BUCKETS];
	int m_statTimePerTileSamples;
	
	
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
		DRAWMODE_POLYMESH_DETAIL,
		MAX_DRAWMODE
	};
	
	DrawMode m_drawMode;
	
	void cleanup();
	
public:
	Sample_StatMeshTiled();
	virtual ~Sample_StatMeshTiled();
	
	virtual void handleSettings();
	virtual void handleDebugMode();
	
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(const float* verts, int nverts,
								   const int* tris, const float* trinorms, int ntris,
								   const float* bmin, const float* bmax);
	virtual bool handleBuild();
};


#endif // RECASTSAMPLESTATMESHTILED_H
