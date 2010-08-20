//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef RECASTSAMPLESOLOMESHTILED_H
#define RECASTSAMPLESOLOMESHTILED_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "Recast.h"
#include "ChunkyTriMesh.h"

class Sample_SoloMeshTiled : public Sample
{
protected:
	struct Tile
	{
		inline Tile() : chf(0), solid(0), cset(0), pmesh(0), dmesh(0), buildTime(0) {}
		inline ~Tile()
		{
			rcFreeCompactHeightfield(chf);
			rcFreeContourSet(cset);
			rcFreeHeightField(solid);
			rcFreePolyMesh(pmesh);
			rcFreePolyMeshDetail(dmesh);
		}
		int x, y;
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
	float m_totalBuildTimeMs;
	
	rcPolyMesh* m_pmesh;
	rcPolyMeshDetail* m_dmesh;
	rcConfig m_cfg;	
	TileSet* m_tileSet;

	static const int MAX_STAT_BUCKETS = 1000;
	int m_statPolysPerTile[MAX_STAT_BUCKETS];
	int m_statPolysPerTileSamples;
	int m_statTimePerTile[MAX_STAT_BUCKETS];
	int m_statTimePerTileSamples;
	
	int m_highLightedTileX, m_highLightedTileY;
	
	enum DrawMode
	{
		DRAWMODE_NAVMESH,
		DRAWMODE_NAVMESH_TRANS,
		DRAWMODE_NAVMESH_BVTREE,
		DRAWMODE_NAVMESH_NODES,
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
	bool canDrawTile(int x, int y);
	
public:
	Sample_SoloMeshTiled();
	virtual ~Sample_SoloMeshTiled();
	
	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(class InputGeom* geom);
	virtual bool handleBuild();
	
	void setHighlightedTile(const float* pos);
	inline int getHilightedTileX() const { return m_highLightedTileX; }
	inline int getHilightedTileY() const { return m_highLightedTileY; }
};


#endif // RECASTSAMPLESOLOMESHTILED_H
