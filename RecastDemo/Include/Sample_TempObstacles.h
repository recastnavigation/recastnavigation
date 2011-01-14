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

#ifndef RECASTSAMPLETEMPOBSTACLE_H
#define RECASTSAMPLETEMPOBSTACLE_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "Recast.h"
#include "ChunkyTriMesh.h"


class Sample_TempObstacles : public Sample
{
protected:
	bool m_keepInterResults;
	float m_cacheBuildTimeMs;
	bool m_drawPortals;

	class TileCache* m_tileCache;
	class ObstacleSet* m_obs;
	
	enum DrawMode
	{
		DRAWMODE_NAVMESH,
		DRAWMODE_NAVMESH_TRANS,
		DRAWMODE_NAVMESH_BVTREE,
		DRAWMODE_NAVMESH_NODES,
		DRAWMODE_NAVMESH_PORTALS,
		DRAWMODE_NAVMESH_INVIS,
		DRAWMODE_MESH,
		MAX_DRAWMODE
	};
	
	DrawMode m_drawMode;
	
	int m_maxTiles;
	int m_maxPolysPerTile;
	float m_tileSize;
	
	int m_rebuildTileCount;
	float m_rebuildTime;
	
	int calcTouchedTiles(const float minx, const float minz, const float maxx, const float maxz,
						 struct TouchedTile* touched, const int maxTouched);
	
	void rebuildTiles(const struct TouchedTile* touched, const int ntouched);
	
public:
	Sample_TempObstacles();
	virtual ~Sample_TempObstacles();
	
	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(class InputGeom* geom);
	virtual bool handleBuild();
	
	void getTilePos(const float* pos, int& tx, int& ty);
	
	void renderCachedTile(const int tx, const int ty);
	void renderCachedTileOverlay(const int tx, const int ty, double* proj, double* model, int* view);

	void addTempObstacle(const float* pos);
	void removeTempObstacle(const float* sp, const float* sq);
	void clearAllTempObstacles();
};


#endif // RECASTSAMPLETEMPOBSTACLE_H
