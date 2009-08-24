//
// Copyright (c) 2009 Mikko Mononen memon@inside.org
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

#ifndef RECASTSAMPLETILEDMESH_H
#define RECASTSAMPLETILEDMESH_H

#include "Sample.h"
#include "DetourTileNavMesh.h"
#include "Recast.h"
#include "RecastLog.h"
#include "ChunkyTriMesh.h"

class Sample_TileMesh : public Sample
{
protected:
	
	bool m_keepInterResults;
	rcBuildTimes m_buildTimes; 

	dtTiledNavMesh* m_navMesh;
	rcChunkyTriMesh* m_chunkyMesh;
	unsigned char* m_triflags;
	rcHeightfield* m_solid;
	rcCompactHeightfield* m_chf;
	rcContourSet* m_cset;
	rcPolyMesh* m_pmesh;
	rcPolyMeshDetail* m_dmesh;
	rcConfig m_cfg;	
	
	float m_tileSize;
			
	float m_spos[3];
	float m_epos[3];
	bool m_sposSet;
	bool m_eposSet;
	
	float m_tileCol[4];
	float m_tileBmin[3];
	float m_tileBmax[3];
	float m_tileBuildTime;
	float m_tileMemUsage;
	int m_tileTriCount;
	
	enum ToolMode
	{
		TOOLMODE_CREATE_TILES,
		TOOLMODE_PATHFIND,
		TOOLMODE_RAYCAST,
		TOOLMODE_DISTANCE_TO_WALL,
		TOOLMODE_FIND_POLYS_AROUND,
	};
	
	dtTilePolyRef m_startRef;
	dtTilePolyRef m_endRef;
	float m_polyPickExt[3];

	static const int MAX_POLYS = 256;
	
	dtTilePolyRef m_polys[MAX_POLYS];
	dtTilePolyRef m_parent[MAX_POLYS];
	int m_npolys;
	float m_straightPath[MAX_POLYS*3];
	int m_nstraightPath;
	float m_hitPos[3];
	float m_hitNormal[3];
	float m_distanceToWall;
	
	ToolMode m_toolMode;
	
	void toolRecalc();
	
	void buildTile(const float* pos);
	void removeTile(const float* pos);
	
	unsigned char* buildTileMesh(const float* bmin, const float* bmax, int& dataSize);

	void cleanup();
	
public:
	Sample_TileMesh();
	virtual ~Sample_TileMesh();
	
	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	
	virtual void setToolStartPos(const float* p);
	virtual void setToolEndPos(const float* p);
	
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(const float* verts, int nverts,
								   const int* tris, const float* trinorms, int ntris,
								   const float* bmin, const float* bmax);
	virtual bool handleBuild();
};


#endif // RECASTBUILDERTILEDMESH_H
