#ifndef RECASTBUILDERTILEDMESH_H
#define RECASTBUILDERTILEDMESH_H

#include "Builder.h"
#include "DetourTiledNavMesh.h"
#include "Recast.h"
#include "RecastLog.h"
#include "ChunkyTriMesh.h"

class BuilderTiledMesh : public Builder
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
	rcPolyMesh* m_polyMesh;
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
	
/*	static const int MAX_POLYS = 256;
	
	dtPolyRef m_startRef;
	dtPolyRef m_endRef;
	dtPolyRef m_polys[MAX_POLYS];
	dtPolyRef m_parent[MAX_POLYS];
	int m_npolys;
	float m_straightPath[MAX_POLYS*3];
	int m_nstraightPath;
	float m_polyPickExt[3];
	
	float m_spos[3];
	float m_epos[3];
	float m_hitPos[3];
	float m_hitNormal[3];
	float m_distanceToWall;
	bool m_sposSet;
	bool m_eposSet;*/
	
	void buildTile(const float* pos);
	void removeTile(const float* pos);
	
	unsigned char* buildTileMesh(const float* bmin, const float* bmax, int& dataSize);

	void cleanup();
	
public:
	BuilderTiledMesh();
	virtual ~BuilderTiledMesh();
	
	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	
	virtual void setToolStartPos(const float* p);
	virtual void setToolEndPos(const float* p);
	
	virtual void handleRender();
	virtual void handleRenderOverlay(class GLFont* font, double* proj, double* model, int* view);
	virtual void handleMeshChanged(const float* verts, int nverts,
								   const int* tris, const float* trinorms, int ntris,
								   const float* bmin, const float* bmax);
	virtual bool handleBuild();
};


#endif // RECASTBUILDERTILEDMESH_H
