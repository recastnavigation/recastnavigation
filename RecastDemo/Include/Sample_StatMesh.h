#ifndef RECASTSAMPLESTATMESH_H
#define RECASTSAMPLESTATMESH_H

#include "Sample.h"
#include "DetourStatNavMesh.h"
#include "Recast.h"
#include "RecastLog.h"

class Sample_StatMesh : public Sample
{
protected:
	
	dtStatNavMesh* m_navMesh;
	
	enum ToolMode
	{
		TOOLMODE_PATHFIND,
		TOOLMODE_RAYCAST,
		TOOLMODE_DISTANCE_TO_WALL,
		TOOLMODE_FIND_POLYS_AROUND,
	};
	
	ToolMode m_toolMode;
	
	static const int MAX_POLYS = 256;
	
	dtStatPolyRef m_startRef;
	dtStatPolyRef m_endRef;
	dtStatPolyRef m_polys[MAX_POLYS];
	dtStatPolyRef m_parent[MAX_POLYS];
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
	bool m_eposSet;
	
	enum ToolRenderFlags
	{
		NAVMESH_POLYS = 0x01,
		NAVMESH_BVTREE = 0x02,
		NAVMESH_TOOLS = 0x04,
	};
	
	void toolCleanup();
	void toolReset();
	void toolRecalc();
	void toolRender(int flags);
	void toolRenderOverlay(double* proj, double* model, int* view);
	
	void drawAgent(const float* pos, float r, float h, float c, const float* col);


public:
	Sample_StatMesh();
	virtual ~Sample_StatMesh();
	
	virtual void handleTools();
	virtual void setToolStartPos(const float* p);
	virtual void setToolEndPos(const float* p);	
};


#endif // RECASTSAMPLESTATMESH_H
