#ifndef RECASTSAMPLESOLOMESH_H
#define RECASTSAMPLESOLOMESH_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "Recast.h"
#include "RecastLog.h"

class Sample_SoloMesh : public Sample
{
protected:
	
	dtNavMesh* m_navMesh;
	
	enum ToolMode
	{
		TOOLMODE_PATHFIND,
		TOOLMODE_RAYCAST,
		TOOLMODE_DISTANCE_TO_WALL,
		TOOLMODE_FIND_POLYS_AROUND,
	};
	
	ToolMode m_toolMode;
	
	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;
	
	dtPolyRef m_startRef;
	dtPolyRef m_endRef;
	dtPolyRef m_polys[MAX_POLYS];
	dtPolyRef m_parent[MAX_POLYS];
	int m_npolys;
	float m_straightPath[MAX_POLYS*3];
	int m_nstraightPath;
	float m_polyPickExt[3];
	float m_smoothPath[MAX_SMOOTH*3];
	int m_nsmoothPath;
	
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
	Sample_SoloMesh();
	virtual ~Sample_SoloMesh();
	
	virtual void handleTools();
	virtual void setToolStartPos(const float* p);
	virtual void setToolEndPos(const float* p);	
};


#endif // RECASTSAMPLESOLOMESH_H
