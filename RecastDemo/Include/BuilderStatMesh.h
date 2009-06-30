#ifndef RECASTBUILDERSTATMESH_H
#define RECASTBUILDERSTATMESH_H

#include "Builder.h"
#include "DetourStatNavMesh.h"
#include "Recast.h"
#include "RecastLog.h"

class BuilderStatMesh : public Builder
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
	
	dtPolyRef m_startRef;
	dtPolyRef m_endRef;
	dtPolyRef m_polys[MAX_POLYS];
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
	
	void drawAgent(const float* pos, float r, float h, float c, const float* col);

public:
	BuilderStatMesh();
	virtual ~BuilderStatMesh();
	
	virtual void handleTools();
	virtual void setToolStartPos(const float* p);
	virtual void setToolEndPos(const float* p);	
};


#endif // RECASTBUILDERSTATMESH_H
