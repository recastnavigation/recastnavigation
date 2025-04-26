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

#pragma once

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "Sample.h"

class NavMeshTesterTool : public SampleTool
{
	Sample* m_sample = nullptr;

	dtNavMesh* m_navMesh = nullptr;
	dtNavMeshQuery* m_navQuery = nullptr;

	dtQueryFilter m_filter;

	dtStatus m_pathFindStatus = DT_FAILURE;

	enum ToolMode
	{
		TOOLMODE_PATHFIND_FOLLOW,
		TOOLMODE_PATHFIND_STRAIGHT,
		TOOLMODE_PATHFIND_SLICED,
		TOOLMODE_RAYCAST,
		TOOLMODE_DISTANCE_TO_WALL,
		TOOLMODE_FIND_POLYS_IN_CIRCLE,
		TOOLMODE_FIND_POLYS_IN_SHAPE,
		TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD
	};
	ToolMode m_toolMode = TOOLMODE_PATHFIND_FOLLOW;

	int m_straightPathOptions = 0;

	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;

	dtPolyRef m_startRef = 0;
	dtPolyRef m_endRef = 0;
	dtPolyRef m_polys[MAX_POLYS];
	dtPolyRef m_parent[MAX_POLYS];
	int m_npolys = 0;
	float m_straightPath[MAX_POLYS * 3];
	unsigned char m_straightPathFlags[MAX_POLYS];
	dtPolyRef m_straightPathPolys[MAX_POLYS];
	int m_nstraightPath = 0;
	float m_polyPickExt[3] = {2, 4, 2};
	float m_smoothPath[MAX_SMOOTH * 3];
	int m_nsmoothPath = 0;
	float m_queryPoly[4 * 3];

	static const int MAX_RAND_POINTS = 64;
	float m_randPoints[MAX_RAND_POINTS * 3];
	int m_nrandPoints = 0;
	bool m_randPointsInCircle = false;

	float m_spos[3];
	float m_epos[3];
	float m_hitPos[3];
	float m_hitNormal[3];
	bool m_hitResult = false;
	float m_distanceToWall = 0;
	float m_neighbourhoodRadius = 2.5f;
	float m_randomRadius = 5.0f;
	bool m_sposSet = false;
	bool m_eposSet = false;

	int m_pathIterNum = 0;
	dtPolyRef m_pathIterPolys[MAX_POLYS];
	int m_pathIterPolyCount = 0;
	float m_prevIterPos[3], m_iterPos[3], m_steerPos[3], m_targetPos[3];

	static const int MAX_STEER_POINTS = 10;
	float m_steerPoints[MAX_STEER_POINTS * 3];
	int m_steerPointCount = 0;

public:
	NavMeshTesterTool();

	SampleToolType type() override { return SampleToolType::NAVMESH_TESTER; }
	void init(Sample* sample) override;
	void reset() override;
	void handleMenu() override;
	void handleClick(const float* s, const float* p, bool shift) override;
	void handleToggle() override;
	void handleStep() override;
	void handleUpdate(const float dt) override;
	void handleRender() override;
	void handleRenderOverlay(double* proj, double* model, int* view) override;

	void recalc();
	void drawAgent(const float* pos, float r, float h, float c, const unsigned int col);
};
