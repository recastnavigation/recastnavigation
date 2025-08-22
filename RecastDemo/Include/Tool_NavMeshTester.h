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
	static constexpr int MAX_POLYS = 256;
	static constexpr int MAX_SMOOTH = 2048;
	static constexpr int MAX_RAND_POINTS = 64;

	Sample* sample = nullptr;

	dtQueryFilter filter;
	dtStatus pathFindStatus = DT_FAILURE;

	enum class ToolMode : uint8_t
	{
		PATHFIND_FOLLOW,
		PATHFIND_STRAIGHT,
		PATHFIND_SLICED,
		RAYCAST,
		DISTANCE_TO_WALL,
		FIND_POLYS_IN_CIRCLE,
		FIND_POLYS_IN_SHAPE,
		FIND_LOCAL_NEIGHBOURHOOD
	};
	ToolMode toolMode = ToolMode::PATHFIND_FOLLOW;

	int straightPathOptions = 0;

	dtPolyRef startRef = 0;
	dtPolyRef endRef = 0;
	dtPolyRef polys[MAX_POLYS];
	dtPolyRef parent[MAX_POLYS];
	int npolys = 0;
	float straightPath[MAX_POLYS * 3];
	unsigned char straightPathFlags[MAX_POLYS];
	dtPolyRef straightPathPolys[MAX_POLYS];
	int nstraightPath = 0;
	float polyPickExt[3] = {2, 4, 2};

	float smoothPath[MAX_SMOOTH * 3];
	int nsmoothPath = 0;
	float queryPoly[4 * 3];

	float randPoints[MAX_RAND_POINTS * 3];
	int nrandPoints = 0;
	bool randPointsInCircle = false;

	bool sposSet = false;
	float spos[3];
	bool eposSet = false;
	float epos[3];

	float hitPos[3];
	float hitNormal[3];
	bool hitResult = false;
	float distanceToWall = 0;
	float neighbourhoodRadius = 2.5f;
	float randomRadius = 5.0f;

	int pathIterNum = 0;
	dtPolyRef pathIterPolys[MAX_POLYS];
	int pathIterPolyCount = 0;
	float prevIterPos[3];
	float iterPos[3];
	float steerPos[3];
	float targetPos[3];

	static constexpr int MAX_STEER_POINTS = 10;
	float steerPoints[MAX_STEER_POINTS * 3];
	int steerPointCount = 0;

public:
	NavMeshTesterTool();

	SampleToolType type() override { return SampleToolType::NAVMESH_TESTER; }
	void init(Sample* newSample) override;
	void reset() override;
	void drawMenuUI() override;
	void onClick(const float* s, const float* p, bool shift) override;
	void onToggle() override;
	void singleStep() override;
	void update(const float dt) override;
	void render() override;
	void drawOverlayUI(double* proj, double* model, int* view) override;

	void recalc();
	void drawAgent(const float* pos, float r, float h, float c, const unsigned int col) const;
};
