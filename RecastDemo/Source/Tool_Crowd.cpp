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

#include "Tool_Crowd.h"

#include "DetourCommon.h"
#include "DetourDebugDraw.h"
#include "DetourNode.h"
#include "Recast.h"
#include "imguiHelpers.h"

#include <imgui.h>

#ifdef WIN32
#	define snprintf _snprintf
#endif

namespace
{
bool isectSegAABB(const float* sp, const float* sq, const float* amin, const float* amax, float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;

	float d[3];
	dtVsub(d, sq, sp);
	tmin = 0;        // set to -FLT_MAX to get first hit on line
	tmax = FLT_MAX;  // set to max distance ray can travel (for segment)

	// For all three slabs
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (sp[i] < amin[i] || sp[i] > amax[i])
			{
				return false;
			}
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			// Make t1 be intersection with near plane, t2 with far plane
			if (t1 > t2)
			{
				dtSwap(t1, t2);
			}
			// Compute the intersection of slab intersections intervals
			tmin = std::max(t1, tmin);
			tmax = std::min(t2, tmax);
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax)
			{
				return false;
			}
		}
	}

	return true;
}

void getAgentBounds(const dtCrowdAgent* ag, float* bmin, float* bmax)
{
	const float* p = ag->npos;
	const float r = ag->params.radius;
	const float h = ag->params.height;
	bmin[0] = p[0] - r;
	bmin[1] = p[1];
	bmin[2] = p[2] - r;
	bmax[0] = p[0] + r;
	bmax[1] = p[1] + h;
	bmax[2] = p[2] + r;
}
}

CrowdToolState::CrowdToolState()
{
	memset(trails, 0, sizeof(trails));

	obstacleAvoidanceDebugData = dtAllocObstacleAvoidanceDebugData();
	obstacleAvoidanceDebugData->init(2048);

	memset(&agentDebug, 0, sizeof(agentDebug));
	agentDebug.idx = -1;
	agentDebug.vod = obstacleAvoidanceDebugData;
}

CrowdToolState::~CrowdToolState()
{
	dtFreeObstacleAvoidanceDebugData(obstacleAvoidanceDebugData);
}

void CrowdToolState::init(Sample* newSample)
{
	sample = newSample;

	dtNavMesh* navmesh = sample->navMesh;
	if (!navmesh)
	{
		return;
	}

	dtCrowd* crowd = sample->crowd;
	if (!crowd)
	{
		return;
	}

	crowd->init(MAX_AGENTS, sample->agentRadius, navmesh);

	// Make polygons with 'disabled' flag invalid.
	crowd->getEditableFilter(0)->setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);

	// Setup local avoidance params to different qualities.
	dtObstacleAvoidanceParams params;
	// Use mostly default settings, copy from dtCrowd.
	memcpy(&params, crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));

	// Low (11)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 1;
	crowd->setObstacleAvoidanceParams(0, &params);

	// Medium (22)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 2;
	crowd->setObstacleAvoidanceParams(1, &params);

	// Good (45)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 3;
	crowd->setObstacleAvoidanceParams(2, &params);

	// High (66)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 3;
	params.adaptiveDepth = 3;

	crowd->setObstacleAvoidanceParams(3, &params);
}

void CrowdToolState::reset() {}

void CrowdToolState::render()
{
	duDebugDraw& dd = sample->debugDraw;
	const float rad = sample->agentRadius;

	dtNavMesh* nav = sample->navMesh;
	dtCrowd* crowd = sample->crowd;
	if (!nav || !crowd)
	{
		return;
	}

	if (toolParams.showNodes && crowd->getPathQueue())
	{
		const dtNavMeshQuery* navquery = crowd->getPathQueue()->getNavQuery();
		if (navquery)
		{
			duDebugDrawNavMeshNodes(&dd, *navquery);
		}
	}

	dd.depthMask(false);

	// Draw paths
	if (toolParams.showPath)
	{
		for (int i = 0; i < crowd->getAgentCount(); i++)
		{
			if (toolParams.showDetailAll == false && i != agentDebug.idx)
			{
				continue;
			}
			const dtCrowdAgent* ag = crowd->getAgent(i);
			if (!ag->active)
			{
				continue;
			}
			const dtPolyRef* path = ag->corridor.getPath();
			const int npath = ag->corridor.getPathCount();
			for (int j = 0; j < npath; ++j)
			{
				duDebugDrawNavMeshPoly(&dd, *nav, path[j], duRGBA(255, 255, 255, 24));
			}
		}
	}

	if (targetPolyRef)
	{
		duDebugDrawCross(
			&dd,
			targetPosition[0],
			targetPosition[1] + 0.1f,
			targetPosition[2],
			rad,
			duRGBA(255, 255, 255, 192),
			2.0f);
	}

	// Occupancy grid.
	if (toolParams.showGrid)
	{
		float gridy = -FLT_MAX;
		for (int i = 0; i < crowd->getAgentCount(); ++i)
		{
			const dtCrowdAgent* ag = crowd->getAgent(i);
			if (!ag->active)
			{
				continue;
			}
			const float* pos = ag->corridor.getPos();
			gridy = dtMax(gridy, pos[1]);
		}
		gridy += 1.0f;

		dd.begin(DU_DRAW_QUADS);
		const dtProximityGrid* grid = crowd->getGrid();
		const int* bounds = grid->getBounds();
		const float cs = grid->getCellSize();
		for (int y = bounds[1]; y <= bounds[3]; ++y)
		{
			for (int x = bounds[0]; x <= bounds[2]; ++x)
			{
				const int count = grid->getItemCountAt(x, y);
				if (!count)
				{
					continue;
				}
				unsigned int col = duRGBA(128, 0, 0, dtMin(count * 40, 255));
				dd.vertex(static_cast<float>(x) * cs, gridy, static_cast<float>(y) * cs, col);
				dd.vertex(static_cast<float>(x) * cs, gridy, static_cast<float>(y) * cs + cs, col);
				dd.vertex(static_cast<float>(x) * cs + cs, gridy, static_cast<float>(y) * cs + cs, col);
				dd.vertex(static_cast<float>(x) * cs + cs, gridy, static_cast<float>(y) * cs, col);
			}
		}
		dd.end();
	}

	// Trail
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active)
		{
			continue;
		}

		const AgentTrail* trail = &trails[i];
		const float* pos = ag->npos;

		dd.begin(DU_DRAW_LINES, 3.0f);
		float prev[3], preva = 1;
		dtVcopy(prev, pos);
		for (int j = 0; j < AGENT_MAX_TRAIL - 1; ++j)
		{
			const int idx = (trail->htrail + AGENT_MAX_TRAIL - j) % AGENT_MAX_TRAIL;
			const float* v = &trail->trail[idx * 3];
			float a = 1 - j / static_cast<float>(AGENT_MAX_TRAIL);
			dd.vertex(prev[0], prev[1] + 0.1f, prev[2], duRGBA(0, 0, 0, static_cast<int>(128 * preva)));
			dd.vertex(v[0], v[1] + 0.1f, v[2], duRGBA(0, 0, 0, static_cast<int>(128 * a)));
			preva = a;
			dtVcopy(prev, v);
		}
		dd.end();
	}

	// Corners & co
	for (int i = 0; i < crowd->getAgentCount(); i++)
	{
		if (toolParams.showDetailAll == false && i != agentDebug.idx)
		{
			continue;
		}
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active)
		{
			continue;
		}

		const float radius = ag->params.radius;
		const float* pos = ag->npos;

		if (toolParams.showCorners)
		{
			if (ag->ncorners)
			{
				dd.begin(DU_DRAW_LINES, 2.0f);
				for (int j = 0; j < ag->ncorners; ++j)
				{
					const float* va = j == 0 ? pos : &ag->cornerVerts[(j - 1) * 3];
					const float* vb = &ag->cornerVerts[j * 3];
					dd.vertex(va[0], va[1] + radius, va[2], duRGBA(128, 0, 0, 192));
					dd.vertex(vb[0], vb[1] + radius, vb[2], duRGBA(128, 0, 0, 192));
				}
				if (ag->ncorners && ag->cornerFlags[ag->ncorners - 1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
				{
					const float* v = &ag->cornerVerts[(ag->ncorners - 1) * 3];
					dd.vertex(v[0], v[1], v[2], duRGBA(192, 0, 0, 192));
					dd.vertex(v[0], v[1] + radius * 2, v[2], duRGBA(192, 0, 0, 192));
				}

				dd.end();

				if (toolParams.anticipateTurns)
				{
					/*					float dvel[3], pos[3];
					 calcSmoothSteerDirection(ag->pos, ag->cornerVerts, ag->ncorners, dvel);
					 pos[0] = ag->pos[0] + dvel[0];
					 pos[1] = ag->pos[1] + dvel[1];
					 pos[2] = ag->pos[2] + dvel[2];

					 const float off = ag->radius+0.1f;
					 const float* tgt = &ag->cornerVerts[0];
					 const float y = ag->pos[1]+off;

					 dd.begin(DU_DRAW_LINES, 2.0f);

					 dd.vertex(ag->pos[0],y,ag->pos[2], duRGBA(255,0,0,192));
					 dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));

					 dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));
					 dd.vertex(tgt[0],y,tgt[2], duRGBA(255,0,0,192));

					 dd.end();*/
				}
			}
		}

		if (toolParams.showCollisionSegments)
		{
			const float* center = ag->boundary.getCenter();
			duDebugDrawCross(&dd, center[0], center[1] + radius, center[2], 0.2f, duRGBA(192, 0, 128, 255), 2.0f);
			duDebugDrawCircle(
				&dd,
				center[0],
				center[1] + radius,
				center[2],
				ag->params.collisionQueryRange,
				duRGBA(192, 0, 128, 128),
				2.0f);

			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int j = 0; j < ag->boundary.getSegmentCount(); ++j)
			{
				const float* s = ag->boundary.getSegment(j);
				unsigned int col = duRGBA(192, 0, 128, 192);
				if (dtTriArea2D(pos, s, s + 3) < 0.0f)
				{
					col = duDarkenCol(col);
				}

				duAppendArrow(&dd, s[0], s[1] + 0.2f, s[2], s[3], s[4] + 0.2f, s[5], 0.0f, 0.3f, col);
			}
			dd.end();
		}

		if (toolParams.showNeighbors)
		{
			duDebugDrawCircle(
				&dd,
				pos[0],
				pos[1] + radius,
				pos[2],
				ag->params.collisionQueryRange,
				duRGBA(0, 192, 128, 128),
				2.0f);

			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int j = 0; j < ag->nneis; ++j)
			{
				// Get 'n'th active agent.
				// TODO: fix this properly.
				const dtCrowdAgent* nei = crowd->getAgent(ag->neis[j].idx);
				if (nei)
				{
					dd.vertex(pos[0], pos[1] + radius, pos[2], duRGBA(0, 192, 128, 128));
					dd.vertex(nei->npos[0], nei->npos[1] + radius, nei->npos[2], duRGBA(0, 192, 128, 128));
				}
			}
			dd.end();
		}

		if (toolParams.showOpt)
		{
			dd.begin(DU_DRAW_LINES, 2.0f);
			dd.vertex(agentDebug.optStart[0], agentDebug.optStart[1] + 0.3f, agentDebug.optStart[2], duRGBA(0, 128, 0, 192));
			dd.vertex(agentDebug.optEnd[0], agentDebug.optEnd[1] + 0.3f, agentDebug.optEnd[2], duRGBA(0, 128, 0, 192));
			dd.end();
		}
	}

	// Agent cylinders.
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active)
		{
			continue;
		}

		const float radius = ag->params.radius;
		const float* pos = ag->npos;

		unsigned int col = duRGBA(0, 0, 0, 32);
		if (agentDebug.idx == i)
		{
			col = duRGBA(255, 0, 0, 128);
		}

		duDebugDrawCircle(&dd, pos[0], pos[1], pos[2], radius, col, 2.0f);
	}

	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active)
		{
			continue;
		}

		const float height = ag->params.height;
		const float radius = ag->params.radius;
		const float* pos = ag->npos;

		unsigned int col = duRGBA(220, 220, 220, 128);
		if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING || ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
		{
			col = duLerpCol(col, duRGBA(128, 0, 255, 128), 32);
		}
		else if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
		{
			col = duLerpCol(col, duRGBA(128, 0, 255, 128), 128);
		}
		else if (ag->targetState == DT_CROWDAGENT_TARGET_FAILED)
		{
			col = duRGBA(255, 32, 16, 128);
		}
		else if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			col = duLerpCol(col, duRGBA(64, 255, 0, 128), 128);
		}

		duDebugDrawCylinder(
			&dd,
			pos[0] - radius,
			pos[1] + radius * 0.1f,
			pos[2] - radius,
			pos[0] + radius,
			pos[1] + height,
			pos[2] + radius,
			col);
	}

	if (toolParams.showVO)
	{
		for (int i = 0; i < crowd->getAgentCount(); i++)
		{
			if (toolParams.showDetailAll == false && i != agentDebug.idx)
			{
				continue;
			}
			const dtCrowdAgent* ag = crowd->getAgent(i);
			if (!ag->active)
			{
				continue;
			}

			// Draw detail about agent sela
			const dtObstacleAvoidanceDebugData* vod = agentDebug.vod;

			const float dx = ag->npos[0];
			const float dy = ag->npos[1] + ag->params.height;
			const float dz = ag->npos[2];

			duDebugDrawCircle(&dd, dx, dy, dz, ag->params.maxSpeed, duRGBA(255, 255, 255, 64), 2.0f);

			dd.begin(DU_DRAW_QUADS);
			for (int j = 0; j < vod->getSampleCount(); ++j)
			{
				const float* p = vod->getSampleVelocity(j);
				const float sr = vod->getSampleSize(j);
				const float pen = vod->getSamplePenalty(j);
				const float pen2 = vod->getSamplePreferredSidePenalty(j);
				unsigned int col = duLerpCol(duRGBA(255, 255, 255, 220), duRGBA(128, 96, 0, 220), (int)(pen * 255));
				col = duLerpCol(col, duRGBA(128, 0, 0, 220), (int)(pen2 * 128));
				dd.vertex(dx + p[0] - sr, dy, dz + p[2] - sr, col);
				dd.vertex(dx + p[0] - sr, dy, dz + p[2] + sr, col);
				dd.vertex(dx + p[0] + sr, dy, dz + p[2] + sr, col);
				dd.vertex(dx + p[0] + sr, dy, dz + p[2] - sr, col);
			}
			dd.end();
		}
	}

	// Velocity stuff.
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active)
		{
			continue;
		}

		const float radius = ag->params.radius;
		const float height = ag->params.height;
		const float* pos = ag->npos;
		const float* vel = ag->vel;
		const float* dvel = ag->dvel;

		unsigned int col = duRGBA(220, 220, 220, 192);
		if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING || ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
		{
			col = duLerpCol(col, duRGBA(128, 0, 255, 192), 32);
		}
		else if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
		{
			col = duLerpCol(col, duRGBA(128, 0, 255, 192), 128);
		}
		else if (ag->targetState == DT_CROWDAGENT_TARGET_FAILED)
		{
			col = duRGBA(255, 32, 16, 192);
		}
		else if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			col = duLerpCol(col, duRGBA(64, 255, 0, 192), 128);
		}

		duDebugDrawCircle(&dd, pos[0], pos[1] + height, pos[2], radius, col, 2.0f);

		duDebugDrawArrow(
			&dd,
			pos[0],
			pos[1] + height,
			pos[2],
			pos[0] + dvel[0],
			pos[1] + height + dvel[1],
			pos[2] + dvel[2],
			0.0f,
			0.4f,
			duRGBA(0, 192, 255, 192),
			(agentDebug.idx == i) ? 2.0f : 1.0f);

		duDebugDrawArrow(
			&dd,
			pos[0],
			pos[1] + height,
			pos[2],
			pos[0] + vel[0],
			pos[1] + height + vel[1],
			pos[2] + vel[2],
			0.0f,
			0.4f,
			duRGBA(0, 0, 0, 160),
			2.0f);
	}

	dd.depthMask(true);
}

void CrowdToolState::renderOverlay()
{
	// Draw start and end point labels
	if (targetPolyRef)
	{
		DrawWorldspaceText(targetPosition[0], targetPosition[1], targetPosition[2], IM_COL32(0, 0, 0, 220), "TARGET", true);
	}

	char label[32];

	if (toolParams.showNodes)
	{
		dtCrowd* crowd = sample->crowd;
		if (crowd && crowd->getPathQueue())
		{
			const dtNavMeshQuery* navquery = crowd->getPathQueue()->getNavQuery();
			const dtNodePool* pool = navquery->getNodePool();
			if (pool)
			{
				for (int i = 0; i < pool->getHashSize(); ++i)
				{
					for (dtNodeIndex j = pool->getFirst(i); j != DT_NULL_IDX; j = pool->getNext(j))
					{
						const dtNode* node = pool->getNodeAtIdx(j + 1);
						if (!node)
						{
							continue;
						}

						snprintf(label, 32, "%.2f", node->total);
						DrawWorldspaceText(
							node->pos[0],
							node->pos[1] + 0.5f,
							node->pos[2],
							IM_COL32(0, 0, 0, 220),
							label,
							true,
							15);
					}
				}
			}
		}
	}

	if (toolParams.showLabels)
	{
		dtCrowd* crowd = sample->crowd;
		if (crowd)
		{
			for (int i = 0; i < crowd->getAgentCount(); ++i)
			{
				const dtCrowdAgent* agent = crowd->getAgent(i);
				if (!agent->active)
				{
					continue;
				}

				snprintf(label, 32, "%d", i);
				DrawWorldspaceText(
					agent->npos[0],
					agent->npos[1] + agent->params.height,
					agent->npos[2],
					IM_COL32(0, 0, 0, 220),
					label,
					true,
					15);
			}
		}
	}
	if (agentDebug.idx != -1)
	{
		dtCrowd* crowd = sample->crowd;
		if (crowd)
		{
			for (int i = 0; i < crowd->getAgentCount(); i++)
			{
				if (toolParams.showDetailAll == false && i != agentDebug.idx)
				{
					continue;
				}
				const dtCrowdAgent* agent = crowd->getAgent(i);
				if (!agent->active)
				{
					continue;
				}

				if (toolParams.showNeighbors)
				{
					for (int neighborIndex = 0; neighborIndex < agent->nneis; ++neighborIndex)
					{
						const dtCrowdNeighbour& neighborData = agent->neis[neighborIndex];
						const dtCrowdAgent* neighbor = crowd->getAgent(neighborData.idx);
						if (!neighbor->active)
						{
							continue;
						}

						snprintf(label, 32, "%.3f", neighborData.dist);
						DrawWorldspaceText(
							neighbor->npos[0],
							neighbor->npos[1] + agent->params.radius,
							neighbor->npos[2],
							IM_COL32(255, 255, 255, 220),
							label,
							true,
							15);
					}
				}
			}
		}
	}

	if (toolParams.showPerfGraph)
	{
		GraphParams gp;
		gp.setRect(300, 10, 500, 200, 8);
		gp.setValueRange(0.0f, 2.0f, 4, "ms");

		drawGraphBackground(&gp);
		drawGraph(&gp, &crowdTotalTime, 1, "Total", duRGBA(255, 128, 0, 255));

		gp.setRect(300, 10, 500, 50, 8);
		gp.setValueRange(0.0f, 2000.0f, 1, "");
		drawGraph(&gp, &crowdSampleCount, 0, "Sample Count", duRGBA(96, 96, 96, 128));
	}
}

void CrowdToolState::update(const float dt)
{
	if (run)
	{
		updateTick(dt);
	}
}

void CrowdToolState::addAgent(const float* p)
{
	if (!sample)
	{
		return;
	}
	dtCrowd* crowd = sample->crowd;

	dtCrowdAgentParams ap;
	memset(&ap, 0, sizeof(ap));
	ap.radius = sample->agentRadius;
	ap.height = sample->agentHeight;
	ap.maxAcceleration = 8.0f;
	ap.maxSpeed = 3.5f;
	ap.collisionQueryRange = ap.radius * 12.0f;
	ap.pathOptimizationRange = ap.radius * 30.0f;
	ap.updateFlags = 0;
	if (toolParams.anticipateTurns)
	{
		ap.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
	}
	if (toolParams.optimizeVis)
	{
		ap.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
	}
	if (toolParams.optimizeTopo)
	{
		ap.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
	}
	if (toolParams.obstacleAvoidance)
	{
		ap.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	}
	if (toolParams.separation)
	{
		ap.updateFlags |= DT_CROWD_SEPARATION;
	}
	ap.obstacleAvoidanceType = static_cast<unsigned char>(toolParams.obstacleAvoidanceType);
	ap.separationWeight = toolParams.separationWeight;

	int idx = crowd->addAgent(p, &ap);
	if (idx != -1)
	{
		if (targetPolyRef)
		{
			crowd->requestMoveTarget(idx, targetPolyRef, targetPosition);
		}

		// Init trail
		AgentTrail* trail = &trails[idx];
		for (int i = 0; i < AGENT_MAX_TRAIL; ++i)
		{
			dtVcopy(&trail->trail[i * 3], p);
		}
		trail->htrail = 0;
	}
}

void CrowdToolState::removeAgent(const int idx)
{
	if (!sample)
	{
		return;
	}
	dtCrowd* crowd = sample->crowd;

	crowd->removeAgent(idx);

	if (idx == agentDebug.idx)
	{
		agentDebug.idx = -1;
	}
}

void CrowdToolState::highlightAgent(const int idx)
{
	agentDebug.idx = idx;
}

static void calcVel(float* vel, const float* pos, const float* tgt, const float speed)
{
	dtVsub(vel, tgt, pos);
	vel[1] = 0.0;
	dtVnormalize(vel);
	dtVscale(vel, vel, speed);
}

void CrowdToolState::setMoveTarget(const float* p, bool adjust)
{
	if (!sample)
	{
		return;
	}

	// Find nearest point on navmesh and set move request to that location.
	dtNavMeshQuery* navquery = sample->navQuery;
	dtCrowd* crowd = sample->crowd;
	const dtQueryFilter* filter = crowd->getFilter(0);
	const float* halfExtents = crowd->getQueryExtents();

	if (adjust)
	{
		float vel[3];
		// Request velocity
		if (agentDebug.idx != -1)
		{
			const dtCrowdAgent* ag = crowd->getAgent(agentDebug.idx);
			if (ag && ag->active)
			{
				calcVel(vel, ag->npos, p, ag->params.maxSpeed);
				crowd->requestMoveVelocity(agentDebug.idx, vel);
			}
		}
		else
		{
			for (int i = 0; i < crowd->getAgentCount(); ++i)
			{
				const dtCrowdAgent* ag = crowd->getAgent(i);
				if (!ag->active)
				{
					continue;
				}
				calcVel(vel, ag->npos, p, ag->params.maxSpeed);
				crowd->requestMoveVelocity(i, vel);
			}
		}
	}
	else
	{
		navquery->findNearestPoly(p, halfExtents, filter, &targetPolyRef, targetPosition);

		if (agentDebug.idx != -1)
		{
			const dtCrowdAgent* ag = crowd->getAgent(agentDebug.idx);
			if (ag && ag->active)
			{
				crowd->requestMoveTarget(agentDebug.idx, targetPolyRef, targetPosition);
			}
		}
		else
		{
			for (int i = 0; i < crowd->getAgentCount(); ++i)
			{
				const dtCrowdAgent* ag = crowd->getAgent(i);
				if (!ag->active)
				{
					continue;
				}
				crowd->requestMoveTarget(i, targetPolyRef, targetPosition);
			}
		}
	}
}

int CrowdToolState::hitTestAgents(const float* s, const float* p)
{
	if (!sample)
	{
		return -1;
	}
	dtCrowd* crowd = sample->crowd;

	int isel = -1;
	float tsel = FLT_MAX;

	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active)
		{
			continue;
		}
		float bmin[3], bmax[3];
		getAgentBounds(ag, bmin, bmax);
		float tmin, tmax;
		if (isectSegAABB(s, p, bmin, bmax, tmin, tmax))
		{
			if (tmin > 0 && tmin < tsel)
			{
				isel = i;
				tsel = tmin;
			}
		}
	}

	return isel;
}

void CrowdToolState::updateAgentParams()
{
	if (!sample)
	{
		return;
	}
	dtCrowd* crowd = sample->crowd;
	if (!crowd)
	{
		return;
	}

	unsigned char updateFlags = 0;
	updateFlags |= toolParams.anticipateTurns ? DT_CROWD_ANTICIPATE_TURNS : 0;
	updateFlags |= toolParams.optimizeVis ? DT_CROWD_OPTIMIZE_VIS : 0;
	updateFlags |= toolParams.optimizeTopo ? DT_CROWD_OPTIMIZE_TOPO : 0;
	updateFlags |= toolParams.obstacleAvoidance ? DT_CROWD_OBSTACLE_AVOIDANCE : 0;
	updateFlags |= toolParams.separation ? DT_CROWD_SEPARATION : 0;

	unsigned char obstacleAvoidanceType = static_cast<unsigned char>(toolParams.obstacleAvoidanceType);

	dtCrowdAgentParams params;

	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active)
		{
			continue;
		}
		memcpy(&params, &ag->params, sizeof(dtCrowdAgentParams));
		params.updateFlags = updateFlags;
		params.obstacleAvoidanceType = obstacleAvoidanceType;
		params.separationWeight = toolParams.separationWeight;
		crowd->updateAgentParameters(i, &params);
	}
}

void CrowdToolState::updateTick(const float dt)
{
	if (!sample)
	{
		return;
	}
	dtNavMesh* nav = sample->navMesh;
	dtCrowd* crowd = sample->crowd;
	if (!nav || !crowd)
	{
		return;
	}

	TimeVal startTime = getPerfTime();

	crowd->update(dt, &agentDebug);

	TimeVal endTime = getPerfTime();

	// Update agent trails
	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		AgentTrail* trail = &trails[i];
		if (!ag->active)
		{
			continue;
		}
		// Update agent movement trail.
		trail->htrail = (trail->htrail + 1) % AGENT_MAX_TRAIL;
		dtVcopy(&trail->trail[trail->htrail * 3], ag->npos);
	}

	agentDebug.vod->normalizeSamples();

	crowdSampleCount.addSample((float)crowd->getVelocitySampleCount());
	crowdTotalTime.addSample(getPerfTimeUsec(endTime - startTime) / 1000.0f);
}

void CrowdTool::init(Sample* newSample)
{
	sample = newSample;
	if (!sample)
	{
		return;
	}

	state = static_cast<CrowdToolState*>(sample->toolStates[static_cast<int>(type())]);
	if (!state)
	{
		state = new CrowdToolState();
		sample->toolStates[static_cast<int>(type())] = state;
	}
	state->init(sample);
}

void CrowdTool::reset() {}

void CrowdTool::drawMenuUI()
{
	if (!state)
	{
		return;
	}

	if (ImGui::RadioButton("Create Agents", mode == ToolMode::CREATE))
	{
		mode = ToolMode::CREATE;
	}
	if (ImGui::RadioButton("Move Target", mode == ToolMode::MOVE_TARGET))
	{
		mode = ToolMode::MOVE_TARGET;
	}
	if (ImGui::RadioButton("Select Agent", mode == ToolMode::SELECT))
	{
		mode = ToolMode::SELECT;
	}
	if (ImGui::RadioButton("Toggle Polys", mode == ToolMode::TOGGLE_POLYS))
	{
		mode = ToolMode::TOGGLE_POLYS;
	}

	ImGui::Separator();

	CrowdToolParams* params = state->getToolParams();

	if (ImGui::TreeNode("Options"))
	{
		bool paramsChanged = false;
		paramsChanged |= ImGui::Checkbox("Optimize Visibility", &params->optimizeVis);
		paramsChanged |= ImGui::Checkbox("Optimize Topology", &params->optimizeTopo);
		paramsChanged |= ImGui::Checkbox("Anticipate Turns", &params->anticipateTurns);
		paramsChanged |= ImGui::Checkbox("Obstacle Avoidance", &params->obstacleAvoidance);
		paramsChanged |= ImGui::SliderInt("Avoidance Quality", &params->obstacleAvoidanceType, 0, 3);
		paramsChanged |= ImGui::Checkbox("Separation", &params->separation);
		paramsChanged |= ImGui::SliderFloat("##Separation Weight", &params->separationWeight, 0.0f, 20.0f, "Separation Weight = %.2f");

		if (paramsChanged)
		{
			state->updateAgentParams();
		}
		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Selected Debug Draw"))
	{
		ImGui::Checkbox("Show Corners", &params->showCorners);
		ImGui::Checkbox("Show Collision Segments", &params->showCollisionSegments);
		ImGui::Checkbox("Show Path", &params->showPath);
		ImGui::Checkbox("Show VO", &params->showVO);
		ImGui::Checkbox("Show Path Optimization", &params->showOpt);
		ImGui::Checkbox("Show Neighbours", &params->showNeighbors);
		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Debug Draw"))
	{
		ImGui::Checkbox("Show Labels", &params->showLabels);
		ImGui::Checkbox("Show Prox Grid", &params->showGrid);
		ImGui::Checkbox("Show Nodes", &params->showNodes);
		ImGui::Checkbox("Show Perf Graph", &params->showPerfGraph);
		ImGui::Checkbox("Show Detail All", &params->showDetailAll);
		ImGui::TreePop();
	}
}

void CrowdTool::onClick(const float* s, const float* p, bool shift)
{
	if (!sample)
	{
		return;
	}
	if (!state)
	{
		return;
	}
	InputGeom* geom = sample->inputGeometry;
	if (!geom)
	{
		return;
	}
	dtCrowd* crowd = sample->crowd;
	if (!crowd)
	{
		return;
	}

	if (mode == ToolMode::CREATE)
	{
		if (shift)
		{
			// Delete
			int ahit = state->hitTestAgents(s, p);
			if (ahit != -1)
			{
				state->removeAgent(ahit);
			}
		}
		else
		{
			// Add
			state->addAgent(p);
		}
	}
	else if (mode == ToolMode::MOVE_TARGET)
	{
		state->setMoveTarget(p, shift);
	}
	else if (mode == ToolMode::SELECT)
	{
		// Highlight
		int ahit = state->hitTestAgents(s, p);
		state->highlightAgent(ahit);
	}
	else if (mode == ToolMode::TOGGLE_POLYS)
	{
		dtNavMesh* nav = sample->navMesh;
		dtNavMeshQuery* navquery = sample->navQuery;
		if (nav && navquery)
		{
			dtQueryFilter filter;
			const float* halfExtents = crowd->getQueryExtents();
			float tgt[3];
			dtPolyRef ref;
			navquery->findNearestPoly(p, halfExtents, &filter, &ref, tgt);
			if (ref)
			{
				unsigned short flags = 0;
				if (dtStatusSucceed(nav->getPolyFlags(ref, &flags)))
				{
					flags ^= SAMPLE_POLYFLAGS_DISABLED;
					nav->setPolyFlags(ref, flags);
				}
			}
		}
	}
}

void CrowdTool::singleStep()
{
	if (!state)
	{
		return;
	}

	const float dt = 1.0f / 20.0f;
	state->updateTick(dt);

	state->setRunning(false);
}

void CrowdTool::onToggle()
{
	if (!state)
	{
		return;
	}
	state->setRunning(!state->isRunning());
}

void CrowdTool::update(const float dt)
{
	rcIgnoreUnused(dt);
}

void CrowdTool::render() {}

void CrowdTool::drawOverlayUI()
{
	// Tool help
	if (mode == ToolMode::CREATE)
	{
		DrawScreenspaceText(280, 40, IM_COL32(255, 255, 255, 192), "LMB: add agent.  Shift+LMB: remove agent.");
	}
	else if (mode == ToolMode::MOVE_TARGET)
	{
		DrawScreenspaceText(280, 40, IM_COL32(255, 255, 255, 192), "LMB: set move target.  Shift+LMB: adjust set velocity.");
		DrawScreenspaceText(280, 60, IM_COL32(255, 255, 255, 192), "Setting velocity will move the agents without pathfinder.");
	}
	else if (mode == ToolMode::SELECT)
	{
		DrawScreenspaceText(280, 40, IM_COL32(255, 255, 255, 192), "LMB: select agent.");
	}

	DrawScreenspaceText(280, 60, IM_COL32(255, 255, 255, 192), "SPACE: Run/Pause simulation.  1: Step simulation.");

	if (state && state->isRunning())
	{
		DrawScreenspaceText(280, 80, IM_COL32(255, 32, 16, 255), "- RUNNING -");
	}
	else
	{
		DrawScreenspaceText(280, 80, IM_COL32(255, 255, 255, 128), "- PAUSED -");
	}
}
