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

#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include <float.h>
#include <new>
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "DetourAssert.h"
#include "DetourAlloc.h"

static const int VO_ADAPTIVE_DIVS = 7;
static const int VO_ADAPTIVE_RINGS = 2;
static const int VO_ADAPTIVE_DEPTH = 5;

static const int VO_GRID_SIZE = 33;

static const int MAX_ITERS_PER_UPDATE = 10;

static const int MAX_PATHQUEUE_NODES = 4096;
static const int MAX_COMMON_NODES = 512;


static void integrate(dtCrowdAgent* ag, const float dt)
{
	// Fake dynamic constraint.
	const float maxDelta = ag->maxAcceleration * dt;
	float dv[3];
	dtVsub(dv, ag->nvel, ag->vel);
	float ds = dtVlen(dv);
	if (ds > maxDelta)
		dtVscale(dv, dv, maxDelta/ds);
	dtVadd(ag->vel, ag->vel, dv);
	
	// Integrate
	if (dtVlen(ag->vel) > 0.0001f)
		dtVmad(ag->npos, ag->npos, ag->vel, dt);
	else
		dtVset(ag->vel,0,0,0);
}

static float getDistanceToGoal(const dtCrowdAgent* ag, const float range)
{
	if (!ag->ncorners)
		return range;
	
	const bool endOfPath = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
	const bool offMeshConnection = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
	if (endOfPath || offMeshConnection)
		return dtMin(dtVdist2D(ag->npos, &ag->cornerVerts[(ag->ncorners-1)*3]), range);
	
	return range;
}

static void calcSmoothSteerDirection(const dtCrowdAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	
	const int ip0 = 0;
	const int ip1 = dtMin(1, ag->ncorners-1);
	const float* p0 = &ag->cornerVerts[ip0*3];
	const float* p1 = &ag->cornerVerts[ip1*3];
	
	float dir0[3], dir1[3];
	dtVsub(dir0, p0, ag->npos);
	dtVsub(dir1, p1, ag->npos);
	dir0[1] = 0;
	dir1[1] = 0;
	
	float len0 = dtVlen(dir0);
	float len1 = dtVlen(dir1);
	if (len1 > 0.001f)
		dtVscale(dir1,dir1,1.0f/len1);
	
	dir[0] = dir0[0] - dir1[0]*len0*0.5f;
	dir[1] = 0;
	dir[2] = dir0[2] - dir1[2]*len0*0.5f;
	
	dtVnormalize(dir);
}

static void calcStraightSteerDirection(const dtCrowdAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	dtVsub(dir, &ag->cornerVerts[0], ag->npos);
	dir[1] = 0;
	dtVnormalize(dir);
}

static int addNeighbour(const int idx, const float dist,
						dtCrowdNeighbour* neis, const int nneis, const int maxNeis)
{
	// Insert neighbour based on the distance.
	dtCrowdNeighbour* nei = 0;
	if (!nneis)
	{
		nei = &neis[nneis];
	}
	else if (dist >= neis[nneis-1].dist)
	{
		if (nneis >= maxNeis)
			return nneis;
		nei = &neis[nneis];
	}
	else
	{
		int i;
		for (i = 0; i < nneis; ++i)
			if (dist <= neis[i].dist)
				break;
		
		const int tgt = i+1;
		const int n = dtMin(nneis-i, maxNeis-tgt);
		
		dtAssert(tgt+n <= maxNeis);
		
		if (n > 0)
			memmove(&neis[tgt], &neis[i], sizeof(dtCrowdNeighbour)*n);
		nei = &neis[i];
	}
	
	memset(nei, 0, sizeof(dtCrowdNeighbour));
	
	nei->idx = idx;
	nei->dist = dist;
	
	return dtMin(nneis+1, maxNeis);
}



dtCrowd::dtCrowd() :
	m_maxAgents(0),
	m_agents(0),
	m_activeAgents(0),
	m_obstacleQuery(0),
	m_grid(0),
	m_pathResult(0),
	m_maxPathResult(0),
	m_maxAgentRadius(0),
	m_velocitySampleCount(0),
	m_moveRequests(0),
	m_moveRequestCount(0),
	m_navquery(0)
{
}

dtCrowd::~dtCrowd()
{
	purge();
}

void dtCrowd::purge()
{
	for (int i = 0; i < m_maxAgents; ++i)
		m_agents[i].~dtCrowdAgent();
	dtFree(m_agents);
	m_agents = 0;
	m_maxAgents = 0;
	
	dtFree(m_activeAgents);
	m_activeAgents = 0;
	
	dtFree(m_pathResult);
	m_pathResult = 0;
	
	dtFree(m_moveRequests);
	m_moveRequests = 0;
	
	dtFreeProximityGrid(m_grid);
	m_grid = 0;

	dtFreeObstacleAvoidanceQuery(m_obstacleQuery);
	m_obstacleQuery = 0;
	
	dtFreeNavMeshQuery(m_navquery);
	m_navquery = 0;
}

bool dtCrowd::init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav)
{
	m_maxAgents = maxAgents;
	m_maxAgentRadius = maxAgentRadius;

	dtVset(m_ext, m_maxAgentRadius*2.0f,m_maxAgentRadius*1.5f,m_maxAgentRadius*2.0f);
	
	m_grid = dtAllocProximityGrid();
	if (!m_grid)
		return false;
	if (!m_grid->init(m_maxAgents*4, maxAgentRadius*3))
		return false;
	
	m_obstacleQuery = dtAllocObstacleAvoidanceQuery();
	if (!m_obstacleQuery)
		return false;
	m_obstacleQuery->init(6, 8);
	
	m_obstacleQuery->setDesiredVelocityWeight(2.0f);
	m_obstacleQuery->setCurrentVelocityWeight(0.75f);
	m_obstacleQuery->setPreferredSideWeight(0.75f);
	m_obstacleQuery->setCollisionTimeWeight(2.5f);
	m_obstacleQuery->setTimeHorizon(2.5f);
	m_obstacleQuery->setVelocitySelectionBias(0.4f);
	
	// Allocate temp buffer for merging paths.
	m_maxPathResult = 256;
	m_pathResult = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*m_maxPathResult, DT_ALLOC_PERM);
	if (!m_pathResult)
		return false;
	
	m_moveRequests = (MoveRequest*)dtAlloc(sizeof(MoveRequest)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_moveRequests)
		return false;
	m_moveRequestCount = 0;
	
	if (!m_pathq.init(m_maxPathResult, MAX_PATHQUEUE_NODES, nav))
		return false;
	
	m_agents = (dtCrowdAgent*)dtAlloc(sizeof(dtCrowdAgent)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_agents)
		return false;
	
	m_activeAgents = (dtCrowdAgent**)dtAlloc(sizeof(dtCrowdAgent*)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_activeAgents)
		return false;
	
	for (int i = 0; i < m_maxAgents; ++i)
	{
		new(&m_agents[i]) dtCrowdAgent();
		m_agents[i].active = 0;
		if (!m_agents[i].corridor.init(m_maxPathResult))
			return false;
	}

	// The navquery is mostly used for local searches, no need for large node pool.
	m_navquery = dtAllocNavMeshQuery();
	if (!m_navquery)
		return false;
	if (dtStatusFailed(m_navquery->init(nav, MAX_COMMON_NODES)))
		return false;
	
	return true;
}

const int dtCrowd::getAgentCount() const
{
	return m_maxAgents;
}

const dtCrowdAgent* dtCrowd::getAgent(const int idx)
{
	return &m_agents[idx];
}

int dtCrowd::addAgent(const float* pos, const dtCrowdAgentParams* params)
{
	// Find empty slot.
	int idx = -1;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active)
		{
			idx = i;
			break;
		}
	}
	if (idx == -1)
		return -1;
	
	dtCrowdAgent* ag = &m_agents[idx];

	// Find nearest position on navmesh and place the agent there.
	float nearest[3];
	dtPolyRef ref;
	m_navquery->findNearestPoly(pos, m_ext, &m_filter, &ref, nearest);
	if (!ref)
	{
		// Could not find a location on navmesh.
		return -1;
	}
	
	ag->corridor.reset(ref, nearest);
	ag->boundary.reset();

	ag->radius = params->radius;
	ag->height = params->height;
	ag->maxAcceleration = params->maxAcceleration;
	ag->maxSpeed = params->maxSpeed;
	ag->collisionQueryRange = params->collisionQueryRange;
	ag->pathOptimizationRange = params->pathOptimizationRange;
	
	ag->topologyOptTime = 0;
	ag->nneis = 0;
	
	dtVset(ag->dvel, 0,0,0);
	dtVset(ag->nvel, 0,0,0);
	dtVset(ag->vel, 0,0,0);
	dtVcopy(ag->npos, nearest);
	
	ag->desiredSpeed = 0;
	ag->t = 0;
	ag->var = (rand() % 10) / 9.0f;

	ag->active = 1;

	return idx;
}

void dtCrowd::removeAgent(const int idx)
{
	if (idx >= 0 && idx < m_maxAgents)
	{
		m_agents[idx].active = 0;
	}
}

bool dtCrowd::requestMoveTarget(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;
	if (!ref)
		return false;
	
	MoveRequest* req = 0;
	// Check if there is existing request and update that instead.
	for (int i = 0; i < m_moveRequestCount; ++i)
	{
		if (m_moveRequests[i].idx == idx)
		{
			req = &m_moveRequests[i];
			break;
		}
	}
	if (!req)
	{
		if (m_moveRequestCount >= m_maxAgents)
			return false;
		req = &m_moveRequests[m_moveRequestCount++];
		memset(req, 0, sizeof(MoveRequest));
	}
	
	// Initialize request.
	req->idx = idx;
	req->ref = ref;
	dtVcopy(req->pos, pos);
	req->pathqRef = DT_PATHQ_INVALID;
	req->state = MR_TARGET_REQUESTING;
	
	req->temp[0] = ref;
	req->ntemp = 1;

	return true;
}

bool dtCrowd::adjustMoveTarget(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx > m_maxAgents)
		return false;
	if (!ref)
		return false;
	
	MoveRequest* req = 0;
	// Check if there is existing request and update that instead.
	for (int i = 0; i < m_moveRequestCount; ++i)
	{
		if (m_moveRequests[i].idx == idx)
		{
			req = &m_moveRequests[i];
			break;
		}
	}
	if (!req)
	{
		if (m_moveRequestCount >= m_maxAgents)
			return false;
		req = &m_moveRequests[m_moveRequestCount++];
		memset(req, 0, sizeof(MoveRequest));

		// New adjust request
		req->state = MR_TARGET_ADJUST;
		req->idx = idx;
	}

	// Set adjustment request.
	req->aref = ref;
	dtVcopy(req->apos, pos);

	return true;
}

int dtCrowd::getActiveAgents(dtCrowdAgent** agents, const int maxAgents)
{
	int n = 0;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active) continue;
		if (n < maxAgents)
			agents[n++] = &m_agents[i];
	}
	return n;
}



int dtCrowd::getNeighbours(const float* pos, const float height, const float range,
								const dtCrowdAgent* skip, dtCrowdNeighbour* result, const int maxResult)
{
	int n = 0;
	
	static const int MAX_NEIS = 32;
	unsigned short ids[MAX_NEIS];
	int nids = m_grid->queryItems(pos[0]-range, pos[2]-range,
								  pos[0]+range, pos[2]+range,
								  ids, MAX_NEIS);
	
	for (int i = 0; i < nids; ++i)
	{
		dtCrowdAgent* ag = &m_agents[ids[i]];

		if (ag == skip) continue;

		// Check for overlap.
		float diff[3];
		dtVsub(diff, pos, ag->npos);
		if (fabsf(diff[1]) >= (height+ag->height)/2.0f)
			continue;
		diff[1] = 0;
		const float distSqr = dtVlenSqr(diff);
		if (distSqr > dtSqr(range))
			continue;
		
		n = addNeighbour(ids[i], distSqr, result, n, maxResult);
	}
	return n;
}

void dtCrowd::updateMoveRequest(const float dt)
{
	// Fire off new requests.
	for (int i = 0; i < m_moveRequestCount; ++i)
	{
		MoveRequest* req = &m_moveRequests[i];
		dtCrowdAgent* ag = &m_agents[req->idx];
		
		// Agent not active anymore, kill request.
		if (!ag->active)
			req->state = MR_TARGET_FAILED;
		
		// Adjust target
		if (req->aref)
		{
			if (req->state == MR_TARGET_ADJUST)
			{
				// Adjust existing path.
				ag->corridor.moveTargetPosition(req->apos, m_navquery, &m_filter);
				req->state = MR_TARGET_VALID;
			}
			else
			{
				// Adjust on the flight request.
				float result[3];
				static const int MAX_VISITED = 16;
				dtPolyRef visited[MAX_VISITED];
				int nvisited = 0;
				m_navquery->moveAlongSurface(req->temp[req->ntemp-1], req->pos, req->apos, &m_filter,
											 result, visited, &nvisited, MAX_VISITED);
				req->ntemp = dtMergeCorridorEndMoved(req->temp, req->ntemp, MAX_TEMP_PATH, visited, nvisited);
				dtVcopy(req->pos, result);
				
				// Reset adjustment.
				dtVset(req->apos, 0,0,0);
				req->aref = 0;
			}
		}
		
		
		if (req->state == MR_TARGET_REQUESTING)
		{
			// Calculate request position.
			// If there is a lot of latency between requests, it is possible to
			// project the current position ahead and use raycast to find the actual
			// location and path.
			const dtPolyRef* path = ag->corridor.getPath();
			const int npath = ag->corridor.getPathCount();
			dtAssert(npath);
			
			// Here we take the simple approach and set the path to be just the current location.
			float reqPos[3];
			dtVcopy(reqPos, ag->corridor.getPos());	// The location of the request
			dtPolyRef reqPath[8];					// The path to the request location
			reqPath[0] = path[0];
			int reqPathCount = 1;
			
			req->pathqRef = m_pathq.request(reqPath[reqPathCount-1], req->ref, reqPos, req->pos, &m_filter);
			if (req->pathqRef != DT_PATHQ_INVALID)
			{
				ag->corridor.setCorridor(reqPos, reqPath, reqPathCount);
				req->state = MR_TARGET_WAITING_FOR_PATH;
			}
		}
	}

	
	// Update requests.
	m_pathq.update(MAX_ITERS_PER_UPDATE);

	// Process path results.
	for (int i = 0; i < m_moveRequestCount; ++i)
	{
		MoveRequest* req = &m_moveRequests[i];
		dtCrowdAgent* ag = &m_agents[req->idx];
		
		if (req->state == MR_TARGET_WAITING_FOR_PATH)
		{
			// Poll path queue.
			dtStatus status = m_pathq.getRequestStatus(req->pathqRef);
			if (dtStatusFailed(status))
			{
				req->pathqRef = DT_PATHQ_INVALID;
				req->state = MR_TARGET_FAILED;
			}
			else if (dtStatusSucceed(status))
			{
				const dtPolyRef* path = ag->corridor.getPath();
				const int npath = ag->corridor.getPathCount();
				dtAssert(npath);
				
				// Apply results.
				float targetPos[3];
				dtVcopy(targetPos, req->pos);
				
				dtPolyRef* res = m_pathResult;
				bool valid = true;
				int nres = 0;
				dtStatus status = m_pathq.getPathResult(req->pathqRef, res, &nres, m_maxPathResult);
				if (dtStatusFailed(status) || !nres)
					valid = false;
				
				// Merge with any target adjustment that happened during the search.
				if (req->ntemp > 1)
				{
					nres = dtMergeCorridorEndMoved(res, nres, m_maxPathResult, req->temp, req->ntemp);
				}
				
				// Merge result and existing path.
				// The agent might have moved whilst the request is
				// being processed, so the path may have changed.
				// We assume that the end of the path is at the same location
				// where the request was issued.
				
				// The last ref in the old path should be the same as
				// the location where the request was issued..
				if (valid && path[npath-1] != res[0])
					valid = false;
				
				if (valid)
				{
					// Put the old path infront of the old path.
					if (npath > 1)
					{
						// Make space for the old path.
						if ((npath-1)+nres > m_maxPathResult)
							nres = m_maxPathResult - (npath-1);
						memmove(res+npath-1, res, sizeof(dtPolyRef)*nres);
						// Copy old path in the beginning.
						memcpy(res, path, sizeof(dtPolyRef)*(npath-1));
						nres += npath-1;
					}
					
					// Check for partial path.
					if (res[nres-1] != req->ref)
					{
						// Partial path, constrain target position inside the last polygon.
						float nearest[3];
						if (m_navquery->closestPointOnPoly(res[nres-1], targetPos, nearest) == DT_SUCCESS)
							dtVcopy(targetPos, nearest);
						else
							valid = false;
					}
				}
				
				if (valid)
				{
					ag->corridor.setCorridor(targetPos, res, nres);
					req->state = MR_TARGET_VALID;
				}
				else
				{
					// Something went wrong.
					req->state = MR_TARGET_FAILED;
				}
			}
		}
		
		// Remove request when done with it.
		if (req->state == MR_TARGET_VALID || req->state == MR_TARGET_FAILED)
		{
			m_moveRequestCount--;
			if (i != m_moveRequestCount)
				memcpy(&m_moveRequests[i], &m_moveRequests[m_moveRequestCount], sizeof(MoveRequest));
			--i;
		}
	}
	
}



static int addToOptQueue(dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents)
{
	// Insert neighbour based on greatest time.
	int slot = 0;
	if (!nagents)
	{
		slot = nagents;
	}
	else if (newag->topologyOptTime <= agents[nagents-1]->topologyOptTime)
	{
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
			if (newag->topologyOptTime >= agents[i]->topologyOptTime)
				break;
		
		const int tgt = i+1;
		const int n = dtMin(nagents-i, maxAgents-tgt);
		
		dtAssert(tgt+n <= maxAgents);
		
		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*)*n);
		slot = i;
	}

	agents[slot] = newag;
	
	return dtMin(nagents+1, maxAgents);
}

void dtCrowd::updateTopologyOptimization(dtCrowdAgent** agents, const int nagents, const float dt)
{
	if (!nagents)
		return;
	
	const float OPT_TIME_THR = 0.5f; // seconds
	const int OPT_MAX_AGENTS = 1;
	dtCrowdAgent* queue[OPT_MAX_AGENTS];
	int nqueue = 0;
	
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		ag->topologyOptTime += dt;
		if (ag->topologyOptTime >= OPT_TIME_THR)
			nqueue = addToOptQueue(ag, queue, nqueue, OPT_MAX_AGENTS);
	}

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];
		ag->corridor.optimizePathTopology(m_navquery, &m_filter);
		ag->topologyOptTime = 0;
	}

}

void dtCrowd::update(const float dt, unsigned int flags, dtCrowdAgentDebugInfo* debug)
{
	m_velocitySampleCount = 0;
	
	const int debugIdx = debug ? debug->idx : -1;
	
	dtCrowdAgent** agents = m_activeAgents;
	int nagents = getActiveAgents(agents, m_maxAgents);
	
	// Update async move request and path finder.
	updateMoveRequest(dt);

	// Optimize path topology.
	if (flags & DT_CROWD_OPTIMIZE_TOPO)
		updateTopologyOptimization(agents, nagents, dt);
	
	// Register agents to proximity grid.
	m_grid->clear();
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		const float* p = ag->npos;
		const float r = ag->radius;
		m_grid->addItem((unsigned short)i, p[0]-r, p[2]-r, p[0]+r, p[2]+r);
	}
	
	// Get nearby navmesh segments and agents to collide with.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		// Only update the collision boundary after certain distance has been passed.
		if (dtVdist2DSqr(ag->npos, ag->boundary.getCenter()) > dtSqr(ag->collisionQueryRange*0.25f))
			ag->boundary.update(ag->corridor.getFirstPoly(), ag->npos, ag->collisionQueryRange, m_navquery, &m_filter);
		// Query neighbour agents
		ag->nneis = getNeighbours(ag->npos, ag->height, ag->collisionQueryRange, ag, ag->neis, DT_CROWDAGENT_MAX_NEIGHBOURS);
	}
	
	// Find next corner to steer to.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		
		// Find corners for steering
		ag->ncorners = ag->corridor.findCorners(ag->cornerVerts, ag->cornerFlags, ag->cornerPolys,
												DT_CROWDAGENT_MAX_CORNERS, m_navquery, &m_filter);
		
		// Check to see if the corner after the next corner is directly visible,
		// and short cut to there.
		if ((flags & DT_CROWD_OPTIMIZE_VIS) && ag->ncorners > 0)
		{
			const float* target = &ag->cornerVerts[dtMin(1,ag->ncorners-1)*3];
			ag->corridor.optimizePathVisibility(target, ag->pathOptimizationRange, m_navquery, &m_filter);
			
			// Copy data for debug purposes.
			if (debugIdx == i)
			{
				dtVcopy(debug->optStart, ag->corridor.getPos());
				dtVcopy(debug->optEnd, target);
			}
			
		}
		else
		{
			// Copy data for debug purposes.
			if (debugIdx == i)
			{
				dtVset(debug->optStart, 0,0,0);
				dtVset(debug->optEnd, 0,0,0);
			}
		}
	}
	
	// Calculate steering.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		
		float dvel[3] = {0,0,0};
		
		// Calculate steering direction.
		if (flags & DT_CROWD_ANTICIPATE_TURNS)
			calcSmoothSteerDirection(ag, dvel);
		else
			calcStraightSteerDirection(ag, dvel);
		
		// Calculate speed scale, which tells the agent to slowdown at the end of the path.
		const float slowDownRadius = ag->radius*2;	// TODO: make less hacky.
		const float speedScale = getDistanceToGoal(ag, slowDownRadius) / slowDownRadius;
		
		// Apply style.
		// TODO: find way to express custom movement styles.
/*		if (flags & DT_CROWD_DRUNK)
		{
			// Drunken steering
			
			// Pulsating speed.
			ag->t += dt * (1.0f - ag->var*0.25f);
			ag->desiredSpeed = ag->maxSpeed * (1 + dtSqr(cosf(ag->t*2.0f))*0.3f);
			
			dtVscale(dvel, dvel, ag->desiredSpeed * speedScale);
			
			// Slightly wandering steering.
			const float amp = cosf(ag->var*13.69f+ag->t*3.123f) * 0.2f;
			const float nx = -dvel[2];
			const float nz = dvel[0];
			dvel[0] += nx*amp;
			dvel[2] += nz*amp;
		}
		else*/
		{
			// Normal steering.
			ag->desiredSpeed = ag->maxSpeed;
			dtVscale(dvel, dvel, ag->desiredSpeed * speedScale);
		}
		
		// Set the desired velocity.
		dtVcopy(ag->dvel, dvel);
	}
	
	// Velocity planning.	
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		
		if (flags & DT_CROWD_USE_VO)
		{
			m_obstacleQuery->reset();
			
			// Add neighbours as obstacles.
			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				m_obstacleQuery->addCircle(nei->npos, nei->radius, nei->vel, nei->dvel);
			}

			// Append neighbour segments as obstacles.
			for (int j = 0; j < ag->boundary.getSegmentCount(); ++j)
			{
				const float* s = ag->boundary.getSegment(j);
				if (dtTriArea2D(ag->npos, s, s+3) < 0.0f)
					continue;
				m_obstacleQuery->addSegment(s, s+3);
			}

			dtObstacleAvoidanceDebugData* vod = 0;
			if (debugIdx == i) 
				vod = debug->vod;
			
			// Sample new safe velocity.
			bool adaptive = true;
			int ns = 0;

			if (adaptive)
			{
				ns = m_obstacleQuery->sampleVelocityAdaptive(ag->npos, ag->radius, ag->desiredSpeed,
															 ag->vel, ag->dvel, ag->nvel,
															 VO_ADAPTIVE_DIVS, VO_ADAPTIVE_RINGS, VO_ADAPTIVE_DEPTH,
															 vod);
			}
			else
			{
				ns = m_obstacleQuery->sampleVelocityGrid(ag->npos, ag->radius, ag->desiredSpeed,
														 ag->vel, ag->dvel, ag->nvel,
														 VO_GRID_SIZE, vod);
			}
			m_velocitySampleCount += ns;
		}
		else
		{
			// If not using velocity planning, new velocity is directly the desired velocity.
			dtVcopy(ag->nvel, ag->dvel);
		}
	}
	
	// Integrate.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		integrate(ag, dt);
	}
	
	// Handle collisions.
	static const float COLLISION_RESOLVE_FACTOR = 0.7f;
	
	for (int iter = 0; iter < 4; ++iter)
	{
		for (int i = 0; i < nagents; ++i)
		{
			dtCrowdAgent* ag = agents[i];
			
			dtVset(ag->disp, 0,0,0);
			
			float w = 0;

			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				
				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				
				if (fabsf(diff[1]) >= (ag->height+  nei->height)/2.0f)
					continue;
				
				diff[1] = 0;
				
				float dist = dtVlenSqr(diff);
				if (dist > dtSqr(ag->radius + nei->radius))
					continue;
				dist = sqrtf(dist);
				float pen = (ag->radius + nei->radius) - dist;
				if (dist > 0.0001f)
					pen = (1.0f/dist) * (pen*0.5f) * COLLISION_RESOLVE_FACTOR;
				
				dtVmad(ag->disp, ag->disp, diff, pen);			
				
				w += 1.0f;
			}
			
			if (w > 0.0001f)
			{
				const float iw = 1.0f / w;
				dtVscale(ag->disp, ag->disp, iw);
			}
		}
		
		for (int i = 0; i < nagents; ++i)
		{
			dtCrowdAgent* ag = agents[i];
			dtVadd(ag->npos, ag->npos, ag->disp);
		}
	}
	
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		// Move along navmesh.
		ag->corridor.movePosition(ag->npos, m_navquery, &m_filter);
		// Get valid constrained position back.
		dtVcopy(ag->npos, ag->corridor.getPos());
	}
	
}


