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
#include <stdlib.h>
#include <new>
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "DetourAssert.h"
#include "DetourAlloc.h"


dtCrowd* dtAllocCrowd()
{
	void* mem = dtAlloc(sizeof(dtCrowd), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtCrowd;
}

void dtFreeCrowd(dtCrowd* ptr)
{
	if (!ptr) return;
	ptr->~dtCrowd();
	dtFree(ptr);
}


static const int MAX_ITERS_PER_UPDATE = 10;

static const int MAX_PATHQUEUE_NODES = 4096;
static const int MAX_COMMON_NODES = 512;

inline float tween(const float t, const float t0, const float t1)
{
	return dtClamp((t-t0) / (t1-t0), 0.0f, 1.0f);
}

static void integrate(dtCrowdAgent* ag, const float dt)
{
	// Fake dynamic constraint.
	const float maxDelta = ag->params.maxAcceleration * dt;
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

static bool overOffmeshConnection(const dtCrowdAgent* ag, const float radius)
{
	if (!ag->ncorners)
		return false;
	
	const bool offMeshConnection = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
	if (offMeshConnection)
	{
		const float distSq = dtVdist2DSqr(ag->npos, &ag->cornerVerts[(ag->ncorners-1)*3]);
		if (distSq < radius*radius)
			return true;
	}
	
	return false;
}

static float getDistanceToGoal(const dtCrowdAgent* ag, const float range)
{
	if (!ag->ncorners)
		return range;
	
	const bool endOfPath = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
	if (endOfPath)
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

static int getNeighbours(const float* pos, const float height, const float range,
						 const dtCrowdAgent* skip, dtCrowdNeighbour* result, const int maxResult,
						 dtCrowdAgent** agents, const int /*nagents*/, dtProximityGrid* grid)
{
	int n = 0;
	
	static const int MAX_NEIS = 32;
	unsigned short ids[MAX_NEIS];
	int nids = grid->queryItems(pos[0]-range, pos[2]-range,
								pos[0]+range, pos[2]+range,
								ids, MAX_NEIS);
	
	for (int i = 0; i < nids; ++i)
	{
		const dtCrowdAgent* ag = agents[ids[i]];
		
		if (ag == skip) continue;
		
		// Check for overlap.
		float diff[3];
		dtVsub(diff, pos, ag->npos);
		if (fabsf(diff[1]) >= (height+ag->params.height)/2.0f)
			continue;
		diff[1] = 0;
		const float distSqr = dtVlenSqr(diff);
		if (distSqr > dtSqr(range))
			continue;
		
		n = addNeighbour(ids[i], distSqr, result, n, maxResult);
	}
	return n;
}



dtCrowd::dtCrowd() :
	m_maxAgents(0),
	m_agents(0),
	m_activeAgents(0),
	m_agentAnims(0),
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

	dtFree(m_agentAnims);
	m_agentAnims = 0;
	
	dtFree(m_pathResult);
	m_pathResult = 0;
	
	dtFree(m_moveRequests);
	m_moveRequests = 0;
	m_moveRequestCount = 0;
	
	dtFreeProximityGrid(m_grid);
	m_grid = 0;

	dtFreeObstacleAvoidanceQuery(m_obstacleQuery);
	m_obstacleQuery = 0;
	
	dtFreeNavMeshQuery(m_navquery);
	m_navquery = 0;
}

bool dtCrowd::init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav)
{
	purge();
	
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
	if (!m_obstacleQuery->init(6, 8))
		return false;

	// Init obstacle query params.
	memset(m_obstacleQueryParams, 0, sizeof(m_obstacleQueryParams));
	for (int i = 0; i < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS; ++i)
	{
		dtObstacleAvoidanceParams* params = &m_obstacleQueryParams[i];
		params->velBias = 0.4f;
		params->weightDesVel = 2.0f;
		params->weightCurVel = 0.75f;
		params->weightSide = 0.75f;
		params->weightToi = 2.5f;
		params->horizTime = 2.5f;
		params->gridSize = 33;
		params->adaptiveDivs = 7;
		params->adaptiveRings = 2;
		params->adaptiveDepth = 5;
	}
	
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

	m_agentAnims = (dtCrowdAgentAnimation*)dtAlloc(sizeof(dtCrowdAgentAnimation)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_agentAnims)
		return false;
	
	for (int i = 0; i < m_maxAgents; ++i)
	{
		new(&m_agents[i]) dtCrowdAgent();
		m_agents[i].active = 0;
		if (!m_agents[i].corridor.init(m_maxPathResult))
			return false;
	}

	for (int i = 0; i < m_maxAgents; ++i)
	{
		m_agentAnims[i].active = 0;
	}

	// The navquery is mostly used for local searches, no need for large node pool.
	m_navquery = dtAllocNavMeshQuery();
	if (!m_navquery)
		return false;
	if (dtStatusFailed(m_navquery->init(nav, MAX_COMMON_NODES)))
		return false;
	
	return true;
}

void dtCrowd::setObstacleAvoidanceParams(const int idx, const dtObstacleAvoidanceParams* params)
{
	if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
		memcpy(&m_obstacleQueryParams[idx], params, sizeof(dtObstacleAvoidanceParams));
}

const dtObstacleAvoidanceParams* dtCrowd::getObstacleAvoidanceParams(const int idx) const
{
	if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
		return &m_obstacleQueryParams[idx];
	return 0;
}

const int dtCrowd::getAgentCount() const
{
	return m_maxAgents;
}

const dtCrowdAgent* dtCrowd::getAgent(const int idx)
{
	return &m_agents[idx];
}

void dtCrowd::updateAgentParameters(const int idx, const dtCrowdAgentParams* params)
{
	if (idx < 0 || idx > m_maxAgents)
		return;
	memcpy(&m_agents[idx].params, params, sizeof(dtCrowdAgentParams));
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
	
	ag->corridor.reset(ref, nearest);
	ag->boundary.reset();

	updateAgentParameters(idx, params);
	
	ag->topologyOptTime = 0;
	ag->nneis = 0;
	
	dtVset(ag->dvel, 0,0,0);
	dtVset(ag->nvel, 0,0,0);
	dtVset(ag->vel, 0,0,0);
	dtVcopy(ag->npos, nearest);
	
	ag->desiredSpeed = 0;
	ag->t = 0;
	ag->var = (rand() % 10) / 9.0f;

	if (ref)
		ag->state = DT_CROWDAGENT_STATE_WALKING;
	else
		ag->state = DT_CROWDAGENT_STATE_INVALID;
	
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

const dtCrowd::MoveRequest* dtCrowd::getActiveMoveTarget(const int idx) const
{
	if (idx < 0 || idx > m_maxAgents)
		return 0;
	
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
	
	return req;
}

bool dtCrowd::requestMoveTargetReplan(const int idx, dtPolyRef ref, const float* pos)
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
	req->replan = true;
	
	req->temp[0] = ref;
	req->ntemp = 1;

	return true;
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
	req->replan = false;
	
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


void dtCrowd::updateMoveRequest(const float /*dt*/)
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
			// If agent location is invalid, try to recover.
			if (ag->state == DT_CROWDAGENT_STATE_INVALID)
			{
				float agentPos[3];
				dtVcopy(agentPos, ag->npos);
				dtPolyRef agentRef = ag->corridor.getFirstPoly();
				if (!m_navquery->isValidPolyRef(agentRef, &m_filter))
				{
					// Current location is not valid, try to reposition.
					// TODO: this can snap agents, how to handle that?
					float nearest[3];
					agentRef = 0;
					m_navquery->findNearestPoly(ag->npos, m_ext, &m_filter, &agentRef, nearest);
					dtVcopy(agentPos, nearest);
				}
				if (!agentRef)
				{
					// Current location still outside navmesh, fail the request.
					req->state = MR_TARGET_FAILED;
					continue;
				}

				// Could relocation agent on navmesh again.
				ag->state = DT_CROWDAGENT_STATE_WALKING;
				ag->corridor.reset(agentRef, agentPos);
			}
			
			// Calculate request position.
			
			if (req->replan)
			{
				// Replan from the end of the current path.
				dtPolyRef reqRef = ag->corridor.getLastPoly();
				float reqPos[3];
				dtVcopy(reqPos, ag->corridor.getTarget());
				req->pathqRef = m_pathq.request(reqRef, req->ref, reqPos, req->pos, &m_filter);
				if (req->pathqRef != DT_PATHQ_INVALID)
				{
					req->state = MR_TARGET_WAITING_FOR_PATH;
				}
			}
			else
			{
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
						
						// Remove trackbacks
						for (int j = 0; j < nres; ++j)
						{
							if (j-1 >= 0 && j+1 < nres)
							{
								if (res[j-1] == res[j+1])
								{
									memmove(res+(j-1), res+(j+1), sizeof(dtPolyRef)*(nres-(j+1)));
									nres -= 2;
									j -= 2;
								}
							}
						}
						
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
					// Set current corridor.
					ag->corridor.setCorridor(targetPos, res, nres);
					// Force to update boundary.
					ag->boundary.reset();
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
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_TOPO) == 0)
			continue;
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

void dtCrowd::checkPathValidty(dtCrowdAgent** agents, const int nagents, const float dt)
{
	static const int CHECK_LOOKAHEAD = 10;
	
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		
		// Skip if the corridor is valid
		if (ag->corridor.isValid(CHECK_LOOKAHEAD, m_navquery, &m_filter))
			continue;

		// The current path is bad, try to recover.

		// Store current state.
		float agentPos[3];
		dtPolyRef agentRef = 0;
		dtVcopy(agentPos, ag->npos);
		agentRef = ag->corridor.getFirstPoly();
		
		float targetPos[3];
		dtPolyRef targetRef = 0;
		dtVcopy(targetPos, ag->corridor.getTarget());
		targetRef = ag->corridor.getLastPoly();
		// If has active move target, get target location from that instead.
		const int idx = getAgentIndex(ag);
		const MoveRequest* req = getActiveMoveTarget(idx);
		if (req)
		{
			if (req->state == MR_TARGET_REQUESTING ||
				req->state == MR_TARGET_WAITING_FOR_PATH ||
				req->state == MR_TARGET_VALID)
			{
				targetRef = req->ref;
				dtVcopy(targetPos, req->pos);
			}
			else if (req->state == MR_TARGET_ADJUST)
			{
				targetRef = req->aref;
				dtVcopy(targetPos, req->apos);
			}
		}
		
		// First check that the current location is valid.
		if (!m_navquery->isValidPolyRef(agentRef, &m_filter))
		{
			// Current location is not valid, try to reposition.
			// TODO: this can snap agents, how to handle that?
			float nearest[3];
			agentRef = 0;
			m_navquery->findNearestPoly(ag->npos, m_ext, &m_filter, &agentRef, nearest);
			dtVcopy(agentPos, nearest);
		}

		if (!agentRef)
		{
			// Count not find location in navmesh, set state to invalid.
			ag->corridor.reset(0, agentPos);
			ag->boundary.reset();
			ag->state = DT_CROWDAGENT_STATE_INVALID;
			continue;
		}
		
		// TODO: make temp path by raycasting towards current velocity.
		
		ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
		ag->boundary.reset();
		dtVcopy(ag->npos, agentPos);
		
		
		// Check that target is still reachable.
		if (!m_navquery->isValidPolyRef(targetRef, &m_filter))
		{
			// Current target is not valid, try to reposition.
			float nearest[3];
			targetRef = 0;
			m_navquery->findNearestPoly(targetPos, m_ext, &m_filter, &targetRef, nearest);
			dtVcopy(targetPos, nearest);
		}
		
		if (!targetRef)
		{
			// Target not reachable anymore, cannot replan.
			// TODO: indicate failure!
			continue;
		}
		
		// Try to replan path to goal.
		requestMoveTargetReplan(idx, targetRef, targetPos);
	}
}
	
void dtCrowd::update(const float dt, dtCrowdAgentDebugInfo* debug)
{
	m_velocitySampleCount = 0;
	
	const int debugIdx = debug ? debug->idx : -1;
	
	dtCrowdAgent** agents = m_activeAgents;
	int nagents = getActiveAgents(agents, m_maxAgents);
	
	// Check that all agents still have valid paths.
	checkPathValidty(agents, nagents, dt);
	
	// Update async move request and path finder.
	updateMoveRequest(dt);

	// Optimize path topology.
	updateTopologyOptimization(agents, nagents, dt);
	
	// Register agents to proximity grid.
	m_grid->clear();
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		const float* p = ag->npos;
		const float r = ag->params.radius;
		m_grid->addItem((unsigned short)i, p[0]-r, p[2]-r, p[0]+r, p[2]+r);
	}
	
	// Get nearby navmesh segments and agents to collide with.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		// Update the collision boundary after certain distance has been passed or
		// if it has become invalid.
		const float updateThr = ag->params.collisionQueryRange*0.25f;
		if (dtVdist2DSqr(ag->npos, ag->boundary.getCenter()) > dtSqr(updateThr) ||
			!ag->boundary.isValid(m_navquery, &m_filter))
		{
			ag->boundary.update(ag->corridor.getFirstPoly(), ag->npos, ag->params.collisionQueryRange,
								m_navquery, &m_filter);
		}
		// Query neighbour agents
		ag->nneis = getNeighbours(ag->npos, ag->params.height, ag->params.collisionQueryRange,
								  ag, ag->neis, DT_CROWDAGENT_MAX_NEIGHBOURS,
								  agents, nagents, m_grid);
	}
	
	// Find next corner to steer to.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		
		// Find corners for steering
		ag->ncorners = ag->corridor.findCorners(ag->cornerVerts, ag->cornerFlags, ag->cornerPolys,
												DT_CROWDAGENT_MAX_CORNERS, m_navquery, &m_filter);
		
		// Check to see if the corner after the next corner is directly visible,
		// and short cut to there.
		if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_VIS) && ag->ncorners > 0)
		{
			const float* target = &ag->cornerVerts[dtMin(1,ag->ncorners-1)*3];
			ag->corridor.optimizePathVisibility(target, ag->params.pathOptimizationRange, m_navquery, &m_filter);
			
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
	
	// Trigger off-mesh connections (depends on corners).
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		
		// Check 
		const float triggerRadius = ag->params.radius*2.25f;
		if (overOffmeshConnection(ag, triggerRadius))
		{
			// Prepare to off-mesh connection.
			const int idx = ag - m_agents;
			dtCrowdAgentAnimation* anim = &m_agentAnims[idx];
			
			// Adjust the path over the off-mesh connection.
			dtPolyRef refs[2];
			if (ag->corridor.moveOverOffmeshConnection(ag->cornerPolys[ag->ncorners-1], refs,
													   anim->startPos, anim->endPos, m_navquery))
			{
				dtVcopy(anim->initPos, ag->npos);
				anim->polyRef = refs[1];
				anim->active = 1;
				anim->t = 0.0f;
				anim->tmax = (dtVdist2D(anim->startPos, anim->endPos) / ag->params.maxSpeed) * 0.5f;
				
				ag->state = DT_CROWDAGENT_STATE_OFFMESH;
				ag->ncorners = 0;
				ag->nneis = 0;
				continue;
			}
			else
			{
				// TODO: Off-mesh connection failed, replan.
			}
			
		}
	}
		
	// Calculate steering.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		
		float dvel[3] = {0,0,0};
		
		// Calculate steering direction.
		if (ag->params.updateFlags & DT_CROWD_ANTICIPATE_TURNS)
			calcSmoothSteerDirection(ag, dvel);
		else
			calcStraightSteerDirection(ag, dvel);
		
		// Calculate speed scale, which tells the agent to slowdown at the end of the path.
		const float slowDownRadius = ag->params.radius*2;	// TODO: make less hacky.
		const float speedScale = getDistanceToGoal(ag, slowDownRadius) / slowDownRadius;
			
		ag->desiredSpeed = ag->params.maxSpeed;
		dtVscale(dvel, dvel, ag->desiredSpeed * speedScale);

		// Separation
		if (ag->params.updateFlags & DT_CROWD_SEPARATION)
		{
			const float separationDist = ag->params.collisionQueryRange; 
			const float invSeparationDist = 1.0f / separationDist; 
			const float separationWeight = ag->params.separationWeight;
			
			float w = 0;
			float disp[3] = {0,0,0};
			
			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				
				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				diff[1] = 0;
				
				const float distSqr = dtVlenSqr(diff);
				if (distSqr < 0.00001f)
					continue;
				if (distSqr > dtSqr(separationDist))
					continue;
				const float dist = sqrtf(distSqr);
				const float weight = separationWeight * (1.0f - dtSqr(dist*invSeparationDist));
				
				dtVmad(disp, disp, diff, weight/dist);
				w += 1.0f;
			}
			
			if (w > 0.0001f)
			{
				// Adjust desired velocity.
				dtVmad(dvel, dvel, disp, 1.0f/w);
				// Clamp desired velocity to desired speed.
				const float speedSqr = dtVlenSqr(dvel);
				const float desiredSqr = dtSqr(ag->desiredSpeed);
				if (speedSqr > desiredSqr)
					dtVscale(dvel, dvel, desiredSqr/speedSqr);
			}
		}
		
		// Set the desired velocity.
		dtVcopy(ag->dvel, dvel);
	}
	
	// Velocity planning.	
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		
		if (ag->params.updateFlags & DT_CROWD_OBSTACLE_AVOIDANCE)
		{
			m_obstacleQuery->reset();
			
			// Add neighbours as obstacles.
			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = agents[ag->neis[j].idx];
				m_obstacleQuery->addCircle(nei->npos, nei->params.radius, nei->vel, nei->dvel);
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

			const dtObstacleAvoidanceParams* params = &m_obstacleQueryParams[ag->params.obstacleAvoidanceType];
				
			if (adaptive)
			{
				ns = m_obstacleQuery->sampleVelocityAdaptive(ag->npos, ag->params.radius, ag->desiredSpeed,
															 ag->vel, ag->dvel, ag->nvel, params, vod);
			}
			else
			{
				ns = m_obstacleQuery->sampleVelocityGrid(ag->npos, ag->params.radius, ag->desiredSpeed,
														 ag->vel, ag->dvel, ag->nvel, params, vod);
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
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		integrate(ag, dt);
	}
	
	// Handle collisions.
	static const float COLLISION_RESOLVE_FACTOR = 0.7f;
	
	for (int iter = 0; iter < 4; ++iter)
	{
		for (int i = 0; i < nagents; ++i)
		{
			dtCrowdAgent* ag = agents[i];
			const int idx0 = getAgentIndex(ag);
			
			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			dtVset(ag->disp, 0,0,0);
			
			float w = 0;

			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = agents[ag->neis[j].idx];
				const int idx1 = getAgentIndex(nei);

				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				diff[1] = 0;
				
				float dist = dtVlenSqr(diff);
				if (dist > dtSqr(ag->params.radius + nei->params.radius))
					continue;
				dist = sqrtf(dist);
				float pen = (ag->params.radius + nei->params.radius) - dist;
				if (dist < 0.0001f)
				{
					// Agents on top of each other, try to choose diverging separation directions.
					if (idx0 > idx1)
						dtVset(diff, -ag->dvel[2],0,ag->dvel[0]);
					else
						dtVset(diff, ag->dvel[2],0,-ag->dvel[0]);
					pen = 0.01f;
				}
				else
				{
					pen = (1.0f/dist) * (pen*0.5f) * COLLISION_RESOLVE_FACTOR;
				}
				
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
			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;
			
			dtVadd(ag->npos, ag->npos, ag->disp);
		}
	}
	
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		
		// Move along navmesh.
		ag->corridor.movePosition(ag->npos, m_navquery, &m_filter);
		// Get valid constrained position back.
		dtVcopy(ag->npos, ag->corridor.getPos());
	}
	
	// Update agents using off-mesh connection.
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgentAnimation* anim = &m_agentAnims[i];
		if (!anim->active)
			continue;
		dtCrowdAgent* ag = agents[i];

		anim->t += dt;
		if (anim->t > anim->tmax)
		{
			// Reset animation
			anim->active = 0;
			// Prepare agent for walking.
			ag->state = DT_CROWDAGENT_STATE_WALKING;
			continue;
		}
		
		// Update position
		const float ta = anim->tmax*0.15f;
		const float tb = anim->tmax;
		if (anim->t < ta)
		{
			const float u = tween(anim->t, 0.0, ta);
			dtVlerp(ag->npos, anim->initPos, anim->startPos, u);
		}
		else
		{
			const float u = tween(anim->t, ta, tb);
			dtVlerp(ag->npos, anim->startPos, anim->endPos, u);
		}
			
		// Update velocity.
		dtVset(ag->vel, 0,0,0);
		dtVset(ag->dvel, 0,0,0);
	}
	
}


