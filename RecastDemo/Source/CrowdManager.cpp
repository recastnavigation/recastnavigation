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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "CrowdManager.h"
#include "SampleInterfaces.h" // For timer


static const int VO_ADAPTIVE_GRID_SIZE = 4;
static const int VO_ADAPTIVE_GRID_DEPTH = 5;
static const int VO_GRID_SIZE = 33;


static int fixupCorridor(dtPolyRef* path, const int npath, const int maxPath,
						 const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;
	
	// Find furthest common polygon.
	for (int i = npath-1; i >= 0; --i)
	{
		bool found = false;
		for (int j = nvisited-1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
			break;
	}
	
	// If no intersection found just return current path. 
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;
	
	// Concatenate paths.	
	
	// Adjust beginning of the buffer to include the visited.
	const int req = nvisited - furthestVisited;
	const int orig = dtMin(furthestPath+1, npath);
	int size = dtMax(0, npath-orig);
	if (req+size > maxPath)
		size = maxPath-req;
	if (size)
		memmove(path+req, path+orig, size*sizeof(dtPolyRef));
	
	// Store visited
	for (int i = 0; i < req; ++i)
		path[i] = visited[(nvisited-1)-i];				
	
	return req+size;
}

static int mergeCorridor(dtPolyRef* path, const int npath, const int maxPath,
						 const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;
	
	// Find furthest common polygon.
	for (int i = npath-1; i >= 0; --i)
	{
		bool found = false;
		for (int j = nvisited-1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
			break;
	}
	
	// If no intersection found just return current path. 
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;
	
	// Concatenate paths.	
	
	// Adjust beginning of the buffer to include the visited.
	const int req = furthestVisited;
	if (req <= 0)
		return npath;
	
	const int orig = furthestPath;
	int size = dtMax(0, npath-orig);
	if (req+size > maxPath)
		size = maxPath-req;
	if (size)
		memmove(path+req, path+orig, size*sizeof(dtPolyRef));
	
	// Store visited
	for (int i = 0; i < req; ++i)
		path[i] = visited[i];
	
	return req+size;
}

// Finds straight path towards the goal and prunes it to contain only relevant vertices.
static int findCorners(const float* pos, const float* target,
					   const dtPolyRef* path, const int npath,
					   float* cornerVerts, unsigned char* cornerFlags,
					   dtPolyRef* cornerPolys, const int maxCorners,
					   const dtNavMeshQuery* navquery)
{
	static const float MIN_TARGET_DIST = 0.01f;
	
	int ncorners = navquery->findStraightPath(pos, target, path, npath,
											  cornerVerts, cornerFlags, cornerPolys,
											  maxCorners);
	
	// Prune points in the beginning of the path which are too close.
	while (ncorners)
	{
		if ((cornerFlags[0] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			dtVdist2DSqr(&cornerVerts[0], pos) > dtSqr(MIN_TARGET_DIST))
			break;
		ncorners--;
		if (ncorners)
		{
			memmove(cornerFlags, cornerFlags+1, sizeof(unsigned char)*ncorners);
			memmove(cornerPolys, cornerPolys+1, sizeof(dtPolyRef)*ncorners);
			memmove(cornerVerts, cornerVerts+3, sizeof(float)*3*ncorners);
		}
	}
	
	// Prune points after an off-mesh connection.
	for (int i = 0; i < ncorners; ++i)
	{
		if (cornerFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
		{
			ncorners = i+1;
			break;
		}
	}
	
	return ncorners;
}

static int optimizePath(const float* pos, const float* next, const float maxLookAhead,
						dtPolyRef* path, const int npath,
						const dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	// Clamp the ray to max distance.
	float goal[3];
	dtVcopy(goal, next);
	const float distSqr = dtVdist2DSqr(pos, goal);
	
	// If too close to the goal, do not try to optimize.
	if (distSqr < dtSqr(0.01f))
		return npath;
	
	// If too far truncate ray length.
	if (distSqr > dtSqr(maxLookAhead))
	{
		float delta[3];
		dtVsub(delta, goal, pos);
		dtVmad(goal, pos, delta, dtSqr(maxLookAhead)/distSqr);
	}
	
	static const int MAX_RES = 32;
	dtPolyRef res[MAX_RES];
	float t, norm[3];
	const int nres = navquery->raycast(path[0], pos, goal, filter, t, norm, res, MAX_RES);
	if (nres > 1 && t > 0.99f)
	{
		return mergeCorridor(path, npath, AGENT_MAX_PATH, res, nres);
	}
	
	return npath;
}

static void updateLocalNeighbourhood(Agent* ag, dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	if (!ag->npath)
		return;
	
	// Only update the neigbourhood after certain distance has been passed.
	if (dtVdist2DSqr(ag->pos, ag->colcenter) < dtSqr(ag->colradius*0.25f))
		return;
	
	dtVcopy(ag->colcenter, ag->pos);
	static const int MAX_LOCALS = 32;
	dtPolyRef locals[MAX_LOCALS];
	
	const int nlocals = navquery->findLocalNeighbourhood(ag->path[0], ag->pos, ag->colradius, filter, locals, 0, MAX_LOCALS);
	
	ag->ncolsegs = 0;
	for (int j = 0; j < nlocals; ++j)
	{
		float segs[DT_VERTS_PER_POLYGON*3*2];
		const int nsegs = navquery->getPolyWallSegments(locals[j], filter, segs);
		for (int k = 0; k < nsegs; ++k)
		{
			const float* s = &segs[k*6];
			// Skip too distant segments.
			float tseg;
			const float distSqr = dtDistancePtSegSqr2D(ag->pos, s, s+3, tseg);
			if (distSqr > dtSqr(ag->colradius))
				continue;
			if (ag->ncolsegs < AGENT_MAX_COLSEGS)
			{
				memcpy(&ag->colsegs[ag->ncolsegs*6], s, sizeof(float)*6);
				ag->ncolsegs++;
			}
		}
	}
}

static void collectObstacles(Agent* ag, Agent** agents, const int nagents,
							 dtObstacleAvoidanceQuery* obstacleQuery)
{
	obstacleQuery->reset();
	
	// Add dynamic obstacles.
	for (int j = 0; j < nagents; ++j)
	{
		const Agent* nei = agents[j];
		if (nei == ag) continue;
		
		float diff[3];
		dtVsub(diff, ag->npos, nei->npos);
		if (fabsf(diff[1]) >= (ag->height+nei->height)/2.0f)
			continue;
		diff[1] = 0;
		
		const float distSqr = dtVlenSqr(diff);
		if (distSqr > dtSqr(ag->colradius))
			continue;
		
		obstacleQuery->addCircle(nei->pos, nei->radius, nei->vel, nei->dvel, distSqr);
	}
	
	// Add static segment obstacles.
	for (int j = 0; j < ag->ncolsegs; ++j)
	{
		const float* s = &ag->colsegs[j*6];
		if (dtTriArea2D(ag->pos, s, s+3) < 0.0f)
			continue;
		
		float tseg;
		const float distSqr = dtDistancePtSegSqr2D(ag->pos, s, s+3, tseg);
		
		obstacleQuery->addSegment(s, s+3, distSqr);
	}
}

static void calcSmoothSteerDirection(const float* pos, const float* corners, const int ncorners, float* dvel)
{
	const int ip0 = 0;
	const int ip1 = dtMin(1, ncorners-1);
	const float* p0 = &corners[ip0*3];
	const float* p1 = &corners[ip1*3];
	
	float dir0[3], dir1[3];
	dtVsub(dir0, p0, pos);
	dtVsub(dir1, p1, pos);
	dir0[1] = 0;
	dir1[1] = 0;
	
	float len0 = dtVlen(dir0);
	float len1 = dtVlen(dir1);
	if (len1 > 0.001f)
		dtVscale(dir1,dir1,1.0f/len1);
	
	const float strength = 0.5f;
	
	dvel[0] = dir0[0] - dir1[0]*len0*strength;
	dvel[1] = 0;
	dvel[2] = dir0[2] - dir1[2]*len0*strength;
	
	dtVnormalize(dvel);
}

static void calcStraightSteerDirection(const float* pos, const float* corners, const int ncorners, float* dvel)
{
	dtVsub(dvel, &corners[0], pos);
	dvel[1] = 0;
	
	dtVnormalize(dvel);
}

CrowdManager::CrowdManager() :
	m_obstacleQuery(0),
	m_totalTime(0),
	m_rvoTime(0),
	m_sampleCount(0)
{
	
	m_obstacleQuery = dtAllocObstacleAvoidanceQuery();
	m_obstacleQuery->init(6, 10);
	
	m_obstacleQuery->setDesiredVelocityWeight(2.0f);
	m_obstacleQuery->setCurrentVelocityWeight(0.75f);
	m_obstacleQuery->setPreferredSideWeight(0.75f);
	m_obstacleQuery->setCollisionTimeWeight(2.5f);
	m_obstacleQuery->setTimeHorizon(2.5f);
	m_obstacleQuery->setVelocitySelectionBias(0.4f);
	
	memset(m_vodebug, 0, sizeof(m_vodebug));
	const int sampleCount = dtMax(VO_GRID_SIZE*VO_GRID_SIZE, (VO_ADAPTIVE_GRID_SIZE*VO_ADAPTIVE_GRID_SIZE)*VO_ADAPTIVE_GRID_DEPTH);
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		m_vodebug[i] = dtAllocObstacleAvoidanceDebugData();
		m_vodebug[i]->init(sampleCount);
	}
	
	reset();
}

CrowdManager::~CrowdManager()
{
	for (int i = 0; i < MAX_AGENTS; ++i)
		dtFreeObstacleAvoidanceDebugData(m_vodebug[i]);
	dtFreeObstacleAvoidanceQuery(m_obstacleQuery);
}

void CrowdManager::reset()
{
	for (int i = 0; i < MAX_AGENTS; ++i)
		memset(&m_agents[i], 0, sizeof(Agent));
}

const int CrowdManager::getAgentCount() const
{
	return MAX_AGENTS;
}

const Agent* CrowdManager::getAgent(const int idx)
{
	return &m_agents[idx];
}

int CrowdManager::addAgent(const float* pos, const float radius, const float height)
{
	// Find empty slot.
	int idx = -1;
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		if (!m_agents[i].active)
		{
			idx = i;
			break;
		}
	}
	if (idx == -1)
		return -1;
	
	Agent* ag = &m_agents[idx];
	memset(ag, 0, sizeof(Agent));
	dtVcopy(ag->pos, pos);
	ag->radius = radius;
	ag->colradius = radius * 7.5f;
	ag->height = height;
	ag->active = 1;
	ag->var = (rand() % 10) / 9.0f;
	
	// Init trail
	for (int i = 0; i < AGENT_MAX_TRAIL; ++i)
		dtVcopy(&ag->trail[i*3], ag->pos);
	ag->htrail = 0;
	
	return idx;
}

void CrowdManager::removeAgent(const int idx)
{
	if (idx >= 0 && idx < MAX_AGENTS)
		memset(&m_agents[idx], 0, sizeof(Agent));
}

void CrowdManager::setMoveTarget(const int idx, const float* pos)
{
	Agent* ag = &m_agents[idx];
	dtVcopy(ag->target, pos);
	ag->targetState = AGENT_TARGET_SET;
}

int CrowdManager::getActiveAgents(Agent** agents, const int maxAgents)
{
	int n = 0;
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		if (!m_agents[i].active) continue;
		if (n < maxAgents)
			agents[n++] = &m_agents[i];
	}
	return n;
}

void CrowdManager::update(const float dt, unsigned int flags, dtNavMeshQuery* navquery)
{
	m_sampleCount = 0;
	m_totalTime = 0;
	m_rvoTime = 0;
	
	if (!navquery)
		return;
	
	TimeVal startTime = getPerfTime();
	
	const float ext[3] = {2,4,2};
	dtQueryFilter filter;
	
	Agent* agents[MAX_AGENTS];
	int nagents = getActiveAgents(agents, MAX_AGENTS);
	
	
	// Update target and agent navigation state.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		// Make sure that the first path polygon corresponds to the current agent location.
		if (!ag->npath)
		{
			float nearest[3];
			ag->path[0] = navquery->findNearestPoly(ag->pos, ext, &filter, nearest);
			if (ag->path[0])
			{
				ag->npath = 1;
				dtVcopy(ag->pos, nearest);
			}
		}
		
		if (ag->targetState == AGENT_TARGET_SET)
		{
			float nearest[3];
			ag->targetRef = navquery->findNearestPoly(ag->target, ext, &filter, nearest);
			if (ag->targetRef)
				dtVcopy(ag->target, nearest);
			ag->targetState = AGENT_TARGET_ACQUIRED;
		}
		
		if (ag->targetState == AGENT_TARGET_ACQUIRED)
		{
			ag->npath = navquery->findPath(ag->path[0], ag->targetRef, ag->pos, ag->target,
										   &filter, ag->path, AGENT_MAX_PATH);
			if (ag->npath)
			{
				ag->targetState = AGENT_TARGET_PATH;
				// Check for partial path.
				if (ag->path[ag->npath-1] != ag->targetRef)
				{
					// Partial path, constrain target position inside the last polygon.
					ag->targetRef = ag->path[ag->npath-1];
					float nearest[3];
					if (navquery->closestPointOnPoly(ag->targetRef, ag->target, nearest))
						dtVcopy(ag->target, nearest);
					else
						ag->targetState = AGENT_TARGET_FAILED;
				}
			}
			else
				ag->targetState = AGENT_TARGET_FAILED;
		}
	}
	
	// Get nearby navmesh segments to collide with.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		updateLocalNeighbourhood(ag, navquery, &filter);
	}
	
	static const float MAX_ACC = 8.0f;
	static const float MAX_SPEED = 3.5f;
	
	// Find next corner to steer to.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		ag->ncorners = 0;
		dtVset(ag->opts, 0,0,0);
		dtVset(ag->opte, 0,0,0);
		
		if (ag->targetState == AGENT_TARGET_PATH)
		{
			// Find nest couple of corners for steering.
			ag->ncorners = findCorners(ag->pos, ag->target, ag->path, ag->npath,
									   ag->cornerVerts, ag->cornerFlags, ag->cornerPolys,
									   AGENT_MAX_CORNERS, navquery);
			
			// Check to see if the corner after the next corner is directly visible,
			// and short cut to there.
			if (ag->ncorners > 1)
			{
				const float maxLookAhead = ag->colradius*4;
				
				dtVcopy(ag->opts, ag->pos);
				dtVcopy(ag->opte, ag->cornerVerts+3);
				
				ag->npath = optimizePath(ag->pos, ag->cornerVerts+3, maxLookAhead,
										 ag->path, ag->npath, navquery, &filter);
			}
		}
	}
	
	// Calculate steering.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		if (!ag->ncorners)
		{
			// No corner to steer to, stop.
			dtVset(ag->dvel, 0,0,0);
		}
		else
		{
			// Calculate steering direction.
			if (flags & CROWDMAN_ANTICIPATE_TURNS)
			{
				calcSmoothSteerDirection(ag->pos, ag->cornerVerts, ag->ncorners, ag->dvel);
			}
			else
			{
				calcStraightSteerDirection(ag->pos, ag->cornerVerts, ag->ncorners, ag->dvel);
			}
			
			// Calculate steering speed.
			
			// Calculate speed scale, which tells the agent to slowdown at the end of the path.
			float speedScale = 1.0f;
			const bool endOfPath = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
			const bool offMeshConnection = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
			const float slowDownRadius = ag->radius*2;
			if (endOfPath || offMeshConnection)
			{
				const float distToGoal = dtVdist2D(ag->pos, &ag->cornerVerts[(ag->ncorners-1)*3]);
				speedScale = dtMin(1.0f, distToGoal / slowDownRadius);
			}
			
			// Apply style.
			if (flags & CROWDMAN_DRUNK)
			{
				// Drunken steering
				
				// Pulsating speed.
				ag->t += dt * (1.0f - ag->var*0.25f);
				ag->maxspeed = MAX_SPEED*(1 + dtSqr(cosf(ag->t*2.0f))*0.3f);
				
				dtVscale(ag->dvel, ag->dvel, ag->maxspeed * speedScale);
				
				// Slightly wandering steering.
				const float amp = cosf(ag->var*13.69f+ag->t*3.123f) * 0.2f;
				const float nx = -ag->dvel[2];
				const float nz = ag->dvel[0];
				ag->dvel[0] += nx*amp;
				ag->dvel[2] += nz*amp;
			}
			else
			{
				// Normal steering.
				ag->maxspeed = MAX_SPEED;
				dtVscale(ag->dvel, ag->dvel, ag->maxspeed * speedScale);
			}
		}
	}
	
	// Velocity planning.
	TimeVal rvoStartTime = getPerfTime();
	
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		if (flags & CROWDMAN_USE_VO)
		{
			collectObstacles(ag, agents, nagents, m_obstacleQuery);
			
			bool adaptive = true;
			
			if (adaptive)
			{
				m_obstacleQuery->setSamplingGridSize(VO_ADAPTIVE_GRID_SIZE);
				m_obstacleQuery->setSamplingGridDepth(VO_ADAPTIVE_GRID_DEPTH);
				m_obstacleQuery->sampleVelocityAdaptive(ag->pos, ag->radius, ag->maxspeed,
														ag->vel, ag->dvel, ag->nvel, m_vodebug[i]);
			}
			else
			{
				m_obstacleQuery->setSamplingGridSize(VO_GRID_SIZE);
				m_obstacleQuery->sampleVelocity(ag->pos, ag->radius, ag->maxspeed, ag->vel, ag->dvel,
												ag->nvel, m_vodebug[i]);
			}
		}
		else
		{
			dtVcopy(ag->nvel, ag->dvel);
		}
	}
	TimeVal rvoEndTime = getPerfTime();
	
	// Integrate.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		// Fake dynamic constraint.
		const float maxDelta = MAX_ACC * dt;
		float dv[3];
		dtVsub(dv, ag->nvel, ag->vel);
		float ds = dtVlen(dv);
		if (ds > maxDelta)
			dtVscale(dv, dv, maxDelta/ds);
		dtVadd(ag->vel, ag->vel, dv);
		
		// Integrate
		if (dtVlen(ag->vel) > 0.0001f)
			dtVmad(ag->npos, ag->pos, ag->vel, dt);
		else
			dtVcopy(ag->npos, ag->pos);
	}
	
	// Handle collisions.
	for (int iter = 0; iter < 4; ++iter)
	{
		for (int i = 0; i < nagents; ++i)
		{
			Agent* ag = agents[i];
			
			dtVset(ag->disp, 0,0,0);
			
			float w = 0;
			
			for (int j = 0; j < nagents; ++j)
			{
				if (i == j) continue;
				Agent* nei = agents[j];
				
				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				
				if (fabsf(diff[1]) >= (ag->height+nei->height)/2.0f)
					continue;
				
				diff[1] = 0;
				
				float dist = dtVlenSqr(diff);
				if (dist > dtSqr(ag->radius+nei->radius))
					continue;
				dist = sqrtf(dist);
				float pen = (ag->radius+nei->radius) - dist;
				if (dist > 0.0001f)
					pen = (1.0f/dist) * (pen*0.5f) * 0.7f;
				
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
			Agent* ag = agents[i];
			dtVadd(ag->npos, ag->npos, ag->disp);
		}
	}
	
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		// Move along navmesh and update new position.
		float result[3];
		dtPolyRef visited[16];
		int nvisited = navquery->moveAlongSurface(ag->path[0], ag->pos, ag->npos, &filter,
												  result, visited, 16);
		ag->npath = fixupCorridor(ag->path, ag->npath, AGENT_MAX_PATH, visited, nvisited);
		
		// Adjust agent height to stay on top of the navmesh.
		float h = 0;
		navquery->getPolyHeight(ag->path[0], result, &h);
		result[1] = h;
		dtVcopy(ag->pos, result);
	}
	
	
	TimeVal endTime = getPerfTime();
	
	int ns = 0;
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		if (flags & CROWDMAN_USE_VO)
		{
			// Normalize samples for debug draw
			m_vodebug[i]->normalizeSamples();
			ns += m_vodebug[i]->getSampleCount();
		}
		
		// Update agent movement trail.
		ag->htrail = (ag->htrail + 1) % AGENT_MAX_TRAIL;
		dtVcopy(&ag->trail[ag->htrail*3], ag->pos);
	}

	m_sampleCount = ns;
	m_rvoTime = getPerfDeltaTimeUsec(rvoStartTime, rvoEndTime);
	m_totalTime = getPerfDeltaTimeUsec(startTime, endTime);
}


