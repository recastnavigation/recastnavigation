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
#include "DetourAssert.h"
#include "DetourAlloc.h"

static const int VO_ADAPTIVE_GRID_SIZE = 4;
static const int VO_ADAPTIVE_GRID_DEPTH = 5;
static const int VO_GRID_SIZE = 33;


inline int hashPos2(int x, int y, int n)
{
	return ((x*73856093) ^ (y*19349663)) & (n-1);
}

ProximityGrid::ProximityGrid() :
	m_maxItems(0),
	m_cellSize(0),
	m_pool(0),
	m_poolHead(0),
	m_poolSize(0),
	m_buckets(0),
	m_bucketsSize(0)
{
}
	
ProximityGrid::~ProximityGrid()
{
	dtFree(m_buckets);
	dtFree(m_pool);
}
	
bool ProximityGrid::init(const int maxItems, const float cellSize)
{
	dtAssert(maxItems > 0);
	dtAssert(cellSize > 0.0f);
	
	m_cellSize = cellSize;
	m_invCellSize = 1.0f / m_cellSize;
	
	// Allocate hashs buckets
	m_bucketsSize = dtNextPow2(maxItems);
	m_buckets = (unsigned short*)dtAlloc(sizeof(unsigned short)*m_bucketsSize, DT_ALLOC_PERM);
	if (!m_buckets)
		return false;

	// Allocate pool of items.
	m_poolSize = maxItems*4;
	m_poolHead = 0;
	m_pool = (Item*)dtAlloc(sizeof(Item)*m_poolSize, DT_ALLOC_PERM);
	if (!m_pool)
		return false;
	
	clear();
	
	return true;
}

void ProximityGrid::clear()
{
	memset(m_buckets, 0xff, sizeof(unsigned short)*m_bucketsSize);
	m_poolHead = 0;
	m_bounds[0] = 0xffff;
	m_bounds[1] = 0xffff;
	m_bounds[2] = -0xffff;
	m_bounds[3] = -0xffff;
}

void ProximityGrid::addItem(const unsigned short id,
							const float minx, const float miny,
							const float maxx, const float maxy)
{
	const int iminx = (int)floorf(minx * m_invCellSize);
	const int iminy = (int)floorf(miny * m_invCellSize);
	const int imaxx = (int)floorf(maxx * m_invCellSize);
	const int imaxy = (int)floorf(maxy * m_invCellSize);
	
	m_bounds[0] = dtMin(m_bounds[0], iminx);
	m_bounds[1] = dtMin(m_bounds[1], iminy);
	m_bounds[2] = dtMax(m_bounds[2], imaxx);
	m_bounds[3] = dtMax(m_bounds[3], imaxy);
	
	for (int y = iminy; y <= imaxy; ++y)
	{
		for (int x = iminx; x <= imaxx; ++x)
		{
			if (m_poolHead < m_poolSize)
			{
				const int h = hashPos2(x, y, m_bucketsSize);
				const unsigned short idx = (unsigned short)m_poolHead;
				m_poolHead++;
				Item& item = m_pool[idx];
				item.x = (short)x;
				item.y = (short)y;
				item.id = id;
				item.next = m_buckets[h];
				m_buckets[h] = idx;
			}
		}
	}
}

int ProximityGrid::queryItems(const float minx, const float miny,
							  const float maxx, const float maxy,
							  unsigned short* ids, const int maxIds) const
{
	const int iminx = (int)floorf(minx * m_invCellSize);
	const int iminy = (int)floorf(miny * m_invCellSize);
	const int imaxx = (int)floorf(maxx * m_invCellSize);
	const int imaxy = (int)floorf(maxy * m_invCellSize);
	
	int n = 0;
	
	for (int y = iminy; y <= imaxy; ++y)
	{
		for (int x = iminx; x <= imaxx; ++x)
		{
			const int h = hashPos2(x, y, m_bucketsSize);
			unsigned short idx = m_buckets[h];
			while (idx != 0xffff)
			{
				Item& item = m_pool[idx];
				if ((int)item.x == x && (int)item.y == y)
				{
					// Check if the id exists already.
					const unsigned short* end = ids + n;
					unsigned short* i = ids;
					while (i != end && *i != item.id)
						++i;
					// Item not found, add it.
					if (i == end)
					{
						if (n >= maxIds)
							return n;
						ids[n++] = item.id;
					}
				}
				idx = item.next;
			}
		}
	}
	
	return n;
}

int ProximityGrid::getItemCountAt(const int x, const int y) const
{
	int n = 0;
	
	const int h = hashPos2(x, y, m_bucketsSize);
	unsigned short idx = m_buckets[h];
	while (idx != 0xffff)
	{
		Item& item = m_pool[idx];
		if ((int)item.x == x && (int)item.y == y)
			n++;
		idx = item.next;
	}

	return n;
}


PathQueue::PathQueue() :
	m_nextHandle(1),
	m_delay(0)
{
	for (int i = 0; i < MAX_QUEUE; ++i)
		m_queue[i].ref = PATHQ_INVALID;
}

PathQueue::~PathQueue()
{
}

void PathQueue::update(dtNavMeshQuery* navquery)
{
	// Artificial delay to test the code better,
	// update only one request too.
	// TODO: Use sliced pathfinder.
	m_delay++;
	if ((m_delay % 4) == 0)
	{
		for (int i = 0; i < MAX_QUEUE; ++i)
		{
			PathQuery& q = m_queue[i];
			if (q.ref == PATHQ_INVALID)
				continue;
			q.npath = navquery->findPath(q.startRef, q.endRef, q.startPos, q.endPos, q.filter, q.path, AGENT_MAX_PATH);
			q.ready = true;
			break;
		}
	}

	// Kill forgotten request.
	for (int i = 0; i < MAX_QUEUE; ++i)
	{
		PathQuery& q = m_queue[i];
		if (q.ref != PATHQ_INVALID && q.ready)
		{
			q.keepalive++;
			if (q.keepalive > 2)
				q.ref = PATHQ_INVALID;
		}
	}
}

PathQueueRef PathQueue::request(dtPolyRef startRef, dtPolyRef endRef,
								const float* startPos, const float* endPos,
								const dtQueryFilter* filter)
{
	// Find empty slot
	int slot = -1;
	for (int i = 0; i < MAX_QUEUE; ++i)
	{
		if (m_queue[i].ref == PATHQ_INVALID)
		{
			slot = i;
			break;
		}
	}
	// Could not find slot.
	if (slot == -1)
		return PATHQ_INVALID;
	
	PathQueueRef ref = m_nextHandle++;
	if (m_nextHandle == PATHQ_INVALID) m_nextHandle++;
	
	PathQuery& q = m_queue[slot];
	q.ref = ref;
	dtVcopy(q.startPos, startPos);
	q.startRef = startRef;
	dtVcopy(q.endPos, endPos);
	q.endRef = endRef;
	q.ready = false;
	q.npath = 0;
	q.filter = filter; // TODO: This is potentially dangerous!
	q.keepalive = 0;
	
	return ref;
}

int PathQueue::getRequestState(PathQueueRef ref)
{
	for (int i = 0; i < MAX_QUEUE; ++i)
	{
		if (m_queue[i].ref == ref)
			return m_queue[i].ready ? PATHQ_STATE_READY : PATHQ_STATE_WORKING;
	}
	
	return PATHQ_STATE_INVALID;
}

int PathQueue::getPathResult(PathQueueRef ref, dtPolyRef* path, const int maxPath)
{
	for (int i = 0; i < MAX_QUEUE; ++i)
	{
		if (m_queue[i].ref == ref)
		{
			PathQuery& q = m_queue[i];
			// Allow to reuse the request.
			q.ref = PATHQ_INVALID;
			int n = 0;
			for (int j = 0; j < q.npath && j < maxPath; ++j)
				path[n++] = q.path[j];
			return n;
		}
	}
	return 0;	
}


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


Mover::Mover()
{
}

Mover::~Mover()
{
}

void Mover::init(dtPolyRef ref, const float* pos, const float radius, const float height,
				 const float collisionQueryRange, const float pathOptimizationRange)
{
	dtVcopy(m_pos, pos);
	dtVcopy(m_target, pos);
	m_radius = radius;
	m_height = height;

	m_path[0] = ref;
	m_npath = 1;

	dtVset(m_dvel, 0,0,0);
	dtVset(m_nvel, 0,0,0);
	dtVset(m_vel, 0,0,0);
	dtVset(m_npos, 0,0,0);

	m_pathOptimizationRange = pathOptimizationRange;
	m_collisionQueryRange = collisionQueryRange;

	dtVset(m_localCenter, 0,0,0);
	m_localSegCount = 0;

	m_ncorners = 0;
}

void Mover::updateLocalNeighbourhood(dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	if (!m_npath)
		return;
	
	// Only update the neigbourhood after certain distance has been passed.
	if (dtVdist2DSqr(m_pos, m_localCenter) < dtSqr(m_collisionQueryRange*0.25f))
		return;
	
	dtVcopy(m_localCenter, m_pos);
	static const int MAX_LOCALS = 32;
	dtPolyRef locals[MAX_LOCALS];
	
	const int nlocals = navquery->findLocalNeighbourhood(m_path[0], m_pos, m_collisionQueryRange,
														 filter, locals, 0, MAX_LOCALS);
	
	m_localSegCount = 0;
	for (int j = 0; j < nlocals; ++j)
	{
		float segs[DT_VERTS_PER_POLYGON*3*2];
		const int nsegs = navquery->getPolyWallSegments(locals[j], filter, segs);
		for (int k = 0; k < nsegs; ++k)
		{
			const float* s = &segs[k*6];
			// Skip too distant segments.
			float tseg;
			const float distSqr = dtDistancePtSegSqr2D(m_pos, s, s+3, tseg);
			if (distSqr > dtSqr(m_collisionQueryRange))
				continue;
			if (m_localSegCount < AGENT_MAX_LOCALSEGS)
			{
				memcpy(&m_localSegs[m_localSegCount*6], s, sizeof(float)*6);
				m_localSegCount++;
			}
		}
	}
}

float Mover::getDistanceToGoal(const float range) const
{
	if (!m_ncorners)
		return range;
	
	const bool endOfPath = (m_cornerFlags[m_ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
	const bool offMeshConnection = (m_cornerFlags[m_ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
	if (endOfPath || offMeshConnection)
		return dtMin(dtVdist2D(m_pos, &m_cornerVerts[(m_ncorners-1)*3]), range);
	
	return range;
}

void Mover::updateCorners(dtNavMeshQuery* navquery, const dtQueryFilter* filter, float* opts, float* opte)
{
	m_ncorners = 0;
	if (opts)
		dtVset(opts, 0,0,0);
	if (opte)
		dtVset(opte, 0,0,0);

	if (!m_npath)
		return;
	
	// Find nest couple of corners for steering.
	m_ncorners = findCorners(m_pos, m_target, m_path, m_npath,
							 m_cornerVerts, m_cornerFlags, m_cornerPolys,
							 AGENT_MAX_CORNERS, navquery);
	
	// Check to see if the corner after the next corner is directly visible,
	// and short cut to there.
	if (m_ncorners > 1)
	{
		if (opts)
			dtVcopy(opts, m_pos);
		if (opte)
			dtVcopy(opte, m_cornerVerts+3);
		
		m_npath = optimizePath(m_pos, m_cornerVerts+3, m_pathOptimizationRange,
							   m_path, m_npath, navquery, filter);
	}
}

void Mover::updatePosition(dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	dtAssert(m_npath);
	
	// Move along navmesh and update new position.
	float result[3];
	static const int MAX_VISITED = 16;
	dtPolyRef visited[MAX_VISITED];
	int nvisited = navquery->moveAlongSurface(m_path[0], m_pos, m_npos, filter,
											  result, visited, MAX_VISITED);
	m_npath = fixupCorridor(m_path, m_npath, AGENT_MAX_PATH, visited, nvisited);
	
	// Adjust agent height to stay on top of the navmesh.
	float h = m_pos[1];
	navquery->getPolyHeight(m_path[0], result, &h);
	result[1] = h;
	dtVcopy(m_pos, result);
}

void Mover::integrate(const float maxAcc, const float dt)
{
	// Fake dynamic constraint.
	const float maxDelta = maxAcc * dt;
	float dv[3];
	dtVsub(dv, m_nvel, m_vel);
	float ds = dtVlen(dv);
	if (ds > maxDelta)
		dtVscale(dv, dv, maxDelta/ds);
	dtVadd(m_vel, m_vel, dv);

	// Integrate
	if (dtVlen(m_vel) > 0.0001f)
		dtVmad(m_npos, m_pos, m_vel, dt);
	else
		dtVcopy(m_npos, m_pos);
}

void Mover::calcSmoothSteerDirection(float* dir)
{
	if (!m_ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}

	const int ip0 = 0;
	const int ip1 = dtMin(1, m_ncorners-1);
	const float* p0 = &m_cornerVerts[ip0*3];
	const float* p1 = &m_cornerVerts[ip1*3];
	
	float dir0[3], dir1[3];
	dtVsub(dir0, p0, m_pos);
	dtVsub(dir1, p1, m_pos);
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

void Mover::calcStraightSteerDirection(float* dir)
{
	if (!m_ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	dtVsub(dir, &m_cornerVerts[0], m_pos);
	dir[1] = 0;
	dtVnormalize(dir);
}

void Mover::appendLocalCollisionSegments(dtObstacleAvoidanceQuery* obstacleQuery)
{
	// Add static segment obstacles.
	for (int j = 0; j < m_localSegCount; ++j)
	{
		const float* s = &m_localSegs[j*6];
		if (dtTriArea2D(m_pos, s, s+3) < 0.0f)
			continue;
		
		float tseg;
		const float distSqr = dtDistancePtSegSqr2D(m_pos, s, s+3, tseg);
		
		obstacleQuery->addSegment(s, s+3, distSqr);
	}
}

void Mover::setNewPos(const float* npos)
{
	dtVcopy(m_npos, npos);
}

void Mover::setDesiredVelocity(const float* dvel)
{
	dtVcopy(m_dvel, dvel);
}

void Mover::setNewVelocity(const float* nvel)
{
	dtVcopy(m_nvel, nvel);
}

void Mover::setCorridor(const float* target, const dtPolyRef* path, int npath)
{
	dtAssert(npath > 0);
	dtAssert(npath < AGENT_MAX_PATH);
	dtVcopy(m_target, target);
	memcpy(m_path, path, sizeof(dtPolyRef)*npath);
	m_npath = npath;
}


CrowdManager::CrowdManager() :
	m_obstacleQuery(0),
	m_totalTime(0),
	m_rvoTime(0),
	m_sampleCount(0),
	m_moveRequestCount(0)
{
	dtVset(m_ext, 2,4,2);
	
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

	// TODO: the radius should be related to the agent radius used to create the navmesh!
	m_grid.init(100, 1.0f);
	
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

int CrowdManager::addAgent(const float* pos, const float radius, const float height, dtNavMeshQuery* navquery)
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

	// Find nearest position on navmesh and place the agent there.
	float nearest[3];
	dtPolyRef ref = navquery->findNearestPoly(pos, m_ext, &m_filter, nearest);
	if (!ref)
	{
		// Could not find a location on navmesh.
		return -1;
	}
	
	const float colRadius = radius * 7.5f;
	const float pathOptRange = colRadius * 4;

	ag->mover.init(ref, nearest, radius, height, colRadius, pathOptRange);

	ag->maxspeed = 0;
	ag->t = 0;
	dtVset(ag->opts, 0,0,0);
	dtVset(ag->opte, 0,0,0);
	ag->active = 1;
	ag->var = (rand() % 10) / 9.0f;
	
	// Init trail
	for (int i = 0; i < AGENT_MAX_TRAIL; ++i)
		dtVcopy(&ag->trail[i*3], ag->mover.getPos());
	ag->htrail = 0;
	
	return idx;
}

void CrowdManager::removeAgent(const int idx)
{
	if (idx >= 0 && idx < MAX_AGENTS)
		memset(&m_agents[idx], 0, sizeof(Agent));
}

bool CrowdManager::requestMoveTarget(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx > MAX_AGENTS)
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
		if (m_moveRequestCount >= MAX_AGENTS)
			return false;
		req = &m_moveRequests[m_moveRequestCount++];
	}
	
	// Initialize request.
	req->idx = idx;
	req->ref = ref;
	dtVcopy(req->pos, pos);
	req->pathqRef = PATHQ_INVALID;
	req->state = MR_TARGET_REQUESTING;

	return true;
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

int CrowdManager::getNeighbours(const float* pos, const float height, const float range,
								const Agent* skip, Agent** result, const int maxResult)
{
	int n = 0;
	
	unsigned short ids[MAX_AGENTS];
	int nids = m_grid.queryItems(pos[0]-range, pos[2]-range,
								 pos[0]+range, pos[2]+range,
								 ids, MAX_AGENTS);
	
	for (int i = 0; i < nids; ++i)
	{
		Agent* ag = &m_agents[ids[i]];

		if (ag == skip) continue;
		
		float diff[3];
		dtVsub(diff, pos, ag->mover.getPos());
		if (fabsf(diff[1]) >= (height+ag->mover.getHeight())/2.0f)
			continue;
		diff[1] = 0;
		const float distSqr = dtVlenSqr(diff);
		if (distSqr > dtSqr(range))
			continue;
		
		if (n < maxResult)
			result[n++] = ag;
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
	
	Agent* agents[MAX_AGENTS];
	Agent* neis[MAX_AGENTS];
	int nagents = getActiveAgents(agents, MAX_AGENTS);
	
	static const float MAX_ACC = 8.0f;
	static const float MAX_SPEED = 3.5f;

	// Update move requests.
	for (int i = 0; i < m_moveRequestCount; ++i)
	{
		MoveRequest* req = &m_moveRequests[i];
		Agent* ag = &m_agents[req->idx];

		if (!ag->active)
			req->state = MR_TARGET_FAILED;
			
		if (req->state == MR_TARGET_REQUESTING)
		{
			// Calculate request position.
			// If there is a lot of latency between requests, it is possible to
			// project the current position ahead and use raycast to find the actual
			// location and path.
			const dtPolyRef* cor = ag->mover.getCorridor();
			const int ncor = ag->mover.getCorridorCount();
			dtAssert(ncor);
			
			// Here we take the simple approach and set the path to be just the current location.
			float reqPos[3];
			dtVcopy(reqPos, ag->mover.getPos());	// The location of the request
			dtPolyRef reqPath[8];					// The path to the request location
			reqPath[0] = cor[0];
			int reqPathCount = 1;
			
			req->pathqRef = m_pathq.request(reqPath[reqPathCount-1], req->ref, reqPos, req->pos, &m_filter);
			if (req->pathqRef != PATHQ_INVALID)
			{
				ag->mover.setCorridor(reqPos, reqPath, reqPathCount);
				req->state = MR_TARGET_WAITING_FOR_PATH;
			}
		}
		else if (req->state == MR_TARGET_WAITING_FOR_PATH)
		{
			// Poll path queue.
			int state = m_pathq.getRequestState(req->pathqRef);
			if (state == PATHQ_STATE_INVALID)
			{
				req->pathqRef = PATHQ_INVALID;
				req->state = MR_TARGET_FAILED;
			}
			else if (state == PATHQ_STATE_READY)
			{
				const dtPolyRef* cor = ag->mover.getCorridor();
				const int ncor = ag->mover.getCorridorCount();
				dtAssert(ncor);

				// Apply results.
				float targetPos[3];
				dtVcopy(targetPos, req->pos);

				bool valid = true;
				dtPolyRef res[AGENT_MAX_PATH];
				int nres = m_pathq.getPathResult(req->pathqRef, res, AGENT_MAX_PATH);
				if (!nres)
					valid = false;

				// Merge result and existing path.
				// The agent might have moved whilst the request is
				// being processed, so the path may have changed.
				// We assume that the end of the path is at the same location
				// where the request was issued.
				
				// The last ref in the old path should be the same as
				// the location where the request was issued..
				if (valid && cor[ncor-1] != res[0])
					valid = false;
				
				if (valid)
				{
					// Put the old path infront of the old path.
					if (ncor > 1)
					{
						// Make space for the old path.
						if ((ncor-1)+nres > AGENT_MAX_PATH)
							nres = AGENT_MAX_PATH - (ncor-1);
						memmove(res+ncor-1, res, sizeof(dtPolyRef)*nres);
						// Copy old path in the beginning.
						memcpy(res, cor, sizeof(dtPolyRef)*(ncor-1));
						nres += ncor-1;
					}
					
					// Check for partial path.
					if (res[nres-1] != req->ref)
					{
						// Partial path, constrain target position inside the last polygon.
						float nearest[3];
						if (navquery->closestPointOnPoly(res[nres-1], targetPos, nearest))
							dtVcopy(targetPos, nearest);
						else
							valid = false;
					}
				}
				
				if (valid)
				{
					ag->mover.setCorridor(targetPos, res, nres);
					req->state = MR_TARGET_FAILED;
				}
				else
				{
					// Something went wrong.
					req->state = MR_TARGET_FAILED;
				}
			}
		}
		
		// Remove request.
		if (req->state == MR_TARGET_VALID || req->state == MR_TARGET_FAILED)
		{
			m_moveRequestCount--;
			if (i != m_moveRequestCount)
				memcpy(&m_moveRequests[i], &m_moveRequests[m_moveRequestCount], sizeof(MoveRequest));
			--i;
		}
	}
	
	m_pathq.update(navquery);
	

	// Register agents to proximity grid.
	m_grid.clear();
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		const float* p = ag->mover.getPos();
		const float r = ag->mover.getRadius();
		const float minx = p[0] - r;
		const float miny = p[2] - r;
		const float maxx = p[0] + r;
		const float maxy = p[2] + r;
		m_grid.addItem((unsigned short)i, minx, miny, maxx, maxy);
	}

	
	// Get nearby navmesh segments to collide with.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		ag->mover.updateLocalNeighbourhood(navquery, &m_filter);
	}
	
	// Find next corner to steer to.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		ag->mover.updateCorners(navquery, &m_filter, ag->opts, ag->opte);
	}
	
	// Calculate steering.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		float dvel[3] = {0,0,0};
		
		// Calculate steering direction.
		if (flags & CROWDMAN_ANTICIPATE_TURNS)
			ag->mover.calcSmoothSteerDirection(dvel);
		else
			ag->mover.calcStraightSteerDirection(dvel);
		
		// Calculate steering speed.
		
		// Calculate speed scale, which tells the agent to slowdown at the end of the path.
		const float slowDownRadius = ag->mover.getRadius()*2;	// TODO: make less hacky.
		const float speedScale = ag->mover.getDistanceToGoal(slowDownRadius) / slowDownRadius;
		
		// Apply style.
		if (flags & CROWDMAN_DRUNK)
		{
			// Drunken steering
			
			// Pulsating speed.
			ag->t += dt * (1.0f - ag->var*0.25f);
			ag->maxspeed = MAX_SPEED*(1 + dtSqr(cosf(ag->t*2.0f))*0.3f);
			
			dtVscale(dvel, dvel, ag->maxspeed * speedScale);
			
			// Slightly wandering steering.
			const float amp = cosf(ag->var*13.69f+ag->t*3.123f) * 0.2f;
			const float nx = -dvel[2];
			const float nz = dvel[0];
			dvel[0] += nx*amp;
			dvel[2] += nz*amp;
		}
		else
		{
			// Normal steering.
			ag->maxspeed = MAX_SPEED;
			dtVscale(dvel, dvel, ag->maxspeed * speedScale);
		}
		
		// Set the desired velocity.
		ag->mover.setDesiredVelocity(dvel);
	}
	
	// Velocity planning.
	TimeVal rvoStartTime = getPerfTime();
	
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		float nvel[3] = {0,0,0};
		
		if (flags & CROWDMAN_USE_VO)
		{
			m_obstacleQuery->reset();
			
			// Find neighbours and add them as obstacles.
			const int nneis = getNeighbours(ag->mover.getPos(), ag->mover.getHeight(),
											ag->mover.getCollisionQueryRange(),
											ag, neis, MAX_AGENTS);
			for (int j = 0; j < nneis; ++j)
			{
				const Agent* nei = neis[j];
				m_obstacleQuery->addCircle(nei->mover.getPos(), nei->mover.getRadius(),
										   nei->mover.getVelocity(), nei->mover.getDesiredVelocity(),
										   dtVdist2DSqr(ag->mover.getPos(), nei->mover.getPos()));
			}

			// Append neighbour segments as obstacles.
			ag->mover.appendLocalCollisionSegments(m_obstacleQuery);

			
			bool adaptive = true;

			if (adaptive)
			{
				m_obstacleQuery->setSamplingGridSize(VO_ADAPTIVE_GRID_SIZE);
				m_obstacleQuery->setSamplingGridDepth(VO_ADAPTIVE_GRID_DEPTH);
				m_obstacleQuery->sampleVelocityAdaptive(ag->mover.getPos(), ag->mover.getRadius(), ag->maxspeed,
														ag->mover.getVelocity(), ag->mover.getDesiredVelocity(),
														nvel, m_vodebug[i]);
			}
			else
			{
				m_obstacleQuery->setSamplingGridSize(VO_GRID_SIZE);
				m_obstacleQuery->sampleVelocity(ag->mover.getPos(), ag->mover.getRadius(), ag->maxspeed,
												ag->mover.getVelocity(), ag->mover.getDesiredVelocity(),
												nvel, m_vodebug[i]);
			}
		}
		else
		{
			// If not using velocity planning, new velocity is directly the desired velocity.
			dtVcopy(nvel, ag->mover.getDesiredVelocity());
		}
		
		ag->mover.setNewVelocity(nvel);
		
		
	}
	TimeVal rvoEndTime = getPerfTime();
	
	// Integrate.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		ag->mover.integrate(MAX_ACC, dt);
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
				dtVsub(diff, ag->mover.getNewPos(), nei->mover.getNewPos());
				
				if (fabsf(diff[1]) >= (ag->mover.getHeight()+nei->mover.getHeight())/2.0f)
					continue;
				
				diff[1] = 0;
				
				float dist = dtVlenSqr(diff);
				if (dist > dtSqr(ag->mover.getRadius()+nei->mover.getRadius()))
					continue;
				dist = sqrtf(dist);
				float pen = (ag->mover.getRadius()+nei->mover.getRadius()) - dist;
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
			float npos[3];
			dtVadd(npos, ag->mover.getNewPos(), ag->disp);
			ag->mover.setNewPos(npos);
		}
	}
	
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		// Move along navmesh.
		ag->mover.updatePosition(navquery, &m_filter);
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
		dtVcopy(&ag->trail[ag->htrail*3], ag->mover.getPos());
	}

	m_sampleCount = ns;
	m_rvoTime = getPerfDeltaTimeUsec(rvoStartTime, rvoEndTime);
	m_totalTime = getPerfDeltaTimeUsec(startTime, endTime);
}


