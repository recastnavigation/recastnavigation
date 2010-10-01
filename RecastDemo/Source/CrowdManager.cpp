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

void Mover::init(const float* p, const float r, const float h, const float cr, const float por)
{
	dtVcopy(m_pos, p);
	dtVcopy(m_target, p);
	m_radius = r;
	m_height = h;

	dtVset(m_dvel, 0,0,0);
	dtVset(m_nvel, 0,0,0);
	dtVset(m_vel, 0,0,0);
	dtVset(m_npos, 0,0,0);
	dtVset(m_disp, 0,0,0);

	m_pathOptimizationRange = por;
	m_colradius = cr;

	dtVset(m_localCenter, 0,0,0);
	m_localSegCount = 0;

	m_state = MOVER_INIT;

	m_reqTargetState = MOVER_TARGET_NONE;
	dtVset(m_reqTarget, 0,0,0);
	m_reqTargetRef = 0;

	m_pathReqRef = PATHQ_INVALID;
	m_npath = 0;
	
	m_ncorners = 0;
}

void Mover::requestMoveTarget(const float* pos)
{
	dtVcopy(m_reqTarget, pos);
	m_reqTargetRef = 0;
	m_reqTargetState = MOVER_TARGET_REQUESTING;
	m_pathReqRef = PATHQ_INVALID;
}

void Mover::updatePathState(dtNavMeshQuery* navquery, const dtQueryFilter* filter, const float* ext,
							PathQueue* pathq)
{
	// Make sure that the first path polygon corresponds to the current agent location.
	if (m_state == MOVER_INIT)
	{
		float nearest[3];
		dtPolyRef ref = navquery->findNearestPoly(m_pos, ext, filter, nearest);
		if (ref)
		{
			m_path[0] = ref;
			m_npath = 1;
			dtVcopy(m_pos, nearest);
			dtVcopy(m_target, nearest);
			m_state = MOVER_OK;
		}
		else
		{
			m_state = MOVER_FAILED;
		}
	}
	
	if (m_state == MOVER_FAILED)
	{
		return;
	}
	
	if (m_reqTargetState == MOVER_TARGET_REQUESTING)
	{
		float nearest[3];
		m_reqTargetRef = navquery->findNearestPoly(m_reqTarget, ext, filter, nearest);
		if (m_reqTargetRef)
		{
			// Calculate request position.
			// If there is a lot of latency between requests, it is possible to
			// project the current position ahead and use raycast to find the actual
			// location and path.
			// Here we take the simple route and set the path to be just the current location.
			float reqPos[3];
			dtVcopy(reqPos, m_pos);		// The location of the request
			dtPolyRef reqPath[8];		// The path to the request location
			reqPath[0] = m_path[0];
			int reqPathCount = 1;
			
			dtVcopy(m_reqTarget, nearest);
			m_pathReqRef = pathq->request(reqPath[reqPathCount-1], m_reqTargetRef, reqPos, m_reqTarget, filter);
			if (m_pathReqRef != PATHQ_INVALID)
			{
				dtVcopy(m_target, reqPos);
				memcpy(m_path, reqPath, sizeof(dtPolyRef)*reqPathCount);
				m_npath = reqPathCount;

				m_reqTargetState = MOVER_TARGET_WAITING_FOR_PATH;
			}
		}
		else
		{
			m_reqTargetState = MOVER_TARGET_FAILED;
		}
	}
	
	if (m_reqTargetState == MOVER_TARGET_WAITING_FOR_PATH)
	{
		// Poll path queue.
		int state = pathq->getRequestState(m_pathReqRef);
		if (state == PATHQ_STATE_INVALID)
		{
			m_pathReqRef = PATHQ_INVALID;
			m_reqTargetState = MOVER_TARGET_FAILED;
		}
		else if (state == PATHQ_STATE_READY)
		{
			dtAssert(m_npath);
			
			// Merge new results and current path.
			dtPolyRef res[AGENT_MAX_PATH];
			int nres = 0;
			nres = pathq->getPathResult(m_pathReqRef, res, AGENT_MAX_PATH);
			
			if (!nres)
			{
				m_reqTargetState = MOVER_TARGET_FAILED;
			}
			else
			{
				// The last ref in the old path should be the same as the first ref of new path.
				if (m_path[m_npath-1] == res[0])
				{
					// Append new path
					if (m_npath-1+nres > AGENT_MAX_PATH)
						nres = AGENT_MAX_PATH-(m_npath-1);
					memcpy(&m_path[m_npath-1], res, sizeof(dtPolyRef)*nres);
					m_npath = m_npath-1 + nres;

					dtVcopy(m_target, m_reqTarget);

					// Check for partial path.
					if (m_path[m_npath-1] != m_reqTargetRef)
					{
						// Partial path, constrain target position inside the last polygon.
						float nearest[3];
						if (navquery->closestPointOnPoly(m_path[m_npath-1], m_target, nearest))
							dtVcopy(m_target, nearest);
						else
							m_reqTargetState = MOVER_TARGET_FAILED;
					}
				}
				else
				{
					// Something went out of sync.
					m_reqTargetState = MOVER_TARGET_FAILED;
				}
			}
			
			m_pathReqRef = PATHQ_INVALID;
		}
	}
}


void Mover::updateLocalNeighbourhood(dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	if (!m_npath)
		return;
	
	// Only update the neigbourhood after certain distance has been passed.
	if (dtVdist2DSqr(m_pos, m_localCenter) < dtSqr(m_colradius*0.25f))
		return;
	
	dtVcopy(m_localCenter, m_pos);
	static const int MAX_LOCALS = 32;
	dtPolyRef locals[MAX_LOCALS];
	
	const int nlocals = navquery->findLocalNeighbourhood(m_path[0], m_pos, m_colradius, filter, locals, 0, MAX_LOCALS);
	
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
			if (distSqr > dtSqr(m_colradius))
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

void Mover::updateLocation(dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	if (!m_npath)
		return;
	
	// Move along navmesh and update new position.
	float result[3];
	dtPolyRef visited[16];
	int nvisited = navquery->moveAlongSurface(m_path[0], m_pos, m_npos, filter,
											  result, visited, 16);
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
//	memset(ag, 0, sizeof(Agent));
	
	const float colRadius = radius * 7.5f;
	const float pathOptRange = colRadius * 4;

	ag->mover.init(pos, radius, height, colRadius, pathOptRange);

	ag->maxspeed = 0;
	ag->t = 0;
	dtVset(ag->opts, 0,0,0);
	dtVset(ag->opte, 0,0,0);
	ag->active = 1;
	ag->var = (rand() % 10) / 9.0f;
	
	// Init trail
	for (int i = 0; i < AGENT_MAX_TRAIL; ++i)
		dtVcopy(&ag->trail[i*3], ag->mover.m_pos);
	ag->htrail = 0;
	
	return idx;
}

void CrowdManager::removeAgent(const int idx)
{
	if (idx >= 0 && idx < MAX_AGENTS)
		memset(&m_agents[idx], 0, sizeof(Agent));
}

void CrowdManager::requestMoveTarget(const int idx, const float* pos)
{
	Agent* ag = &m_agents[idx];
	ag->mover.requestMoveTarget(pos);
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
		dtVsub(diff, pos, ag->mover.m_pos);
		if (fabsf(diff[1]) >= (height+ag->mover.m_height)/2.0f)
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
	
	const float ext[3] = {2,4,2};
	dtQueryFilter filter;
	
	Agent* agents[MAX_AGENTS];
	Agent* neis[MAX_AGENTS];
	int nagents = getActiveAgents(agents, MAX_AGENTS);
	
	static const float MAX_ACC = 8.0f;
	static const float MAX_SPEED = 3.5f;

	m_pathq.update(navquery);
	

	m_grid.clear();
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		const float* p = ag->mover.m_pos;
		const float r = ag->mover.m_radius;
		const float minx = p[0] - r;
		const float miny = p[2] - r;
		const float maxx = p[0] + r;
		const float maxy = p[2] + r;
		m_grid.addItem((unsigned short)i, minx, miny, maxx, maxy);
	}

	
	// Update target and agent navigation state.
	// TODO: add queue for path queries.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		ag->mover.updatePathState(navquery, &filter, ext, &m_pathq);
	}
	
	// Get nearby navmesh segments to collide with.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		ag->mover.updateLocalNeighbourhood(navquery, &filter);
	}
	
	// Find next corner to steer to.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		ag->mover.updateCorners(navquery, &filter, ag->opts, ag->opte);
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
		const float slowDownRadius = ag->mover.m_radius*2;
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
		dtVcopy(ag->mover.m_dvel, dvel);
	}
	
	// Velocity planning.
	TimeVal rvoStartTime = getPerfTime();
	
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		if (flags & CROWDMAN_USE_VO)
		{
			m_obstacleQuery->reset();
			
			// Find neighbours and add them as obstacles.
			const int nneis = getNeighbours(ag->mover.m_pos, ag->mover.m_height, ag->mover.m_colradius,
											ag, neis, MAX_AGENTS);
			for (int j = 0; j < nneis; ++j)
			{
				const Agent* nei = neis[j];
				m_obstacleQuery->addCircle(nei->mover.m_pos, nei->mover.m_radius, nei->mover.m_vel, nei->mover.m_dvel,
										   dtVdist2DSqr(ag->mover.m_pos, nei->mover.m_pos));
			}

			// Append neighbour segments as obstacles.
			ag->mover.appendLocalCollisionSegments(m_obstacleQuery);

			
			bool adaptive = true;
			
			if (adaptive)
			{
				m_obstacleQuery->setSamplingGridSize(VO_ADAPTIVE_GRID_SIZE);
				m_obstacleQuery->setSamplingGridDepth(VO_ADAPTIVE_GRID_DEPTH);
				m_obstacleQuery->sampleVelocityAdaptive(ag->mover.m_pos, ag->mover.m_radius, ag->maxspeed,
														ag->mover.m_vel, ag->mover.m_dvel, ag->mover.m_nvel,
														m_vodebug[i]);
			}
			else
			{
				m_obstacleQuery->setSamplingGridSize(VO_GRID_SIZE);
				m_obstacleQuery->sampleVelocity(ag->mover.m_pos, ag->mover.m_radius, ag->maxspeed,
												ag->mover.m_vel, ag->mover.m_dvel, ag->mover.m_nvel,
												m_vodebug[i]);
			}
		}
		else
		{
			dtVcopy(ag->mover.m_nvel, ag->mover.m_dvel);
		}
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
			
			dtVset(ag->mover.m_disp, 0,0,0);
			
			float w = 0;
			
			for (int j = 0; j < nagents; ++j)
			{
				if (i == j) continue;
				Agent* nei = agents[j];
				
				float diff[3];
				dtVsub(diff, ag->mover.m_npos, nei->mover.m_npos);
				
				if (fabsf(diff[1]) >= (ag->mover.m_height+nei->mover.m_height)/2.0f)
					continue;
				
				diff[1] = 0;
				
				float dist = dtVlenSqr(diff);
				if (dist > dtSqr(ag->mover.m_radius+nei->mover.m_radius))
					continue;
				dist = sqrtf(dist);
				float pen = (ag->mover.m_radius+nei->mover.m_radius) - dist;
				if (dist > 0.0001f)
					pen = (1.0f/dist) * (pen*0.5f) * 0.7f;
				
				dtVmad(ag->mover.m_disp, ag->mover.m_disp, diff, pen);			
				
				w += 1.0f;
			}
			
			if (w > 0.0001f)
			{
				const float iw = 1.0f / w;
				dtVscale(ag->mover.m_disp, ag->mover.m_disp, iw);
			}
		}
		
		for (int i = 0; i < nagents; ++i)
		{
			Agent* ag = agents[i];
			dtVadd(ag->mover.m_npos, ag->mover.m_npos, ag->mover.m_disp);
		}
	}
	
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		// Move along navmesh.
		ag->mover.updateLocation(navquery, &filter);
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
		dtVcopy(&ag->trail[ag->htrail*3], ag->mover.m_pos);
	}

	m_sampleCount = ns;
	m_rvoTime = getPerfDeltaTimeUsec(rvoStartTime, rvoEndTime);
	m_totalTime = getPerfDeltaTimeUsec(startTime, endTime);
}


