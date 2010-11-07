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

static const int VO_ADAPTIVE_DIVS = 7;
static const int VO_ADAPTIVE_RINGS = 2;
static const int VO_ADAPTIVE_DEPTH = 5;

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
			navquery->findPath(q.startRef, q.endRef, q.startPos, q.endPos,
							   q.filter, q.path, &q.npath, PQ_MAX_PATH);
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

static int fixupCorridorEnd(dtPolyRef* path, const int npath, const int maxPath,
							const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;
	
	// Find furthest common polygon.
	for (int i = 0; i < npath; ++i)
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
	const int ppos = furthestPath+1;
	const int vpos = furthestVisited+1;
	const int count = dtMin(nvisited-vpos, maxPath-ppos);
	dtAssert(ppos+count <= maxPath);
	if (count)
		memcpy(path+ppos, visited+vpos, sizeof(dtPolyRef)*count);

	return ppos+count;
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

PathCorridor::PathCorridor() :
	m_path(0),
	m_npath(0),
	m_maxPath(0)
{
	
}

PathCorridor::~PathCorridor()
{
	dtFree(m_path);
}

bool PathCorridor::init(const int maxPath)
{
	dtAssert(!m_path);
	m_path = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*maxPath, DT_ALLOC_PERM);
	if (!m_path)
		return false;
	m_npath = 0;
	m_maxPath = maxPath;
	return true;
}

void PathCorridor::reset(dtPolyRef ref, const float* pos)
{
	dtAssert(m_path);
	dtVcopy(m_pos, pos);
	dtVcopy(m_target, pos);
	m_path[0] = ref;
	m_npath = 1;
}

int PathCorridor::findCorners(float* cornerVerts, unsigned char* cornerFlags,
							  dtPolyRef* cornerPolys, const int maxCorners,
							  dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	dtAssert(m_path);
	dtAssert(m_npath);
	
	static const float MIN_TARGET_DIST = 0.01f;
	
	int ncorners = 0;
	navquery->findStraightPath(m_pos, m_target, m_path, m_npath,
							   cornerVerts, cornerFlags, cornerPolys, &ncorners, maxCorners);
	
	// Prune points in the beginning of the path which are too close.
	while (ncorners)
	{
		if ((cornerFlags[0] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			dtVdist2DSqr(&cornerVerts[0], m_pos) > dtSqr(MIN_TARGET_DIST))
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

void PathCorridor::optimizePathVisibility(const float* next, const float pathOptimizationRange,
										  dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	dtAssert(m_path);
	
	// Clamp the ray to max distance.
	float goal[3];
	dtVcopy(goal, next);
	float dist = dtVdist2D(m_pos, goal);
	
	// If too close to the goal, do not try to optimize.
	if (dist < 0.01f)
		return;

	// Overshoot a little. This helps to optimize open fields in tiled meshes.
	dist = dtMin(dist+0.01f, pathOptimizationRange);
	
	// Adjust ray length.
	float delta[3];
	dtVsub(delta, goal, m_pos);
	dtVmad(goal, m_pos, delta, pathOptimizationRange/dist);
	
	static const int MAX_RES = 32;
	dtPolyRef res[MAX_RES];
	float t, norm[3];
	int nres = 0;
	navquery->raycast(m_path[0], m_pos, goal, filter, &t, norm, res, &nres, MAX_RES);
	if (nres > 1 && t > 0.99f)
	{
		m_npath = mergeCorridor(m_path, m_npath, m_maxPath, res, nres);
	}
}

bool PathCorridor::optimizePathTopology(dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	dtAssert(m_path);

	if (m_npath < 3)
		return false;
	
	static const int MAX_ITER = 32;
	static const int MAX_RES = 32;

	dtPolyRef res[MAX_RES];
	int nres = 0;
	navquery->initSlicedFindPath(m_path[0], m_path[m_npath-1], m_pos, m_target, filter);
	navquery->updateSlicedFindPath(MAX_ITER);
	dtStatus status = navquery->finalizeSlicedFindPathPartial(m_path, m_npath, res, &nres, MAX_RES);

	if (status == DT_SUCCESS && nres > 0)
	{
		m_npath = mergeCorridor(m_path, m_npath, m_maxPath, res, nres);
		return true;
	}
	
	return false;
}

void PathCorridor::movePosition(const float* npos, dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	dtAssert(m_path);
	dtAssert(m_npath);
	
	// Move along navmesh and update new position.
	float result[3];
	static const int MAX_VISITED = 16;
	dtPolyRef visited[MAX_VISITED];
	int nvisited = 0;
	navquery->moveAlongSurface(m_path[0], m_pos, npos, filter,
							   result, visited, &nvisited, MAX_VISITED);
	m_npath = fixupCorridor(m_path, m_npath, m_maxPath, visited, nvisited);
	
	// Adjust the position to stay on top of the navmesh.
	float h = m_pos[1];
	navquery->getPolyHeight(m_path[0], result, &h);
	result[1] = h;
	dtVcopy(m_pos, result);
}

void PathCorridor::moveTargetPosition(const float* npos, dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	dtAssert(m_path);
	dtAssert(m_npath);
	
	// Move along navmesh and update new position.
	float result[3];
	static const int MAX_VISITED = 16;
	dtPolyRef visited[MAX_VISITED];
	int nvisited = 0;
	navquery->moveAlongSurface(m_path[m_npath-1], m_target, npos, filter,
							   result, visited, &nvisited, MAX_VISITED);
	m_npath = fixupCorridorEnd(m_path, m_npath, m_maxPath, visited, nvisited);
	
	// TODO: should we do that?
	// Adjust the position to stay on top of the navmesh.
/*	float h = m_target[1];
	navquery->getPolyHeight(m_path[m_npath-1], result, &h);
	result[1] = h;*/
	
	dtVcopy(m_target, result);
}

void PathCorridor::setCorridor(const float* target, const dtPolyRef* path, const int npath)
{
	dtAssert(m_path);
	dtAssert(npath > 0);
	dtAssert(npath < m_maxPath);
	
	dtVcopy(m_target, target);
	memcpy(m_path, path, sizeof(dtPolyRef)*npath);
	m_npath = npath;
}




void Agent::integrate(const float maxAcc, const float dt)
{
	// Fake dynamic constraint.
	const float maxDelta = maxAcc * dt;
	float dv[3];
	dtVsub(dv, nvel, vel);
	float ds = dtVlen(dv);
	if (ds > maxDelta)
		dtVscale(dv, dv, maxDelta/ds);
	dtVadd(vel, vel, dv);
	
	// Integrate
	if (dtVlen(vel) > 0.0001f)
		dtVmad(npos, npos, vel, dt);
	else
		dtVset(vel,0,0,0);
}

float Agent::getDistanceToGoal(const float range) const
{
	if (!ncorners)
		return range;
	
	const bool endOfPath = (cornerFlags[ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
	const bool offMeshConnection = (cornerFlags[ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
	if (endOfPath || offMeshConnection)
		return dtMin(dtVdist2D(npos, &cornerVerts[(ncorners-1)*3]), range);
	
	return range;
}

void Agent::calcSmoothSteerDirection(float* dir)
{
	if (!ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	
	const int ip0 = 0;
	const int ip1 = dtMin(1, ncorners-1);
	const float* p0 = &cornerVerts[ip0*3];
	const float* p1 = &cornerVerts[ip1*3];
	
	float dir0[3], dir1[3];
	dtVsub(dir0, p0, npos);
	dtVsub(dir1, p1, npos);
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

void Agent::calcStraightSteerDirection(float* dir)
{
	if (!ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	dtVsub(dir, &cornerVerts[0], npos);
	dir[1] = 0;
	dtVnormalize(dir);
}



LocalBoundary::LocalBoundary() :
	m_nsegs(0)
{
	dtVset(m_center, FLT_MAX,FLT_MAX,FLT_MAX);
}

LocalBoundary::~LocalBoundary()
{
}
	
void LocalBoundary::reset()
{
	dtVset(m_center, FLT_MAX,FLT_MAX,FLT_MAX);
	m_nsegs = 0;
}

void LocalBoundary::addSegment(const float dist, const float* s)
{
	// Insert neighbour based on the distance.
	Segment* seg = 0;
	if (!m_nsegs)
	{
		// First, trivial accept.
		seg = &m_segs[0];
	}
	else if (dist >= m_segs[m_nsegs-1].d)
	{
		// Further than the last segment, skip.
		if (m_nsegs >= MAX_SEGS)
			return;
		// Last, trivial accept.
		seg = &m_segs[m_nsegs];
	}
	else
	{
		// Insert inbetween.
		int i;
		for (i = 0; i < m_nsegs; ++i)
			if (dist <= m_segs[i].d)
				break;
		const int tgt = i+1;
		const int n = dtMin(m_nsegs-i, MAX_SEGS-tgt);
		dtAssert(tgt+n <= MAX_SEGS);
		if (n > 0)
			memmove(&m_segs[tgt], &m_segs[i], sizeof(Segment)*n);
		seg = &m_segs[i];
	}
	
	seg->d = dist;
	memcpy(seg->s, s, sizeof(float)*6);
	
	if (m_nsegs < MAX_SEGS)
		m_nsegs++;
}

void LocalBoundary::update(dtPolyRef ref, const float* pos, const float collisionQueryRange,
						   dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	static const int MAX_LOCAL_POLYS = 16;
	static const int MAX_SEGS_PER_POLY = DT_VERTS_PER_POLYGON*2;
	
	if (!ref)
	{
		dtVset(m_center, FLT_MAX,FLT_MAX,FLT_MAX);
		m_nsegs = 0;
		return;
	}
	
	dtVcopy(m_center, pos);
	
	// First query non-overlapping polygons.
	dtPolyRef locals[MAX_LOCAL_POLYS];
	int nlocals = 0;
	navquery->findLocalNeighbourhood(ref, pos, collisionQueryRange,
									 filter, locals, 0, &nlocals, MAX_LOCAL_POLYS);
	
	// Secondly, store all polygon edges.
	m_nsegs = 0;
	float segs[MAX_SEGS_PER_POLY*6];
	int nsegs = 0;
	for (int j = 0; j < nlocals; ++j)
	{
		navquery->getPolyWallSegments(locals[j], filter, segs, &nsegs, MAX_SEGS_PER_POLY);
		for (int k = 0; k < nsegs; ++k)
		{
			const float* s = &segs[k*6];
			// Skip too distant segments.
			float tseg;
			const float distSqr = dtDistancePtSegSqr2D(pos, s, s+3, tseg);
			if (distSqr > dtSqr(collisionQueryRange))
				continue;
			addSegment(distSqr, s);
		}
	}
}


CrowdManager::CrowdManager() :
	m_obstacleQuery(0),
	m_pathResult(0),
	m_maxPathResult(0),
	m_totalTime(0),
	m_rvoTime(0),
	m_sampleCount(0),
	m_moveRequestCount(0)
{
	dtVset(m_ext, 2,4,2);
	
	m_obstacleQuery = dtAllocObstacleAvoidanceQuery();
	m_obstacleQuery->init(6, 8);
	
	m_obstacleQuery->setDesiredVelocityWeight(2.0f);
	m_obstacleQuery->setCurrentVelocityWeight(0.75f);
	m_obstacleQuery->setPreferredSideWeight(0.75f);
	m_obstacleQuery->setCollisionTimeWeight(2.5f);
	m_obstacleQuery->setTimeHorizon(2.5f);
	m_obstacleQuery->setVelocitySelectionBias(0.4f);
	
	memset(m_vodebug, 0, sizeof(m_vodebug));
	const int maxAdaptiveSamples = (VO_ADAPTIVE_DIVS*VO_ADAPTIVE_RINGS+1)*VO_ADAPTIVE_DEPTH;
	const int maxGridSamples = VO_GRID_SIZE*VO_GRID_SIZE;
	const int sampleCount = dtMax(maxAdaptiveSamples, maxGridSamples);
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		m_vodebug[i] = dtAllocObstacleAvoidanceDebugData();
		m_vodebug[i]->init(sampleCount);
	}
	
	// Allocate temp buffer for merging paths.
	m_maxPathResult = 256;
	m_pathResult = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*m_maxPathResult, DT_ALLOC_PERM);

	// Alloca corridors.
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		m_agents[i].corridor.init(m_maxPathResult);
	}

	// TODO: the radius should be related to the agent radius used to create the navmesh!
	m_grid.init(100, 1.0f);
	
	reset();
}

CrowdManager::~CrowdManager()
{
	delete [] m_pathResult;
	
	for (int i = 0; i < MAX_AGENTS; ++i)
		dtFreeObstacleAvoidanceDebugData(m_vodebug[i]);
	dtFreeObstacleAvoidanceQuery(m_obstacleQuery);
}

void CrowdManager::reset()
{
	for (int i = 0; i < MAX_AGENTS; ++i)
		m_agents[i].active = 0;
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
	dtPolyRef ref;
	navquery->findNearestPoly(pos, m_ext, &m_filter, &ref, nearest);
	if (!ref)
	{
		// Could not find a location on navmesh.
		return -1;
	}
	
	ag->corridor.reset(ref, nearest);
	ag->boundary.reset();

	ag->radius = radius;
	ag->height = height;
	ag->collisionQueryRange = radius * 8;
	ag->pathOptimizationRange = radius * 30;
	ag->topologyOptTime = 0;
	ag->nneis = 0;
	
	dtVset(ag->dvel, 0,0,0);
	dtVset(ag->nvel, 0,0,0);
	dtVset(ag->vel, 0,0,0);
	dtVcopy(ag->npos, nearest);
	
	ag->maxspeed = 0;
	ag->t = 0;
	dtVset(ag->opts, 0,0,0);
	dtVset(ag->opte, 0,0,0);
	ag->active = 1;
	ag->var = (rand() % 10) / 9.0f;
	
	// Init trail
	for (int i = 0; i < AGENT_MAX_TRAIL; ++i)
		dtVcopy(&ag->trail[i*3], ag->corridor.getPos());
	ag->htrail = 0;
	
	return idx;
}

void CrowdManager::removeAgent(const int idx)
{
	if (idx >= 0 && idx < MAX_AGENTS)
	{
		m_agents[idx].active = 0;
	}
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
		memset(req, 0, sizeof(MoveRequest));
	}
	
	// Initialize request.
	req->idx = idx;
	req->ref = ref;
	dtVcopy(req->pos, pos);
	req->pathqRef = PATHQ_INVALID;
	req->state = MR_TARGET_REQUESTING;

	req->temp[0] = ref;
	req->ntemp = 1;

	return true;
}

bool CrowdManager::adjustMoveTarget(const int idx, dtPolyRef ref, const float* pos)
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


static int addNeighbour(const int idx, const float dist,
						Neighbour* neis, const int nneis, const int maxNeis)
{
	// Insert neighbour based on the distance.
	Neighbour* nei = 0;
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
			memmove(&neis[tgt], &neis[i], sizeof(Neighbour)*n);
		nei = &neis[i];
	}
	
	memset(nei, 0, sizeof(Neighbour));

	nei->idx = idx;
	nei->dist = dist;
	
	return dtMin(nneis+1, maxNeis);
}

int CrowdManager::getNeighbours(const float* pos, const float height, const float range,
								const Agent* skip, Neighbour* result, const int maxResult)
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

void CrowdManager::updateMoveRequest(const float dt, dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	// Fire off new requests.
	for (int i = 0; i < m_moveRequestCount; ++i)
	{
		MoveRequest* req = &m_moveRequests[i];
		Agent* ag = &m_agents[req->idx];
		
		// Agent not active anymore, kill request.
		if (!ag->active)
			req->state = MR_TARGET_FAILED;
		
		// Adjust target
		if (req->aref)
		{
			if (req->state == MR_TARGET_ADJUST)
			{
				// Adjust existing path.
				ag->corridor.moveTargetPosition(req->apos, navquery, filter);
				req->state = MR_TARGET_VALID;
			}
			else
			{
				// Adjust on the flight request.
				float result[3];
				static const int MAX_VISITED = 16;
				dtPolyRef visited[MAX_VISITED];
				int nvisited = 0;
				navquery->moveAlongSurface(req->temp[req->ntemp-1], req->pos, req->apos, filter,
										   result, visited, &nvisited, MAX_VISITED);
				req->ntemp = fixupCorridorEnd(req->temp, req->ntemp, MAX_TEMP_PATH, visited, nvisited);
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
			if (req->pathqRef != PATHQ_INVALID)
			{
				ag->corridor.setCorridor(reqPos, reqPath, reqPathCount);
				req->state = MR_TARGET_WAITING_FOR_PATH;
			}
		}
	}
	
	// Update requests.
	m_pathq.update(navquery);
	

	// Process path results.
	for (int i = 0; i < m_moveRequestCount; ++i)
	{
		MoveRequest* req = &m_moveRequests[i];
		Agent* ag = &m_agents[req->idx];
		
		if (req->state == MR_TARGET_WAITING_FOR_PATH)
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
				const dtPolyRef* path = ag->corridor.getPath();
				const int npath = ag->corridor.getPathCount();
				dtAssert(npath);
				
				// Apply results.
				float targetPos[3];
				dtVcopy(targetPos, req->pos);
				
				dtPolyRef* res = m_pathResult;
				bool valid = true;
				int nres = m_pathq.getPathResult(req->pathqRef, res, m_maxPathResult);
				if (!nres)
					valid = false;
				
				// Merge with any target adjustment that happened during the search.
				if (req->ntemp > 1)
				{
					nres = fixupCorridorEnd(res, nres, m_maxPathResult, req->temp, req->ntemp);
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
						if (navquery->closestPointOnPoly(res[nres-1], targetPos, nearest) == DT_SUCCESS)
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



static int addToOptQueue(Agent* newag, Agent** agents, const int nagents, const int maxAgents)
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
			memmove(&agents[tgt], &agents[i], sizeof(Agent*)*n);
		slot = i;
	}

	agents[slot] = newag;
	
	return dtMin(nagents+1, maxAgents);
}

void CrowdManager::updateTopologyOptimization(const float dt, dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	Agent* agents[MAX_AGENTS];
	int nagents = getActiveAgents(agents, MAX_AGENTS);
	if (!nagents)
		return;
	
	const float OPT_TIME_THR = 0.5f; // seconds
	const int OPT_MAX_AGENTS = 1;
	
	Agent* queue[OPT_MAX_AGENTS];
	int nqueue = 0;
	
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		ag->topologyOptTime += dt;
		if (ag->topologyOptTime >= OPT_TIME_THR)
		{
			nqueue = addToOptQueue(ag, queue, nqueue, OPT_MAX_AGENTS);
		}
	}

	for (int i = 0; i < nqueue; ++i)
	{
		Agent* ag = queue[i];
		ag->corridor.optimizePathTopology(navquery, filter);
		ag->topologyOptTime = 0;
	}

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
	int nagents = getActiveAgents(agents, MAX_AGENTS);
	
	static const float MAX_ACC = 8.0f;
	static const float MAX_SPEED = 3.5f;

	// Update async move request and path finder.
	updateMoveRequest(dt, navquery, &m_filter);

	// Optimize path topology.
	if (flags & CROWDMAN_OPTIMIZE_TOPO)
		updateTopologyOptimization(dt, navquery, &m_filter);
	
	// Register agents to proximity grid.
	m_grid.clear();
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		const float* p = ag->npos;
		const float r = ag->radius;
		m_grid.addItem((unsigned short)i, p[0]-r, p[2]-r, p[0]+r, p[2]+r);
	}
	
	// Get nearby navmesh segments and agents to collide with.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		// Only update the collision boundary after certain distance has been passed.
		if (dtVdist2DSqr(ag->npos, ag->boundary.getCenter()) > dtSqr(ag->collisionQueryRange*0.25f))
			ag->boundary.update(ag->corridor.getFirstPoly(), ag->npos, ag->collisionQueryRange, navquery, &m_filter);
		// Query neighbour agents
		ag->nneis = getNeighbours(ag->npos, ag->height, ag->collisionQueryRange, ag, ag->neis, AGENT_MAX_NEIGHBOURS);
	}
	
	// Find next corner to steer to.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		// Find corners for steering
		ag->ncorners = ag->corridor.findCorners(ag->cornerVerts, ag->cornerFlags, ag->cornerPolys,
												AGENT_MAX_CORNERS, navquery, &m_filter);
		
		// Check to see if the corner after the next corner is directly visible,
		// and short cut to there.
		if ((flags & CROWDMAN_OPTIMIZE_VIS) && ag->ncorners > 0)
		{
			const float* target = &ag->cornerVerts[dtMin(1,ag->ncorners-1)*3];
			dtVcopy(ag->opts, ag->corridor.getPos());
			dtVcopy(ag->opte, target);
			ag->corridor.optimizePathVisibility(target, ag->pathOptimizationRange, navquery, &m_filter);
		}
		else
		{
			dtVset(ag->opts, 0,0,0);
			dtVset(ag->opte, 0,0,0);
		}
		
		// Copy data for debug purposes.
	}
	
	// Calculate steering.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		float dvel[3] = {0,0,0};
		
		// Calculate steering direction.
		if (flags & CROWDMAN_ANTICIPATE_TURNS)
			ag->calcSmoothSteerDirection(dvel);
		else
			ag->calcStraightSteerDirection(dvel);
		
		// Calculate speed scale, which tells the agent to slowdown at the end of the path.
		const float slowDownRadius = ag->radius*2;	// TODO: make less hacky.
		const float speedScale = ag->getDistanceToGoal(slowDownRadius) / slowDownRadius;
		
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
		dtVcopy(ag->dvel, dvel);
	}
	
	// Velocity planning.
	TimeVal rvoStartTime = getPerfTime();
	
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		
		if (flags & CROWDMAN_USE_VO)
		{
			m_obstacleQuery->reset();
			
			// Add neighbours as obstacles.
			for (int j = 0; j < ag->nneis; ++j)
			{
				const Agent* nei = &m_agents[ag->neis[j].idx];
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

			// Sample new safe velocity.
			bool adaptive = true;

			if (adaptive)
			{
				m_obstacleQuery->sampleVelocityAdaptive(ag->npos, ag->radius, ag->maxspeed,
														ag->vel, ag->dvel, ag->nvel,
														VO_ADAPTIVE_DIVS, VO_ADAPTIVE_RINGS, VO_ADAPTIVE_DEPTH,
														m_vodebug[i]);
			}
			else
			{
				m_obstacleQuery->sampleVelocityGrid(ag->npos, ag->radius, ag->maxspeed,
													ag->vel, ag->dvel, ag->nvel,
													VO_GRID_SIZE, m_vodebug[i]);
			}
		}
		else
		{
			// If not using velocity planning, new velocity is directly the desired velocity.
			dtVcopy(ag->nvel, ag->dvel);
		}
	}
	TimeVal rvoEndTime = getPerfTime();
	
	// Integrate.
	for (int i = 0; i < nagents; ++i)
	{
		Agent* ag = agents[i];
		ag->integrate(MAX_ACC, dt);
	}
	
	// Handle collisions.
	for (int iter = 0; iter < 4; ++iter)
	{
		for (int i = 0; i < nagents; ++i)
		{
			Agent* ag = agents[i];
			
			dtVset(ag->disp, 0,0,0);
			
			float w = 0;

			for (int j = 0; j < ag->nneis; ++j)
			{
				const Agent* nei = &m_agents[ag->neis[j].idx];
				
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
		// Move along navmesh.
		ag->corridor.movePosition(ag->npos, navquery, &m_filter);
		// Get valid constrained position back.
		dtVcopy(ag->npos, ag->corridor.getPos());
	}
	
	TimeVal endTime = getPerfTime();

	// Debug/demo book keeping
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
		dtVcopy(&ag->trail[ag->htrail*3], ag->npos);
	}

	m_sampleCount = ns;
	m_rvoTime = getPerfDeltaTimeUsec(rvoStartTime, rvoEndTime);
	m_totalTime = getPerfDeltaTimeUsec(startTime, endTime);
}


