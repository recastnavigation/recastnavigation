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

#include <string.h>
#include "DetourPathCorridor.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "DetourAssert.h"
#include "DetourAlloc.h"


int dtMergeCorridorStartMoved(dtPolyRef* path, const int npath, const int maxPath,
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

int dtMergeCorridorEndMoved(dtPolyRef* path, const int npath, const int maxPath,
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

int dtMergeCorridorStartShortcut(dtPolyRef* path, const int npath, const int maxPath,
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

dtPathCorridor::dtPathCorridor() :
	m_path(0),
	m_npath(0),
	m_maxPath(0)
{
}

dtPathCorridor::~dtPathCorridor()
{
	dtFree(m_path);
}

bool dtPathCorridor::init(const int maxPath)
{
	dtAssert(!m_path);
	m_path = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*maxPath, DT_ALLOC_PERM);
	if (!m_path)
		return false;
	m_npath = 0;
	m_maxPath = maxPath;
	return true;
}

void dtPathCorridor::reset(dtPolyRef ref, const float* pos)
{
	dtAssert(m_path);
	dtVcopy(m_pos, pos);
	dtVcopy(m_target, pos);
	m_path[0] = ref;
	m_npath = 1;
}

int dtPathCorridor::findCorners(float* cornerVerts, unsigned char* cornerFlags,
							  dtPolyRef* cornerPolys, const int maxCorners,
							  dtNavMeshQuery* navquery, const dtQueryFilter* /*filter*/)
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

void dtPathCorridor::optimizePathVisibility(const float* next, const float pathOptimizationRange,
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
		m_npath = dtMergeCorridorStartShortcut(m_path, m_npath, m_maxPath, res, nres);
	}
}

bool dtPathCorridor::optimizePathTopology(dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	dtAssert(m_path);
	
	if (m_npath < 3)
		return false;
	
	static const int MAX_ITER = 32;
	static const int MAX_RES = 32;
	
	dtPolyRef res[MAX_RES];
	int nres = 0;
	navquery->initSlicedFindPath(m_path[0], m_path[m_npath-1], m_pos, m_target, filter);
	navquery->updateSlicedFindPath(MAX_ITER, 0);
	dtStatus status = navquery->finalizeSlicedFindPathPartial(m_path, m_npath, res, &nres, MAX_RES);
	
	if (status == DT_SUCCESS && nres > 0)
	{
		m_npath = dtMergeCorridorStartShortcut(m_path, m_npath, m_maxPath, res, nres);
		return true;
	}
	
	return false;
}

bool dtPathCorridor::moveOverOffmeshConnection(dtPolyRef offMeshConRef, dtPolyRef* refs,
											   float* startPos, float* endPos,
											   dtNavMeshQuery* navquery)
{
	dtAssert(navquery);
	dtAssert(m_path);
	dtAssert(m_npath);

	// Advance the path up to and over the off-mesh connection.
	dtPolyRef prevRef = 0, polyRef = m_path[0];
	int npos = 0;
	while (npos < m_npath && polyRef != offMeshConRef)
	{
		prevRef = polyRef;
		polyRef = m_path[npos];
		npos++;
	}
	if (npos == m_npath)
	{
		// Could not find offMeshConRef
		return false;
	}
	
	// Prune path
	for (int i = npos; i < m_npath; ++i)
		m_path[i-npos] = m_path[i];
	m_npath -= npos;

	refs[0] = prevRef;
	refs[1] = polyRef;
	
	const dtNavMesh* nav = navquery->getAttachedNavMesh();
	dtAssert(nav);

	dtStatus status = nav->getOffMeshConnectionPolyEndPoints(refs[0], refs[1], startPos, endPos);
	if (dtStatusSucceed(status))
	{
		dtVcopy(m_pos, endPos);
		return true;
	}

	return false;
}

void dtPathCorridor::movePosition(const float* npos, dtNavMeshQuery* navquery, const dtQueryFilter* filter)
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
	m_npath = dtMergeCorridorStartMoved(m_path, m_npath, m_maxPath, visited, nvisited);
	
	// Adjust the position to stay on top of the navmesh.
	float h = m_pos[1];
	navquery->getPolyHeight(m_path[0], result, &h);
	result[1] = h;
	dtVcopy(m_pos, result);
}

void dtPathCorridor::moveTargetPosition(const float* npos, dtNavMeshQuery* navquery, const dtQueryFilter* filter)
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
	m_npath = dtMergeCorridorEndMoved(m_path, m_npath, m_maxPath, visited, nvisited);
	
	// TODO: should we do that?
	// Adjust the position to stay on top of the navmesh.
	/*	float h = m_target[1];
	 navquery->getPolyHeight(m_path[m_npath-1], result, &h);
	 result[1] = h;*/
	
	dtVcopy(m_target, result);
}

void dtPathCorridor::setCorridor(const float* target, const dtPolyRef* path, const int npath)
{
	dtAssert(m_path);
	dtAssert(npath > 0);
	dtAssert(npath < m_maxPath);
	
	dtVcopy(m_target, target);
	memcpy(m_path, path, sizeof(dtPolyRef)*npath);
	m_npath = npath;
}
