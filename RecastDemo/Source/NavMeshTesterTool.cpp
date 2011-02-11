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
#include <string.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "NavMeshTesterTool.h"
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "DetourCommon.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

// Uncomment this to dump all the requests in stdout.
#define DUMP_REQS

inline bool inRange(const float* v1, const float* v2, const float r, const float h)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return (dx*dx + dz*dz) < r*r && fabsf(dy) < h;
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
	const int orig = rcMin(furthestPath+1, npath);
	int size = rcMax(0, npath-orig);
	if (req+size > maxPath)
		size = maxPath-req;
	if (size)
		memmove(path+req, path+orig, size*sizeof(dtPolyRef));
	
	// Store visited
	for (int i = 0; i < req; ++i)
		path[i] = visited[(nvisited-1)-i];				
	
	return req+size;
}

static bool getSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos,
						   const float minTargetDist,
						   const dtPolyRef* path, const int pathSize,
						   float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef,
						   float* outPoints = 0, int* outPointCount = 0)							 
{
	// Find steer target.
	static const int MAX_STEER_POINTS = 3;
	float steerPath[MAX_STEER_POINTS*3];
	unsigned char steerPathFlags[MAX_STEER_POINTS];
	dtPolyRef steerPathPolys[MAX_STEER_POINTS];
	int nsteerPath = 0;
	navQuery->findStraightPath(startPos, endPos, path, pathSize,
							   steerPath, steerPathFlags, steerPathPolys, &nsteerPath, MAX_STEER_POINTS);
	if (!nsteerPath)
		return false;
		
	if (outPoints && outPointCount)
	{
		*outPointCount = nsteerPath;
		for (int i = 0; i < nsteerPath; ++i)
			dtVcopy(&outPoints[i*3], &steerPath[i*3]);
	}

	
	// Find vertex far enough to steer to.
	int ns = 0;
	while (ns < nsteerPath)
	{
		// Stop at Off-Mesh link or when point is further than slop away.
		if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			!inRange(&steerPath[ns*3], startPos, minTargetDist, 1000.0f))
			break;
		ns++;
	}
	// Failed to find good point to steer to.
	if (ns >= nsteerPath)
		return false;
	
	dtVcopy(steerPos, &steerPath[ns*3]);
	steerPos[1] = startPos[1];
	steerPosFlag = steerPathFlags[ns];
	steerPosRef = steerPathPolys[ns];
	
	return true;
}


NavMeshTesterTool::NavMeshTesterTool() :
	m_sample(0),
	m_navMesh(0),
	m_navQuery(0),
	m_pathFindStatus(DT_FAILURE),
	m_toolMode(TOOLMODE_PATHFIND_FOLLOW),
	m_startRef(0),
	m_endRef(0),
	m_npolys(0),
	m_nstraightPath(0),
	m_nsmoothPath(0),
	m_hitResult(false),
	m_distanceToWall(0),
	m_sposSet(false),
	m_eposSet(false),
	m_pathIterNum(0),
	m_steerPointCount(0)
{
	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL);
	m_filter.setExcludeFlags(0);

	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;
	
	m_neighbourhoodRadius = 2.5f;
}

NavMeshTesterTool::~NavMeshTesterTool()
{
}

void NavMeshTesterTool::init(Sample* sample)
{
	m_sample = sample;
	m_navMesh = sample->getNavMesh();
	m_navQuery = sample->getNavMeshQuery();
	recalc();

	if (m_navQuery)
	{
		// Change costs.
		m_filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_WATER, 10.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_ROAD, 1.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_DOOR, 1.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_GRASS, 2.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_JUMP, 1.5f);
	}
	
	m_neighbourhoodRadius = sample->getAgentRadius() * 20.0f;
}

void NavMeshTesterTool::handleMenu()
{
	if (imguiCheck("Pathfind Follow", m_toolMode == TOOLMODE_PATHFIND_FOLLOW))
	{
		m_toolMode = TOOLMODE_PATHFIND_FOLLOW;
		recalc();
	}
	if (imguiCheck("Pathfind Straight", m_toolMode == TOOLMODE_PATHFIND_STRAIGHT))
	{
		m_toolMode = TOOLMODE_PATHFIND_STRAIGHT;
		recalc();
	}
	if (imguiCheck("Pathfind Sliced", m_toolMode == TOOLMODE_PATHFIND_SLICED))
	{
		m_toolMode = TOOLMODE_PATHFIND_SLICED;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Distance to Wall", m_toolMode == TOOLMODE_DISTANCE_TO_WALL))
	{
		m_toolMode = TOOLMODE_DISTANCE_TO_WALL;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Raycast", m_toolMode == TOOLMODE_RAYCAST))
	{
		m_toolMode = TOOLMODE_RAYCAST;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Find Polys in Circle", m_toolMode == TOOLMODE_FIND_POLYS_IN_CIRCLE))
	{
		m_toolMode = TOOLMODE_FIND_POLYS_IN_CIRCLE;
		recalc();
	}
	if (imguiCheck("Find Polys in Shape", m_toolMode == TOOLMODE_FIND_POLYS_IN_SHAPE))
	{
		m_toolMode = TOOLMODE_FIND_POLYS_IN_SHAPE;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Find Local Neighbourhood", m_toolMode == TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD))
	{
		m_toolMode = TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD;
		recalc();
	}
	
	imguiSeparator();

	imguiLabel("Include Flags");

	imguiIndent();
	if (imguiCheck("Walk", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_WALK) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_WALK);
		recalc();
	}
	if (imguiCheck("Swim", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_SWIM) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_SWIM);
		recalc();
	}
	if (imguiCheck("Door", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_DOOR) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_DOOR);
		recalc();
	}
	if (imguiCheck("Jump", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_JUMP) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_JUMP);
		recalc();
	}
	imguiUnindent();

	imguiSeparator();
	imguiLabel("Exclude Flags");
	
	imguiIndent();
	if (imguiCheck("Walk", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_WALK) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_WALK);
		recalc();
	}
	if (imguiCheck("Swim", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_SWIM) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_SWIM);
		recalc();
	}
	if (imguiCheck("Door", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_DOOR) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_DOOR);
		recalc();
	}
	if (imguiCheck("Jump", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_JUMP) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_JUMP);
		recalc();
	}
	imguiUnindent();

	imguiSeparator();	
}

void NavMeshTesterTool::handleClick(const float* /*s*/, const float* p, bool shift)
{
	if (shift)
	{
		m_sposSet = true;
		dtVcopy(m_spos, p);
	}
	else
	{
		m_eposSet = true;
		dtVcopy(m_epos, p);
	}
	recalc();
}

void NavMeshTesterTool::handleStep()
{
}

void NavMeshTesterTool::handleToggle()
{
	// TODO: merge separate to a path iterator. Use same code in recalc() too.
	if (m_toolMode != TOOLMODE_PATHFIND_FOLLOW)
		return;
		
	if (!m_sposSet || !m_eposSet || !m_startRef || !m_endRef)
		return;
		
	static const float STEP_SIZE = 0.5f;
	static const float SLOP = 0.01f;

	if (m_pathIterNum == 0)
	{
		m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);
		m_nsmoothPath = 0;

		m_pathIterPolyCount = m_npolys;
		if (m_pathIterPolyCount)
			memcpy(m_pathIterPolys, m_polys, sizeof(dtPolyRef)*m_pathIterPolyCount); 
		
		if (m_pathIterPolyCount)
		{
			// Iterate over the path to find smooth path on the detail mesh surface.
			
			m_navQuery->closestPointOnPolyBoundary(m_startRef, m_spos, m_iterPos);
			m_navQuery->closestPointOnPolyBoundary(m_pathIterPolys[m_pathIterPolyCount-1], m_epos, m_targetPos);
			
			m_nsmoothPath = 0;
			
			dtVcopy(&m_smoothPath[m_nsmoothPath*3], m_iterPos);
			m_nsmoothPath++;
		}
	}
	
	dtVcopy(m_prevIterPos, m_iterPos);

	m_pathIterNum++;

	if (!m_pathIterPolyCount)
		return;

	if (m_nsmoothPath >= MAX_SMOOTH)
		return;

	// Move towards target a small advancement at a time until target reached or
	// when ran out of memory to store the path.

	// Find location to steer towards.
	float steerPos[3];
	unsigned char steerPosFlag;
	dtPolyRef steerPosRef;
		
	if (!getSteerTarget(m_navQuery, m_iterPos, m_targetPos, SLOP,
						m_pathIterPolys, m_pathIterPolyCount, steerPos, steerPosFlag, steerPosRef,
						m_steerPoints, &m_steerPointCount))
		return;
		
	dtVcopy(m_steerPos, steerPos);
	
	bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
	bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
		
	// Find movement delta.
	float delta[3], len;
	dtVsub(delta, steerPos, m_iterPos);
	len = sqrtf(dtVdot(delta,delta));
	// If the steer target is end of path or off-mesh link, do not move past the location.
	if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
		len = 1;
	else
		len = STEP_SIZE / len;
	float moveTgt[3];
	dtVmad(moveTgt, m_iterPos, delta, len);
		
	// Move
	float result[3];
	dtPolyRef visited[16];
	int nvisited = 0;
	m_navQuery->moveAlongSurface(m_pathIterPolys[0], m_iterPos, moveTgt, &m_filter,
								 result, visited, &nvisited, 16);
	m_pathIterPolyCount = fixupCorridor(m_pathIterPolys, m_pathIterPolyCount, MAX_POLYS, visited, nvisited);
	float h = 0;
	m_navQuery->getPolyHeight(m_pathIterPolys[0], result, &h);
	result[1] = h;
	dtVcopy(m_iterPos, result);
	
	// Handle end of path and off-mesh links when close enough.
	if (endOfPath && inRange(m_iterPos, steerPos, SLOP, 1.0f))
	{
		// Reached end of path.
		dtVcopy(m_iterPos, m_targetPos);
		if (m_nsmoothPath < MAX_SMOOTH)
		{
			dtVcopy(&m_smoothPath[m_nsmoothPath*3], m_iterPos);
			m_nsmoothPath++;
		}
		return;
	}
	else if (offMeshConnection && inRange(m_iterPos, steerPos, SLOP, 1.0f))
	{
		// Reached off-mesh connection.
		float startPos[3], endPos[3];
		
		// Advance the path up to and over the off-mesh connection.
		dtPolyRef prevRef = 0, polyRef = m_pathIterPolys[0];
		int npos = 0;
		while (npos < m_pathIterPolyCount && polyRef != steerPosRef)
		{
			prevRef = polyRef;
			polyRef = m_pathIterPolys[npos];
			npos++;
		}
		for (int i = npos; i < m_pathIterPolyCount; ++i)
			m_pathIterPolys[i-npos] = m_pathIterPolys[i];
		m_pathIterPolyCount -= npos;
				
		// Handle the connection.
		dtStatus status = m_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
		if (dtStatusSucceed(status))
		{
			if (m_nsmoothPath < MAX_SMOOTH)
			{
				dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
				m_nsmoothPath++;
				// Hack to make the dotted path not visible during off-mesh connection.
				if (m_nsmoothPath & 1)
				{
					dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
					m_nsmoothPath++;
				}
			}
			// Move position at the other side of the off-mesh link.
			dtVcopy(m_iterPos, endPos);
			float h;
			m_navQuery->getPolyHeight(m_pathIterPolys[0], m_iterPos, &h);
			m_iterPos[1] = h;
		}
	}
	
	// Store results.
	if (m_nsmoothPath < MAX_SMOOTH)
	{
		dtVcopy(&m_smoothPath[m_nsmoothPath*3], m_iterPos);
		m_nsmoothPath++;
	}

}

void NavMeshTesterTool::handleUpdate(const float /*dt*/)
{
	if (m_toolMode == TOOLMODE_PATHFIND_SLICED)
	{
		if (dtStatusInProgress(m_pathFindStatus))
		{
			m_pathFindStatus = m_navQuery->updateSlicedFindPath(1,0);
		}
		if (dtStatusSucceed(m_pathFindStatus))
		{
			m_navQuery->finalizeSlicedFindPath(m_polys, &m_npolys, MAX_POLYS);
			m_nstraightPath = 0;
			if (m_npolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				float epos[3];
				dtVcopy(epos, m_epos);
				if (m_polys[m_npolys-1] != m_endRef)
				m_navQuery->closestPointOnPoly(m_polys[m_npolys-1], m_epos, epos);

				m_navQuery->findStraightPath(m_spos, epos, m_polys, m_npolys,
											 m_straightPath, m_straightPathFlags,
											 m_straightPathPolys, &m_nstraightPath, MAX_POLYS);
			}
			 
			m_pathFindStatus = DT_FAILURE;
		}
	}
}

void NavMeshTesterTool::reset()
{
	m_startRef = 0;
	m_endRef = 0;
	m_npolys = 0;
	m_nstraightPath = 0;
	m_nsmoothPath = 0;
	memset(m_hitPos, 0, sizeof(m_hitPos));
	memset(m_hitNormal, 0, sizeof(m_hitNormal));
	m_distanceToWall = 0;
}


void NavMeshTesterTool::recalc()
{
	if (!m_navMesh)
		return;
	
	if (m_sposSet)
		m_navQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, 0);
	else
		m_startRef = 0;
	
	if (m_eposSet)
		m_navQuery->findNearestPoly(m_epos, m_polyPickExt, &m_filter, &m_endRef, 0);
	else
		m_endRef = 0;
	
	m_pathFindStatus = DT_FAILURE;
	
	if (m_toolMode == TOOLMODE_PATHFIND_FOLLOW)
	{
		m_pathIterNum = 0;
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			printf("pi  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif

			m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);

			m_nsmoothPath = 0;

			if (m_npolys)
			{
				// Iterate over the path to find smooth path on the detail mesh surface.
				dtPolyRef polys[MAX_POLYS];
				memcpy(polys, m_polys, sizeof(dtPolyRef)*m_npolys); 
				int npolys = m_npolys;
				
				float iterPos[3], targetPos[3];
				m_navQuery->closestPointOnPolyBoundary(m_startRef, m_spos, iterPos);
				m_navQuery->closestPointOnPolyBoundary(polys[npolys-1], m_epos, targetPos);
				
				static const float STEP_SIZE = 0.5f;
				static const float SLOP = 0.01f;
				
				m_nsmoothPath = 0;
				
				dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
				m_nsmoothPath++;
				
				// Move towards target a small advancement at a time until target reached or
				// when ran out of memory to store the path.
				while (npolys && m_nsmoothPath < MAX_SMOOTH)
				{
					// Find location to steer towards.
					float steerPos[3];
					unsigned char steerPosFlag;
					dtPolyRef steerPosRef;
					
					if (!getSteerTarget(m_navQuery, iterPos, targetPos, SLOP,
										polys, npolys, steerPos, steerPosFlag, steerPosRef))
						break;
					
					bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
					bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
					
					// Find movement delta.
					float delta[3], len;
					dtVsub(delta, steerPos, iterPos);
					len = dtSqrt(dtVdot(delta,delta));
					// If the steer target is end of path or off-mesh link, do not move past the location.
					if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
						len = 1;
					else
						len = STEP_SIZE / len;
					float moveTgt[3];
					dtVmad(moveTgt, iterPos, delta, len);
					
					// Move
					float result[3];
					dtPolyRef visited[16];
					int nvisited = 0;
					m_navQuery->moveAlongSurface(polys[0], iterPos, moveTgt, &m_filter,
												 result, visited, &nvisited, 16);
															   
					npolys = fixupCorridor(polys, npolys, MAX_POLYS, visited, nvisited);
					float h = 0;
					m_navQuery->getPolyHeight(polys[0], result, &h);
					result[1] = h;
					dtVcopy(iterPos, result);

					// Handle end of path and off-mesh links when close enough.
					if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached end of path.
						dtVcopy(iterPos, targetPos);
						if (m_nsmoothPath < MAX_SMOOTH)
						{
							dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
							m_nsmoothPath++;
						}
						break;
					}
					else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached off-mesh connection.
						float startPos[3], endPos[3];
						
						// Advance the path up to and over the off-mesh connection.
						dtPolyRef prevRef = 0, polyRef = polys[0];
						int npos = 0;
						while (npos < npolys && polyRef != steerPosRef)
						{
							prevRef = polyRef;
							polyRef = polys[npos];
							npos++;
						}
						for (int i = npos; i < npolys; ++i)
							polys[i-npos] = polys[i];
						npolys -= npos;
						
						// Handle the connection.
						dtStatus status = m_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
						if (dtStatusSucceed(status))
						{
							if (m_nsmoothPath < MAX_SMOOTH)
							{
								dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
								m_nsmoothPath++;
								// Hack to make the dotted path not visible during off-mesh connection.
								if (m_nsmoothPath & 1)
								{
									dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
									m_nsmoothPath++;
								}
							}
							// Move position at the other side of the off-mesh link.
							dtVcopy(iterPos, endPos);
							float h;
							m_navQuery->getPolyHeight(polys[0], iterPos, &h);
							iterPos[1] = h;
						}
					}
					
					// Store results.
					if (m_nsmoothPath < MAX_SMOOTH)
					{
						dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
						m_nsmoothPath++;
					}
				}
			}

		}
		else
		{
			m_npolys = 0;
			m_nsmoothPath = 0;
		}
	}
	else if (m_toolMode == TOOLMODE_PATHFIND_STRAIGHT)
	{
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
			m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);
			m_nstraightPath = 0;
			if (m_npolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				float epos[3];
				dtVcopy(epos, m_epos);
				if (m_polys[m_npolys-1] != m_endRef)
					m_navQuery->closestPointOnPoly(m_polys[m_npolys-1], m_epos, epos);
				
				m_navQuery->findStraightPath(m_spos, epos, m_polys, m_npolys,
											 m_straightPath, m_straightPathFlags,
											 m_straightPathPolys, &m_nstraightPath, MAX_POLYS);
			}
		}
		else
		{
			m_npolys = 0;
			m_nstraightPath = 0;
		}
	}
	else if (m_toolMode == TOOLMODE_PATHFIND_SLICED)
	{
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
			m_npolys = 0;
			m_nstraightPath = 0;
			
			m_pathFindStatus = m_navQuery->initSlicedFindPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter);
		}
		else
		{
			m_npolys = 0;
			m_nstraightPath = 0;
		}
	}
	else if (m_toolMode == TOOLMODE_RAYCAST)
	{
		m_nstraightPath = 0;
		if (m_sposSet && m_eposSet && m_startRef)
		{
#ifdef DUMP_REQS
			printf("rc  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
			float t = 0;
			m_npolys = 0;
			m_nstraightPath = 2;
			m_straightPath[0] = m_spos[0];
			m_straightPath[1] = m_spos[1];
			m_straightPath[2] = m_spos[2];
			m_navQuery->raycast(m_startRef, m_spos, m_epos, &m_filter, &t, m_hitNormal, m_polys, &m_npolys, MAX_POLYS);
			if (t > 1)
			{
				// No hit
				dtVcopy(m_hitPos, m_epos);
				m_hitResult = false;
			}
			else
			{
				// Hit
				m_hitPos[0] = m_spos[0] + (m_epos[0] - m_spos[0]) * t;
				m_hitPos[1] = m_spos[1] + (m_epos[1] - m_spos[1]) * t;
				m_hitPos[2] = m_spos[2] + (m_epos[2] - m_spos[2]) * t;
				if (m_npolys)
				{
					float h = 0;
					m_navQuery->getPolyHeight(m_polys[m_npolys-1], m_hitPos, &h);
					m_hitPos[1] = h;
				}
				m_hitResult = true;
			}
			dtVcopy(&m_straightPath[3], m_hitPos);
		}
	}
	else if (m_toolMode == TOOLMODE_DISTANCE_TO_WALL)
	{
		m_distanceToWall = 0;
		if (m_sposSet && m_startRef)
		{
#ifdef DUMP_REQS
			printf("dw  %f %f %f  %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], 100.0f,
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
			m_distanceToWall = 0.0f;
			m_navQuery->findDistanceToWall(m_startRef, m_spos, 100.0f, &m_filter, &m_distanceToWall, m_hitPos, m_hitNormal);
		}
	}
	else if (m_toolMode == TOOLMODE_FIND_POLYS_IN_CIRCLE)
	{
		if (m_sposSet && m_startRef && m_eposSet)
		{
			const float dx = m_epos[0] - m_spos[0];
			const float dz = m_epos[2] - m_spos[2];
			float dist = sqrtf(dx*dx + dz*dz);
#ifdef DUMP_REQS
			printf("fpc  %f %f %f  %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], dist,
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			m_navQuery->findPolysAroundCircle(m_startRef, m_spos, dist, &m_filter,
											  m_polys, m_parent, 0, &m_npolys, MAX_POLYS);

		}
	}
	else if (m_toolMode == TOOLMODE_FIND_POLYS_IN_SHAPE)
	{
		if (m_sposSet && m_startRef && m_eposSet)
		{
			const float nx = (m_epos[2] - m_spos[2])*0.25f;
			const float nz = -(m_epos[0] - m_spos[0])*0.25f;
			const float agentHeight = m_sample ? m_sample->getAgentHeight() : 0;

			m_queryPoly[0] = m_spos[0] + nx*1.2f;
			m_queryPoly[1] = m_spos[1] + agentHeight/2;
			m_queryPoly[2] = m_spos[2] + nz*1.2f;

			m_queryPoly[3] = m_spos[0] - nx*1.3f;
			m_queryPoly[4] = m_spos[1] + agentHeight/2;
			m_queryPoly[5] = m_spos[2] - nz*1.3f;

			m_queryPoly[6] = m_epos[0] - nx*0.8f;
			m_queryPoly[7] = m_epos[1] + agentHeight/2;
			m_queryPoly[8] = m_epos[2] - nz*0.8f;

			m_queryPoly[9] = m_epos[0] + nx;
			m_queryPoly[10] = m_epos[1] + agentHeight/2;
			m_queryPoly[11] = m_epos[2] + nz;
			
#ifdef DUMP_REQS
			printf("fpp  %f %f %f  %f %f %f  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_queryPoly[0],m_queryPoly[1],m_queryPoly[2],
				   m_queryPoly[3],m_queryPoly[4],m_queryPoly[5],
				   m_queryPoly[6],m_queryPoly[7],m_queryPoly[8],
				   m_queryPoly[9],m_queryPoly[10],m_queryPoly[11],
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			m_navQuery->findPolysAroundShape(m_startRef, m_queryPoly, 4, &m_filter,
											 m_polys, m_parent, 0, &m_npolys, MAX_POLYS);
		}
	}
	else if (m_toolMode == TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD)
	{
		if (m_sposSet && m_startRef)
		{
#ifdef DUMP_REQS
			printf("fln  %f %f %f  %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_neighbourhoodRadius,
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			m_navQuery->findLocalNeighbourhood(m_startRef, m_spos, m_neighbourhoodRadius, &m_filter,
											   m_polys, m_parent, &m_npolys, MAX_POLYS);
		}
	}
}

static void getPolyCenter(dtNavMesh* navMesh, dtPolyRef ref, float* center)
{
	center[0] = 0;
	center[1] = 0;
	center[2] = 0;
	
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	dtStatus status = navMesh->getTileAndPolyByRef(ref, &tile, &poly);
	if (dtStatusFailed(status))
		return;
		
	for (int i = 0; i < (int)poly->vertCount; ++i)
	{
		const float* v = &tile->verts[poly->verts[i]*3];
		center[0] += v[0];
		center[1] += v[1];
		center[2] += v[2];
	}
	const float s = 1.0f / poly->vertCount;
	center[0] *= s;
	center[1] *= s;
	center[2] *= s;
}



void NavMeshTesterTool::handleRender()
{
	DebugDrawGL dd;
	
	static const unsigned int startCol = duRGBA(128,25,0,192);
	static const unsigned int endCol = duRGBA(51,102,0,129);
	static const unsigned int pathCol = duRGBA(0,0,0,64);
	
	const float agentRadius = m_sample->getAgentRadius();
	const float agentHeight = m_sample->getAgentHeight();
	const float agentClimb = m_sample->getAgentClimb();
	
	dd.depthMask(false);
	if (m_sposSet)
		drawAgent(m_spos, agentRadius, agentHeight, agentClimb, startCol);
	if (m_eposSet)
		drawAgent(m_epos, agentRadius, agentHeight, agentClimb, endCol);
	dd.depthMask(true);
	
	if (!m_navMesh)
	{
		return;
	}

	if (m_toolMode == TOOLMODE_PATHFIND_FOLLOW)
	{
		duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);
		duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_endRef, endCol);
		
		if (m_npolys)
		{
			for (int i = 1; i < m_npolys-1; ++i)
				duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
		}
				
		if (m_nsmoothPath)
		{
			dd.depthMask(false);
			const unsigned int pathCol = duRGBA(0,0,0,220);
			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int i = 0; i < m_nsmoothPath; ++i)
				dd.vertex(m_smoothPath[i*3], m_smoothPath[i*3+1]+0.1f, m_smoothPath[i*3+2], pathCol);
			dd.end();
			dd.depthMask(true);
		}
		
		if (m_pathIterNum)
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_pathIterPolys[0], duRGBA(255,255,255,128));

			dd.depthMask(false);
			dd.begin(DU_DRAW_LINES, 1.0f);
			
			const unsigned int prevCol = duRGBA(255,192,0,220);
			const unsigned int curCol = duRGBA(255,255,255,220);
			const unsigned int steerCol = duRGBA(0,192,255,220);

			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]-0.3f,m_prevIterPos[2], prevCol);
			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], prevCol);

			dd.vertex(m_iterPos[0],m_iterPos[1]-0.3f,m_iterPos[2], curCol);
			dd.vertex(m_iterPos[0],m_iterPos[1]+0.3f,m_iterPos[2], curCol);

			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], prevCol);
			dd.vertex(m_iterPos[0],m_iterPos[1]+0.3f,m_iterPos[2], prevCol);

			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], steerCol);
			dd.vertex(m_steerPos[0],m_steerPos[1]+0.3f,m_steerPos[2], steerCol);
			
			for (int i = 0; i < m_steerPointCount-1; ++i)
			{
				dd.vertex(m_steerPoints[i*3+0],m_steerPoints[i*3+1]+0.2f,m_steerPoints[i*3+2], duDarkenCol(steerCol));
				dd.vertex(m_steerPoints[(i+1)*3+0],m_steerPoints[(i+1)*3+1]+0.2f,m_steerPoints[(i+1)*3+2], duDarkenCol(steerCol));
			}
			
			dd.end();
			dd.depthMask(true);
		}
	}
	else if (m_toolMode == TOOLMODE_PATHFIND_STRAIGHT || m_toolMode == TOOLMODE_PATHFIND_SLICED)
	{
		duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);
		duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_endRef, endCol);
		
		if (m_npolys)
		{
			for (int i = 1; i < m_npolys-1; ++i)
				duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
		}
		
		if (m_nstraightPath)
		{
			dd.depthMask(false);
			const unsigned int pathCol = duRGBA(64,16,0,220);
			const unsigned int offMeshCol = duRGBA(128,96,0,220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0; i < m_nstraightPath-1; ++i)
			{
				unsigned int col = 0;
				if (m_straightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
					col = offMeshCol;
				else
					col = pathCol;
				
				dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], col);
				dd.vertex(m_straightPath[(i+1)*3], m_straightPath[(i+1)*3+1]+0.4f, m_straightPath[(i+1)*3+2], col);
			}
			dd.end();
			dd.begin(DU_DRAW_POINTS, 6.0f);
			for (int i = 0; i < m_nstraightPath; ++i)
			{
				unsigned int col = 0;
				if (m_straightPathFlags[i] & DT_STRAIGHTPATH_START)
					col = startCol;
				else if (m_straightPathFlags[i] & DT_STRAIGHTPATH_START)
					col = endCol;
				else if (m_straightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
					col = offMeshCol;
				else
					col = pathCol;
				dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], pathCol);
			}
			dd.end();
			dd.depthMask(true);
		}
	}
	else if (m_toolMode == TOOLMODE_RAYCAST)
	{
		duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);
		
		if (m_nstraightPath)
		{
			for (int i = 1; i < m_npolys; ++i)
				duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
			
			dd.depthMask(false);
			const unsigned int pathCol = m_hitResult ? duRGBA(64,16,0,220) : duRGBA(240,240,240,220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0; i < m_nstraightPath-1; ++i)
			{
				dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], pathCol);
				dd.vertex(m_straightPath[(i+1)*3], m_straightPath[(i+1)*3+1]+0.4f, m_straightPath[(i+1)*3+2], pathCol);
			}
			dd.end();
			dd.begin(DU_DRAW_POINTS, 4.0f);
			for (int i = 0; i < m_nstraightPath; ++i)
				dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], pathCol);
			dd.end();

			if (m_hitResult)
			{
				const unsigned int hitCol = duRGBA(0,0,0,128);
				dd.begin(DU_DRAW_LINES, 2.0f);
				dd.vertex(m_hitPos[0], m_hitPos[1] + 0.4f, m_hitPos[2], hitCol);
				dd.vertex(m_hitPos[0] + m_hitNormal[0]*agentRadius,
						  m_hitPos[1] + 0.4f + m_hitNormal[1]*agentRadius,
						  m_hitPos[2] + m_hitNormal[2]*agentRadius, hitCol);
				dd.end();
			}
			dd.depthMask(true);
		}
	}
	else if (m_toolMode == TOOLMODE_DISTANCE_TO_WALL)
	{
		duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);
		dd.depthMask(false);
		duDebugDrawCircle(&dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], m_distanceToWall, duRGBA(64,16,0,220), 2.0f);
		dd.begin(DU_DRAW_LINES, 3.0f);
		dd.vertex(m_hitPos[0], m_hitPos[1] + 0.02f, m_hitPos[2], duRGBA(0,0,0,192));
		dd.vertex(m_hitPos[0], m_hitPos[1] + agentHeight, m_hitPos[2], duRGBA(0,0,0,192));
		dd.end();
		dd.depthMask(true);
	}
	else if (m_toolMode == TOOLMODE_FIND_POLYS_IN_CIRCLE)
	{
		for (int i = 0; i < m_npolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
			dd.depthMask(false);
			if (m_parent[i])
			{
				float p0[3], p1[3];
				dd.depthMask(false);
				getPolyCenter(m_navMesh, m_parent[i], p0);
				getPolyCenter(m_navMesh, m_polys[i], p1);
				duDebugDrawArc(&dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0,0,0,128), 2.0f);
				dd.depthMask(true);
			}
			dd.depthMask(true);
		}
		
		if (m_sposSet && m_eposSet)
		{
			dd.depthMask(false);
			const float dx = m_epos[0] - m_spos[0];
			const float dz = m_epos[2] - m_spos[2];
			const float dist = sqrtf(dx*dx + dz*dz);
			duDebugDrawCircle(&dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], dist, duRGBA(64,16,0,220), 2.0f);
			dd.depthMask(true);
		}
	}	
	else if (m_toolMode == TOOLMODE_FIND_POLYS_IN_SHAPE)
	{
		for (int i = 0; i < m_npolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
			dd.depthMask(false);
			if (m_parent[i])
			{
				float p0[3], p1[3];
				dd.depthMask(false);
				getPolyCenter(m_navMesh, m_parent[i], p0);
				getPolyCenter(m_navMesh, m_polys[i], p1);
				duDebugDrawArc(&dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0,0,0,128), 2.0f);
				dd.depthMask(true);
			}
			dd.depthMask(true);
		}
		
		if (m_sposSet && m_eposSet)
		{
			dd.depthMask(false);
			const unsigned int col = duRGBA(64,16,0,220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0, j = 3; i < 4; j=i++)
			{
				const float* p0 = &m_queryPoly[j*3];
				const float* p1 = &m_queryPoly[i*3];
				dd.vertex(p0, col);
				dd.vertex(p1, col);
			}
			dd.end();
			dd.depthMask(true);
		}
	}
	else if (m_toolMode == TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD)
	{
		for (int i = 0; i < m_npolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
			dd.depthMask(false);
			if (m_parent[i])
			{
				float p0[3], p1[3];
				dd.depthMask(false);
				getPolyCenter(m_navMesh, m_parent[i], p0);
				getPolyCenter(m_navMesh, m_polys[i], p1);
				duDebugDrawArc(&dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0,0,0,128), 2.0f);
				dd.depthMask(true);
			}

			static const int MAX_SEGS = DT_VERTS_PER_POLYGON*4;
			float segs[MAX_SEGS*6];
			dtPolyRef refs[MAX_SEGS];
			memset(refs, 0, sizeof(dtPolyRef)*MAX_SEGS); 
			int nsegs = 0;
			m_navQuery->getPolyWallSegments(m_polys[i], &m_filter, segs, refs, &nsegs, MAX_SEGS);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int j = 0; j < nsegs; ++j)
			{
				const float* s = &segs[j*6];
				
				// Skip too distant segments.
				float tseg;
				float distSqr = dtDistancePtSegSqr2D(m_spos, s, s+3, tseg);
				if (distSqr > dtSqr(m_neighbourhoodRadius))
					continue;
				
				float delta[3], norm[3], p0[3], p1[3];
				dtVsub(delta, s+3,s);
				dtVmad(p0, s, delta, 0.5f);
				norm[0] = delta[2];
				norm[1] = 0;
				norm[2] = -delta[0];
				dtVnormalize(norm);
				dtVmad(p1, p0, norm, agentRadius*0.5f);

				// Skip backfacing segments.
				if (refs[j])
				{
					unsigned int col = duRGBA(255,255,255,32);
					dd.vertex(s[0],s[1]+agentClimb,s[2],col);
					dd.vertex(s[3],s[4]+agentClimb,s[5],col);
				}
				else
				{
					unsigned int col = duRGBA(192,32,16,192);
					if (dtTriArea2D(m_spos, s, s+3) < 0.0f)
						col = duRGBA(96,32,16,192);
					
					dd.vertex(p0[0],p0[1]+agentClimb,p0[2],col);
					dd.vertex(p1[0],p1[1]+agentClimb,p1[2],col);

					dd.vertex(s[0],s[1]+agentClimb,s[2],col);
					dd.vertex(s[3],s[4]+agentClimb,s[5],col);
				}
			}
			dd.end();
			
			dd.depthMask(true);
		}
		
		if (m_sposSet)
		{
			dd.depthMask(false);
			duDebugDrawCircle(&dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], m_neighbourhoodRadius, duRGBA(64,16,0,220), 2.0f);
			dd.depthMask(true);
		}
	}	
}

void NavMeshTesterTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_sposSet && gluProject((GLdouble)m_spos[0], (GLdouble)m_spos[1], (GLdouble)m_spos[2],
								model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0,0,0,220));
	}
	if (m_eposSet && gluProject((GLdouble)m_epos[0], (GLdouble)m_epos[1], (GLdouble)m_epos[2],
								model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, "End", imguiRGBA(0,0,0,220));
	}
	
	// Tool help
	const int h = view[3];
	imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB+SHIFT: Set start location  LMB: Set end location", imguiRGBA(255,255,255,192));	
}

void NavMeshTesterTool::drawAgent(const float* pos, float r, float h, float c, const unsigned int col)
{
	DebugDrawGL dd;
	
	dd.depthMask(false);
	
	// Agent dimensions.	
	duDebugDrawCylinderWire(&dd, pos[0]-r, pos[1]+0.02f, pos[2]-r, pos[0]+r, pos[1]+h, pos[2]+r, col, 2.0f);

	duDebugDrawCircle(&dd, pos[0],pos[1]+c,pos[2],r,duRGBA(0,0,0,64),1.0f);

	unsigned int colb = duRGBA(0,0,0,196);
	dd.begin(DU_DRAW_LINES);
	dd.vertex(pos[0], pos[1]-c, pos[2], colb);
	dd.vertex(pos[0], pos[1]+c, pos[2], colb);
	dd.vertex(pos[0]-r/2, pos[1]+0.02f, pos[2], colb);
	dd.vertex(pos[0]+r/2, pos[1]+0.02f, pos[2], colb);
	dd.vertex(pos[0], pos[1]+0.02f, pos[2]-r/2, colb);
	dd.vertex(pos[0], pos[1]+0.02f, pos[2]+r/2, colb);
	dd.end();
	
	dd.depthMask(true);
}
