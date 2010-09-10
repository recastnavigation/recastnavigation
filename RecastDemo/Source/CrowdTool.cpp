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
#include <float.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "CrowdTool.h"
#include "InputGeom.h"
#include "Sample.h"
#include "DetourDebugDraw.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "SampleInterfaces.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

static const int VO_ADAPTIVE_GRID_SIZE = 4;
static const int VO_ADAPTIVE_GRID_DEPTH = 5;
static const int VO_GRID_SIZE = 33;


static bool isectSegAABB(const float* sp, const float* sq,
						 const float* amin, const float* amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	dtVsub(d, sq, sp);
	tmin = 0;  // set to -FLT_MAX to get first hit on line
	tmax = FLT_MAX;		// set to max distance ray can travel (for segment)
	
	// For all three slabs
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			// Make t1 be intersection with near plane, t2 with far plane
			if (t1 > t2) dtSwap(t1, t2);
			// Compute the intersection of slab intersections intervals
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax) return false;
		}
	}
	
	return true;
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



static void getAgentBounds(const Agent* ag, float* bmin, float* bmax)
{
	bmin[0] = ag->pos[0] - ag->radius;
	bmin[1] = ag->pos[1];
	bmin[2] = ag->pos[2] - ag->radius;
	bmax[0] = ag->pos[0] + ag->radius;
	bmax[1] = ag->pos[1] + ag->height;
	bmax[2] = ag->pos[2] + ag->radius;
}


CrowdManager::CrowdManager() :
	m_obstacleQuery(0)
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
}

void CrowdManager::update(const float dt, unsigned int flags, dtNavMeshQuery* navquery)
{
	if (!navquery)
		return;
	
	TimeVal startTime = getPerfTime();
	
	const float ext[3] = {2,4,2};
	dtQueryFilter filter;
		
	// Update target and agent navigation state.
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		if (!m_agents[i].active) continue;
		Agent* ag = &m_agents[i];
		
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

		if (ag->npath && dtVdist2DSqr(ag->pos, ag->colcenter) > dtSqr(ag->colradius*0.25f))
		{
			dtVcopy(ag->colcenter, ag->pos);
			static const int MAX_LOCALS = 32;
			dtPolyRef locals[MAX_LOCALS];

			const int nlocals = navquery->findLocalNeighbourhood(ag->path[0], ag->pos, ag->colradius, &filter, locals, 0, MAX_LOCALS);

			ag->ncolsegs = 0;
			for (int j = 0; j < nlocals; ++j)
			{
				float segs[DT_VERTS_PER_POLYGON*3*2];
				const int nsegs = navquery->getPolyWallSegments(locals[j], &filter, segs);
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
	}
	
	static const float MAX_ACC = 8.0f;
	static const float MAX_SPEED = 3.5f;

	static const float MIN_TARGET_DIST = 0.01f;

	// Calculate steering.
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		if (!m_agents[i].active) continue;
		if (m_agents[i].targetState != AGENT_TARGET_PATH) continue;
		Agent* ag = &m_agents[i];

		if (flags & CROWDMAN_DRUNK)
		{
			ag->t += dt * (1.0f - ag->var*0.25f);
			ag->maxspeed = MAX_SPEED*(1 + dtSqr(cosf(ag->t*2.0f))*0.3f);
		}
		else
		{
			ag->maxspeed = MAX_SPEED;
		}

		unsigned char cornerFlags[AGENT_MAX_CORNERS];
		dtPolyRef cornerPolys[AGENT_MAX_CORNERS];
		ag->ncorners = navquery->findStraightPath(ag->pos, ag->target, ag->path, ag->npath,
											   ag->corners, cornerFlags, cornerPolys, AGENT_MAX_CORNERS);

		// Prune points in the beginning of the path which are too close.
		while (ag->ncorners)
		{
			if ((cornerFlags[0] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
				dtVdist2DSqr(&ag->corners[0], ag->pos) > dtSqr(MIN_TARGET_DIST))
				break;
			ag->ncorners--;
			if (ag->ncorners)
			{
				memmove(cornerFlags, cornerFlags+1, sizeof(unsigned char)*ag->ncorners);
				memmove(cornerPolys, cornerPolys+1, sizeof(dtPolyRef)*ag->ncorners);
				memmove(ag->corners, ag->corners+3, sizeof(float)*3*ag->ncorners);
			}
		}

		// Prune points after an off-mesh connection.
		for (int i = 0; i < ag->ncorners; ++i)
		{
			if (cornerFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
			{
				ag->ncorners = i+1;
				break;
			}
		}

		if (!ag->ncorners)
		{
			// No corner to steer to, stop.
			dtVset(ag->dvel, 0,0,0);
		}
		else
		{
			// Calculate delta movement.
			
			if (flags & CROWDMAN_ANTICIPATE_TURNS)
			{
				calcSmoothSteerDirection(ag->pos, ag->corners, ag->ncorners, ag->dvel);
			}
			else
			{
				dtVsub(ag->dvel, &ag->corners[0], ag->pos);
				ag->dvel[1] = 0;
			}

			bool endOfPath = (cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
			bool offMeshConnection = (cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;

			// Limit desired velocity to max speed.
			const float slowDownRadius = ag->radius*2;
			float distToGoal = slowDownRadius;
			if (endOfPath || offMeshConnection)
				distToGoal = dtVdist2D(ag->pos, &ag->corners[(ag->ncorners-1)*3]);

			float clampedSpeed = ag->maxspeed * dtMin(1.0f, distToGoal / slowDownRadius);
			float speed = dtVlen(ag->dvel);
			if (speed > 0.0001f)
				clampedSpeed /= speed;
			dtVscale(ag->dvel, ag->dvel, clampedSpeed);

			if (flags & CROWDMAN_DRUNK)
			{
				const float amp = cosf(ag->var*13.69f+ag->t*3.123f) * 0.2f;
				const float nx = -ag->dvel[2];
				const float nz = ag->dvel[0];
				ag->dvel[0] += nx*amp;
				ag->dvel[2] += nz*amp;
			}
		}
	}
	
	// Velocity planning.
	TimeVal rvoStartTime = getPerfTime();

	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		if (!m_agents[i].active) continue;
		if (m_agents[i].targetState != AGENT_TARGET_PATH) continue;
		Agent* ag = &m_agents[i];
		
		if (flags & CROWDMAN_USE_VO)
		{
			m_obstacleQuery->reset();
			
			// Add dynamic obstacles.
			for (int j = 0; j < MAX_AGENTS; ++j)
			{
				if (i == j) continue;
				const int idx = j;
				
				if (!m_agents[idx].active) continue;
				Agent* nei = &m_agents[idx];
				
				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				if (fabsf(diff[1]) >= (ag->height+nei->height)/2.0f)
					continue;
				diff[1] = 0;
				
				const float distSqr = dtVlenSqr(diff);
				if (distSqr > dtSqr(ag->colradius))
					continue;
				
				m_obstacleQuery->addCircle(nei->pos, nei->radius, nei->vel, nei->dvel, distSqr);
			}
			
			// Add static obstacles.
			for (int j = 0; j < ag->ncolsegs; ++j)
			{
				const float* s = &ag->colsegs[j*6];
				if (dtTriArea2D(ag->pos, s, s+3) < 0.0f)
					continue;
					
				float tseg;
				const float distSqr = dtDistancePtSegSqr2D(ag->pos, s, s+3, tseg);

				m_obstacleQuery->addSegment(s, s+3, distSqr);
			}

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
	
	// Integrate and update perceived velocity.
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		if (!m_agents[i].active) continue;
		Agent* ag = &m_agents[i];
		
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
		for (int i = 0; i < MAX_AGENTS; ++i)
		{
			if (!m_agents[i].active) continue;
			Agent* ag = &m_agents[i];

			dtVset(ag->disp, 0,0,0);

			float w = 0;

			for (int j = 0; j < MAX_AGENTS; ++j)
			{
				if (i == j) continue;
				if (!m_agents[j].active) continue;
				Agent* nei = &m_agents[j];
				
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

		for (int i = 0; i < MAX_AGENTS; ++i)
		{
			if (!m_agents[i].active) continue;
			Agent* ag = &m_agents[i];
			dtVadd(ag->npos, ag->npos, ag->disp);
		}
	}
		
	// Move along navmesh and update new position.
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		if (!m_agents[i].active) continue;
		Agent* ag = &m_agents[i];
		
		float result[3];
		dtPolyRef visited[16];
		int nvisited = navquery->moveAlongSurface(ag->path[0], ag->pos, ag->npos, &filter,
												   result, visited, 16);
		ag->npath = fixupCorridor(ag->path, ag->npath, AGENT_MAX_PATH, visited, nvisited);

		float h = 0;
		navquery->getPolyHeight(ag->path[0], result, &h);
		result[1] = h;
		dtVcopy(ag->pos, result);
		
		ag->htrail = (ag->htrail + 1) % AGENT_MAX_TRAIL;
		dtVcopy(&ag->trail[ag->htrail*3], ag->pos);
	}

	// Optimize path
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		if (!m_agents[i].active) continue;
		Agent* ag = &m_agents[i];
		
		dtVset(ag->opts, 0,0,0);
		dtVset(ag->opte, 0,0,0);
		
		if (ag->npath && ag->ncorners > 1)
		{
			// The target is the corner after the next corner to steer to.
			float tgt[3];
			dtVcopy(tgt, &ag->corners[3]);
			const float distSqr = dtVdist2DSqr(ag->pos, tgt);
			if (distSqr > dtSqr(0.01f))
			{
				// Clamp teh ray to max distance.
				const float maxDist = ag->colradius*3;
				if (distSqr > dtSqr(maxDist))
				{
					float delta[3];
					dtVsub(delta, tgt, ag->pos);
					dtVmad(tgt, ag->pos, delta, dtSqr(maxDist)/distSqr);
				}
			
				dtVcopy(ag->opts, ag->pos);
				dtVcopy(ag->opte, tgt);

				static const int MAX_RES = 32;
				dtPolyRef res[MAX_RES];
				float t, norm[3];
				const int nres = navquery->raycast(ag->path[0], ag->pos, tgt, &filter, t, norm, res, MAX_RES);
				if (nres > 1 && t > 0.99f)
				{
					ag->npath = mergeCorridor(ag->path, ag->npath, AGENT_MAX_PATH, res, nres);
				}
			}
		}
	}
	


	TimeVal endTime = getPerfTime();

	int ns = 0;
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		if (!m_agents[i].active) continue;
		if (m_agents[i].targetState != AGENT_TARGET_PATH) continue;
		if (flags & CROWDMAN_USE_VO)
		{
			// Normalize samples for debug draw
			m_vodebug[i]->normalizeSamples();
			ns += m_vodebug[i]->getSampleCount();
		}
	}

	m_sampleCount.addSample((float)ns);
	m_totalTime.addSample(getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
	m_rvoTime.addSample(getPerfDeltaTimeUsec(rvoStartTime, rvoEndTime) / 1000.0f);
}

static int insertIsect(float u, int inside, Isect* ints, int nints)
{
	int i;
	if (nints >= FORM_MAX_ISECT) return nints;
	if (!nints || u >= ints[nints-1].u)
	{
		ints[nints].u = u;
		ints[nints].inside = inside;
		return nints+1;
	}
	for (i = 0; i < nints; ++i)
		if (u <= ints[i].u) break;
	if (nints-i > 0) memmove(ints+i+1,ints+i,sizeof(Isect)*(nints-i));
	ints[i].u = u;
	ints[i].inside = inside;
	return nints+1;
}

static int removeAdjacent(Isect* ints, int nints)
{
	const float eps = 0.0001f;
	if (nints < 2)
		return nints;
	for (int i = 0; i < nints-1; ++i)
	{
		if (fabsf(ints[i].u - ints[i+1].u) < eps) // && ints[i].inside != ints[i+1].inside)
		{
			nints -= 2;
			for (int j = i; j < nints; ++j)
				ints[j] = ints[j+2];
//			if (nints-i > 0) memmove(ints+i,ints+i+2,sizeof(Isect)*(nints-i));
			i--;
		}
	}
	return nints;
}

static int getPolyVerts(const dtNavMesh* navMesh, dtPolyRef ref, float* verts)
{
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (!navMesh->getTileAndPolyByRef(ref, &tile, &poly))
		return 0;
	for (int i = 0; i < (int)poly->vertCount; ++i)
		dtVcopy(&verts[i*3], &tile->verts[poly->verts[i]*3]);
	return poly->vertCount;
}

static void createFormation(Formation* form, const dtNavMesh* navmesh)
{
	float verts[DT_VERTS_PER_POLYGON*3];
	for (int i = 0; i < form->nsegs; i++)
	{
		FormationSeg* seg = &form->segs[i];
		seg->nints = 0;
		
		int startInside = 0;

		for (int j = 0; j < form->npolys; ++j)
		{
			const int nverts = getPolyVerts(navmesh, form->polys[j], verts);
			if (!nverts) continue;
			
			float tmin, tmax;
			int smin, smax;
			bool res = dtIntersectSegmentPoly2D(seg->p, seg->q, verts, nverts, tmin, tmax, smin, smax);

			if (!res)
				continue;
			
			if (tmin >= 0.0f && tmin <= 1.0f)
				seg->nints = insertIsect(tmin, 1, seg->ints, seg->nints);
			if (tmax >= 0.0f && tmax <= 1.0f)
				seg->nints = insertIsect(tmax, -1, seg->ints, seg->nints);
			if (tmin < 0.0f && tmax > 0.0f)
				startInside++;
		}
		
		seg->nints = removeAdjacent(seg->ints, seg->nints);
	}
	
	// Calc winding
	for (int i = 0; i < form->nsegs; ++i)
	{
		FormationSeg* seg = &form->segs[i];
		int inside = 0;
		for (int j = 0; j < seg->nints; ++j)
		{
			inside += seg->ints[j].inside;
			seg->ints[j].inside = inside;
		}
	}
}

CrowdTool::CrowdTool() :
	m_sample(0),
	m_targetPosSet(0),
	m_expandDebugDraw(false),
	m_showLabels(true),
	m_showCorners(false),
	m_showTargets(false),
	m_showCollisionSegments(false),
	m_showPath(false),
	m_showVO(false),
	m_showOpt(false),
	m_expandOptions(true),
	m_anticipateTurns(true),
	m_useVO(true),
	m_drunkMove(false),
	m_run(true),
	m_mode(TOOLMODE_CREATE)
{
	memset(&m_form, 0, sizeof(Formation));
}

CrowdTool::~CrowdTool()
{
}

void CrowdTool::init(Sample* sample)
{
	m_sample = sample;
}

void CrowdTool::reset()
{
	m_targetPosSet = false;
}

void CrowdTool::handleMenu()
{

	if (imguiCheck("Create Agents", m_mode == TOOLMODE_CREATE))
		m_mode = TOOLMODE_CREATE;
	if (imguiCheck("Move Agents", m_mode == TOOLMODE_MOVE))
		m_mode = TOOLMODE_MOVE;
	
	imguiSeparator();
	
	if (m_mode == TOOLMODE_CREATE)
	{
		imguiValue("Click to add agents.");
		imguiValue("Shift+Click to remove.");
	}
	else if (m_mode == TOOLMODE_MOVE)
	{
		imguiValue("Click to set move target.");
	}
	
	imguiSeparator();
	imguiSeparator();
	
	if (imguiCollapse("Options", 0, m_expandOptions))
		m_expandOptions = !m_expandOptions;
	
	if (m_expandOptions)
	{
		imguiIndent();
		if (imguiCheck("Anticipate Turns", m_anticipateTurns))
			m_anticipateTurns = !m_anticipateTurns;
		if (imguiCheck("Use VO", m_useVO))
			m_useVO = !m_useVO;
		if (imguiCheck("Drunk Move", m_drunkMove))
			m_drunkMove = !m_drunkMove;
		imguiUnindent();
	}

	if (imguiCollapse("Debug Draw", 0, m_expandDebugDraw))
		m_expandDebugDraw = !m_expandDebugDraw;
		
	if (m_expandDebugDraw)
	{
		imguiIndent();
		if (imguiCheck("Show Labels", m_showLabels))
			m_showLabels = !m_showLabels;
		if (imguiCheck("Show Corners", m_showCorners))
			m_showCorners = !m_showCorners;
		if (imguiCheck("Show Targets", m_showTargets))
			m_showTargets = !m_showTargets;
		if (imguiCheck("Show Collision Segs", m_showCollisionSegments))
			m_showCollisionSegments = !m_showCollisionSegments;
		if (imguiCheck("Show Path", m_showPath))
			m_showPath = !m_showPath;
		if (imguiCheck("Show VO", m_showVO))
			m_showVO = !m_showVO;
		if (imguiCheck("Show Opt", m_showOpt))
			m_showOpt = !m_showOpt;
		imguiUnindent();
	}
}

void CrowdTool::handleClick(const float* s, const float* p, bool shift)
{
	if (!m_sample) return;
	InputGeom* geom = m_sample->getInputGeom();
	if (!geom) return;

	if (m_mode == TOOLMODE_CREATE)
	{
		if (shift)
		{
			// Delete
			int isel = -1;
			float tsel = FLT_MAX;
			
			for (int i = 0; i < m_crowd.getAgentCount(); ++i)
			{
				const Agent* ag = m_crowd.getAgent(i);
				if (!ag->active) continue;
				float bmin[3], bmax[3];
				getAgentBounds(ag, bmin, bmax);
				float tmin, tmax;
				if (isectSegAABB(s, p, bmin,bmax, tmin, tmax))
				{
					if (tmin > 0 && tmin < tsel)
					{
						isel = i;
						tsel = tmin;
					} 
				}
			}
			if (isel != -1)
			{
				m_crowd.removeAgent(isel);
			}
		}
		else
		{
			bool single = false;
			
			if (single)
			{
				// Add
				int idx = m_crowd.addAgent(p, m_sample->getAgentRadius(), m_sample->getAgentHeight());
				if (idx != -1 && m_targetPosSet)
					m_crowd.setMoveTarget(idx, m_targetPos);
			}
			else
			{
				const dtNavMesh* navmesh = m_sample->getNavMesh();
				const dtNavMeshQuery* navquery = m_sample->getNavMeshQuery();

				memset(&m_form, 0, sizeof(Formation));

				const float ext[3] = {2,4,2};
				dtQueryFilter filter;

				const float r = m_sample->getAgentRadius();

				float nearest[3];
				dtPolyRef centerRef = navquery->findNearestPoly(p, ext, &filter, nearest);
				if (centerRef)
				{
					const int rows = 6;
					for (int i = 0; i < rows; ++i)
					{
						const float x0 = -r*2.5f*rows/2 + (i&1)*r;
						const float x1 = r*2.5f*rows/2 + (i&1)*r;
						const float z = (i-rows*0.5f)*r*2.5f;
						dtVset(m_form.segs[m_form.nsegs].p, p[0]+x0, p[1]+2.0f, p[2]+z);
						dtVset(m_form.segs[m_form.nsegs].q, p[0]+x1, p[1]+2.0f, p[2]+z);
						m_form.nsegs++;
					}
					
					m_form.npolys = navquery->findLocalNeighbourhood(centerRef, p, r*rows*2.5f, &filter, m_form.polys, 0, FORM_MAX_POLYS);
					
					createFormation(&m_form, navmesh);
					
					const int createCount = 25;
					int num = 0;
					
					const float r = m_sample->getAgentRadius();
					for (int i = 0; i < m_form.nsegs; ++i)
					{
						const FormationSeg* seg = &m_form.segs[i];
						for (int j = 0; j < seg->nints-1; ++j)
						{
							if (seg->ints[j].inside == 0) continue;
							const float u0 = seg->ints[j].u;
							const float u1 = seg->ints[j+1].u;
							float ia[3], ib[3];
							dtVlerp(ia, seg->p,seg->q, u0);
							dtVlerp(ib, seg->p,seg->q, u1);
							
							const float spacing = r*2.5f;
							float delta[3];
							dtVsub(delta, ib,ia);
							float d = dtVlen(delta);
							int np = (int)floorf(d/spacing);
							for (int k = 0; k < np; ++k)
							{
								float pos[3];
								dtVmad(pos, ia, delta, (float)(k+0.5f)/(float)np);

								if (num < createCount)
								{
									num++;
									int idx = m_crowd.addAgent(pos, m_sample->getAgentRadius(), m_sample->getAgentHeight());
									if (idx != -1 && m_targetPosSet)
										m_crowd.setMoveTarget(idx, m_targetPos);
								}
							}
						}
					}
				}
			}
			
		}
	}
	else if (m_mode == TOOLMODE_MOVE)
	{
		dtVcopy(m_targetPos, p);
		m_targetPosSet = true;

		for (int i = 0; i < m_crowd.getAgentCount(); ++i)
		{
			const Agent* ag = m_crowd.getAgent(i);
			if (!ag->active) continue;
			m_crowd.setMoveTarget(i, m_targetPos);
		}
	}
}

void CrowdTool::handleStep()
{
	m_run = !m_run;
}

void CrowdTool::handleUpdate(const float dt)
{
	if (!m_sample) return;
	if (!m_sample->getNavMesh()) return;
	if (m_run)
	{
		unsigned int flags = 0;

		if (m_anticipateTurns)
			flags |= CROWDMAN_ANTICIPATE_TURNS;
		if (m_useVO)
			flags |= CROWDMAN_USE_VO;
		if (m_drunkMove)
			flags |= CROWDMAN_DRUNK;
			
		m_crowd.update(dt, flags, m_sample->getNavMeshQuery());
	}
}

void CrowdTool::handleRender()
{
	DebugDrawGL dd;
	const float s = m_sample->getAgentRadius();
	
	dtNavMesh* nmesh = m_sample->getNavMesh();
	if (!nmesh)
		return;
	
	if (m_targetPosSet)
		duDebugDrawCross(&dd, m_targetPos[0],m_targetPos[1]+0.1f,m_targetPos[2], s, duRGBA(0,0,0,128), 2.0f);


	for (int i = 0; i < m_crowd.getAgentCount(); ++i)
	{
		const Agent* ag = m_crowd.getAgent(i);
		if (!ag->active) continue;

		dd.depthMask(false);
		
		if (m_showPath)
		{
			for (int i = 0; i < ag->npath; ++i)
				duDebugDrawNavMeshPoly(&dd, *nmesh, ag->path[i], duRGBA(0,0,0,64));
		}
		
		dd.begin(DU_DRAW_LINES,3.0f);
		float prev[3], preva = 1;
		dtVcopy(prev, ag->pos);
		for (int j = 0; j < AGENT_MAX_TRAIL-1; ++j)
		{
			const int idx = (ag->htrail + AGENT_MAX_TRAIL-j) % AGENT_MAX_TRAIL;
			const float* v = &ag->trail[idx*3];
			float a = 1 - j/(float)AGENT_MAX_TRAIL;
			dd.vertex(prev[0],prev[1]+0.1f,prev[2], duRGBA(0,0,0,(int)(128*preva)));
			dd.vertex(v[0],v[1]+0.1f,v[2], duRGBA(0,0,0,(int)(128*a)));
			preva = a;
			dtVcopy(prev, v);
		}
		dd.end();
		
		if (m_showTargets)
		{
			if (ag->targetState != AGENT_TARGET_NONE)
			{
				duDebugDrawArc(&dd, ag->pos[0], ag->pos[1], ag->pos[2],
									ag->target[0], ag->target[1], ag->target[2], 0.25f,
									0, 0.4f, duRGBA(0,0,0,128), 1.0f);
			}
		}

		if (m_showCorners)
		{
			if (ag->ncorners)
			{
				dd.begin(DU_DRAW_LINES, 2.0f);
				for (int j = 0; j < ag->ncorners; ++j)
				{
					const float* va = j == 0 ? ag->pos : &ag->corners[(j-1)*3];
					const float* vb = &ag->corners[j*3];
					dd.vertex(va[0],va[1]+ag->radius,va[2], duRGBA(128,0,0,64));
					dd.vertex(vb[0],vb[1]+ag->radius,vb[2], duRGBA(128,0,0,64));
				}
				dd.end();

				if (m_anticipateTurns)
				{
					float dvel[3], pos[3];
					calcSmoothSteerDirection(ag->pos, ag->corners, ag->ncorners, dvel);
					pos[0] = ag->pos[0] + dvel[0];
					pos[1] = ag->pos[1] + dvel[1];
					pos[2] = ag->pos[2] + dvel[2];
					
					const float off = ag->radius+0.1f;
					const float* tgt = &ag->corners[0];
					const float y = ag->pos[1]+off;
					
					dd.begin(DU_DRAW_LINES, 2.0f);
					
					dd.vertex(ag->pos[0],y,ag->pos[2], duRGBA(255,0,0,192));
					dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));

					dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));
					dd.vertex(tgt[0],y,tgt[2], duRGBA(255,0,0,192));

					dd.end();
				}
			}
		}
		
		if (m_showCollisionSegments)
		{
			const float off = ag->radius;
			duDebugDrawCross(&dd, ag->colcenter[0],ag->colcenter[1]+off,ag->colcenter[2], s, duRGBA(192,0,128,255), 2.0f);
			duDebugDrawCircle(&dd, ag->colcenter[0],ag->colcenter[1]+off,ag->colcenter[2], ag->colradius, duRGBA(192,0,128,128), 2.0f);

			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int j = 0; j < ag->ncolsegs; ++j)
			{
				const float* s = &ag->colsegs[j*6];
				unsigned int col = duRGBA(192,0,128,192);
				if (dtTriArea2D(ag->pos, s, s+3) < 0.0f)
					col = duDarkenCol(col);
				
//				dd.vertex(s[0],s[1]+0.2f,s[2], col);
//				dd.vertex(s[3],s[4]+0.2f,s[5], col);

				duAppendArrow(&dd, s[0],s[1]+0.2f,s[2], s[3],s[4]+0.2f,s[5], 0.0f, 0.3f, col);
			}
			dd.end();
		}

		if (m_showOpt)
		{
			dd.begin(DU_DRAW_LINES, 2.0f);
			dd.vertex(ag->opts[0],ag->opts[1]+0.3f,ag->opts[2], duRGBA(0,128,0,192));
			dd.vertex(ag->opte[0],ag->opte[1]+0.3f,ag->opte[2], duRGBA(0,128,0,192));
			dd.end();
		}

		if (m_showVO)
		{
			// Draw detail about agent sela
			const dtObstacleAvoidanceDebugData* debug = m_crowd.getVODebugData(i);

			const float dx = ag->pos[0];
			const float dy = ag->pos[1]+ag->height;
			const float dz = ag->pos[2];
			
			dd.begin(DU_DRAW_QUADS);
			for (int i = 0; i < debug->getSampleCount(); ++i)
			{
				const float* p = debug->getSampleVelocity(i);
				const float sr = debug->getSampleSize(i);
				const float pen = debug->getSamplePenalty(i);
				const float pen2 = debug->getSamplePreferredSidePenalty(i);
				unsigned int col = duLerpCol(duRGBA(255,255,255,220), duRGBA(0,96,128,220), (int)(pen*255));
				col = duLerpCol(col, duRGBA(128,0,0,220), (int)(pen2*128));
				dd.vertex(dx+p[0]-sr, dy, dz+p[2]-sr, col);
				dd.vertex(dx+p[0]-sr, dy, dz+p[2]+sr, col);
				dd.vertex(dx+p[0]+sr, dy, dz+p[2]+sr, col);
				dd.vertex(dx+p[0]+sr, dy, dz+p[2]-sr, col);
			}
			dd.end();
			
		}

		duDebugDrawArrow(&dd, ag->pos[0],ag->pos[1]+ag->height,ag->pos[2],
							  ag->pos[0]+ag->vel[0],ag->pos[1]+ag->height+ag->vel[1],ag->pos[2]+ag->vel[2],
							  0.0f, 0.4f, duRGBA(0,0,0,192), 2.0f);

		duDebugDrawArrow(&dd, ag->pos[0],ag->pos[1]+ag->height-0.1f,ag->pos[2],
						 ag->pos[0]+ag->dvel[0],ag->pos[1]+ag->height-0.1f+ag->dvel[1],ag->pos[2]+ag->dvel[2],
						 0.0f, 0.4f, duRGBA(0,192,255,192), 1.0f);
		
		duDebugDrawCylinderWire(&dd, ag->pos[0]-ag->radius, ag->pos[1]+ag->radius*0.1f, ag->pos[2]-ag->radius,
								ag->pos[0]+ag->radius, ag->pos[1]+ag->height, ag->pos[2]+ag->radius,
								duRGBA(0,192,255,255), 3.0f);
		
		dd.depthMask(true);
	}
	
	
/*
	for (int i = 0; i < m_form.npolys; ++i)
	{
		duDebugDrawNavMeshPoly(&dd, *nmesh, m_form.polys[i], duRGBA(255,255,255,32));
	}
	
	dd.depthMask(false);

	dd.begin(DU_DRAW_POINTS, 4.0f);
	for (int i = 0; i < m_form.nsegs; ++i)
	{
		const FormationSeg* seg = &m_form.segs[i];
		for (int j = 0; j < seg->nints-1; ++j)
		{
			if (seg->ints[j].inside == 0) continue;
			const float u0 = seg->ints[j].u;
			const float u1 = seg->ints[j+1].u;
			float ia[3], ib[3];
			dtVlerp(ia, seg->p,seg->q, u0);
			dtVlerp(ib, seg->p,seg->q, u1);
			dd.vertex(ia,duRGBA(128,0,0,192));
			dd.vertex(ib,duRGBA(128,0,0,192));
		}
	}
	dd.end();
	
	dd.begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < m_form.nsegs; ++i)
	{
		const FormationSeg* seg = &m_form.segs[i];
		dd.vertex(seg->p,duRGBA(255,255,255,128));
		dd.vertex(seg->q,duRGBA(255,255,255,128));
		for (int j = 0; j < seg->nints-1; ++j)
		{
			if (seg->ints[j].inside == 0) continue;
			const float u0 = seg->ints[j].u;
			const float u1 = seg->ints[j+1].u;
			float ia[3], ib[3];
			dtVlerp(ia, seg->p,seg->q, u0);
			dtVlerp(ib, seg->p,seg->q, u1);
			dd.vertex(ia,duRGBA(128,0,0,192));
			dd.vertex(ib,duRGBA(128,0,0,192));
		}
	}
	dd.end();

	{
		const float r = m_sample->getAgentRadius();
		dd.begin(DU_DRAW_LINES, 2.0f);
		for (int i = 0; i < m_form.nsegs; ++i)
		{
			const FormationSeg* seg = &m_form.segs[i];
			dd.vertex(seg->p,duRGBA(255,255,255,128));
			dd.vertex(seg->q,duRGBA(255,255,255,128));
			for (int j = 0; j < seg->nints-1; ++j)
			{
				if (seg->ints[j].inside == 0) continue;
				const float u0 = seg->ints[j].u;
				const float u1 = seg->ints[j+1].u;
				float ia[3], ib[3];
				dtVlerp(ia, seg->p,seg->q, u0);
				dtVlerp(ib, seg->p,seg->q, u1);

				const float spacing = r*2.5f;
				float delta[3];
				dtVsub(delta, ib,ia);
				float d = dtVlen(delta);
				int np = (int)floorf(d/spacing);
				for (int k = 0; k < np; ++k)
				{
					float pos[3];
					dtVmad(pos, ia, delta, (float)(k+0.5f)/(float)np);
					dd.vertex(pos[0],pos[1]-1,pos[2],duRGBA(128,0,0,192));
					dd.vertex(pos[0],pos[1]+2,pos[2],duRGBA(128,0,0,192));
				}
			}
		}
		dd.end();
	}
	
	dd.depthMask(true);
*/
	
}

void CrowdTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_targetPosSet && gluProject((GLdouble)m_targetPos[0], (GLdouble)m_targetPos[1], (GLdouble)m_targetPos[2],
								model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y+25), IMGUI_ALIGN_CENTER, "TARGET", imguiRGBA(0,0,0,220));
	}
	
	if (m_showLabels)
	{
		char label[32];
		for (int i = 0; i < m_crowd.getAgentCount(); ++i)
		{
			const Agent* ag = m_crowd.getAgent(i);
			if (!ag->active) continue;
			
			if (gluProject((GLdouble)ag->pos[0], (GLdouble)ag->pos[1]+ag->height, (GLdouble)ag->pos[2],
						   model, proj, view, &x, &y, &z))
			{
				snprintf(label, 32, "%d", i);
				imguiDrawText((int)x, (int)y+15, IMGUI_ALIGN_CENTER, label, imguiRGBA(0,0,0,220));
			}
			
		}
	}
	
	GraphParams gp;
	gp.setRect(300, 10, 500, 200, 8);
	gp.setValueRange(0.0f, 2.0f, 4, "ms");

	drawGraphBackground(&gp);
	drawGraph(&gp, m_crowd.getRVOTimeGraph(), 0, "RVO Sampling", duRGBA(255,0,128,255));
	drawGraph(&gp, m_crowd.getTotalTimeGraph(), 1, "Total", duRGBA(128,255,0,255));
	
	gp.setRect(300, 10, 500, 50, 8);
	gp.setValueRange(0.0f, 2000.0f, 1, "0");
	drawGraph(&gp, m_crowd.getSampleCountGraph(), 0, "Sample Count", duRGBA(255,255,255,255));
}
