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
#include "DetourCommon.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


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



static void normalizeArray(float* arr, const int n)
{
	// Normalize penaly range.
	float minPen = FLT_MAX;
	float maxPen = -FLT_MAX;
	for (int i = 0; i < n; ++i)
	{
		minPen = dtMin(minPen, arr[i]);
		maxPen = dtMax(maxPen, arr[i]);
	}
	const float penRange = maxPen-minPen;
	const float s = penRange > 0.001f ? (1.0f / penRange) : 1;
	for (int i = 0; i < n; ++i)
		arr[i] = dtClamp((arr[i]-minPen)*s, 0.0f, 1.0f);
}

void normalizeSamples(RVO* rvo)
{
	normalizeArray(rvo->spen, rvo->ns);
	normalizeArray(rvo->svpen, rvo->ns);
	normalizeArray(rvo->svcpen, rvo->ns);
	normalizeArray(rvo->sspen, rvo->ns);
	normalizeArray(rvo->stpen, rvo->ns);
}


void setDynCircleBody(Body* b, const float* pos, const float rad, const float* vel, const float* dvel)
{
	b->type = BODY_CIRCLE;
	dtVcopy(b->p, pos);
	dtVcopy(b->vel, vel);
	dtVcopy(b->dvel, dvel);
	b->rad = rad;
}

void setStatCircleBody(Body* b, const float* pos, const float rad)
{
	b->type = BODY_CIRCLE;
	dtVcopy(b->p, pos);
	dtVset(b->vel, 0,0,0);
	dtVset(b->dvel, 0,0,0);
	b->rad = rad;
}

void setStatCapsuleBody(Body* b, const float* p, const float* q, const float rad)
{
	b->type = BODY_CAPSULE;
	dtVcopy(b->p, p);
	dtVcopy(b->q, q);
	dtVset(b->vel, 0,0,0);
	dtVset(b->dvel, 0,0,0);
	b->rad = rad;
}

static const float VEL_WEIGHT = 2.0f;
static const float CUR_VEL_WEIGHT = 0.75f;
static const float SIDE_WEIGHT = 0.75f;
static const float TOI_WEIGHT = 2.5f;

static int sweepCircleCircle(const float* c0, const float r0, const float* v,
							 const float* c1, const float r1,
							 float& tmin, float& tmax)
{
	static const float EPS = 0.0001f;
	float s[3];
	dtVsub(s,c1,c0);
	float r = r0+r1;
	float c = dtVdot2D(s,s) - r*r;
	float a = dtVdot2D(v,v);
	if (a < EPS) return 0;	// not moving
	
	// Overlap, calc time to exit.
	float b = dtVdot2D(v,s);
	float d = b*b - a*c;
	if (d < 0.0f) return 0; // no intersection.
	tmin = (b - dtSqrt(d)) / a;
	tmax = (b + dtSqrt(d)) / a;
	return 1;
}

static int sweepCircleSegment(const float* c0, const float r0, const float* v,
							  const float* sa, const float* sb, const float sr,
							  float& tmin, float &tmax)
{
	// equation parameters
	float L[3], H[3];
	dtVsub(L, sb, sa);
	dtVsub(H, c0, sa);
	const float radius = r0+sr;
	const float l2 = dtVdot2D(L, L);
	const float r2 = radius * radius;
	const float dl = dtVperp2D(v, L);
	const float hl = dtVperp2D(H, L);
	const float a = dl * dl;
	const float b = 2.0f * hl * dl;
	const float c = hl * hl - (r2 * l2);
	float d = (b*b) - (4.0f * a * c);
	
	// infinite line missed by infinite ray.
	if (d < 0.0f)
		return 0;
	
	const float i2a = 1.0f/(2*a);
	d = dtSqrt(d);
	tmin = (-b - d) * i2a;
	tmax = (-b + d) * i2a;
	
	// line missed by ray range.
	/*	if (tmax < 0.0f || tmin > 1.0f)
	 return 0;*/
	
	// find what part of the ray was collided.
	const float il2 = 1.0f / l2;
	float Pedge[3];
	dtVmad(Pedge, c0, v, tmin);
	dtVsub(H, Pedge, sa);
	const float e0 = dtVdot2D(H, L) * il2;
	dtVmad(Pedge, c0, v, tmax);
	dtVsub(H, Pedge, sa);
	const float e1 = dtVdot2D(H, L) * il2;
	
	if (e0 < 0.0f || e1 < 0.0f)
	{
		float ctmin, ctmax;
		if (sweepCircleCircle(c0, r0, v, sa, sr, ctmin, ctmax))
		{
			if (e0 < 0.0f && ctmin > tmin)
				tmin = ctmin;
			if (e1 < 0.0f && ctmax < tmax)
				tmax = ctmax;
		}
		else
		{
			return 0;
		}
	}
	
	if (e0 > 1.0f || e1 > 1.0f)
	{
		float ctmin, ctmax;
		if (sweepCircleCircle(c0, r0, v, sb, sr, ctmin, ctmax))
		{
			if (e0 > 1.0f && ctmin > tmin)
				tmin = ctmin;
			if (e1 > 1.0f && ctmax < tmax)
				tmax = ctmax;
		}
		else
		{
			return 0;
		}
	}
	
	return 1;
}

static void processSamples(Body* agent, const float vmax,
						   const Body* obs, const int nobs, RVO* rvo,
						   const float* spos, const float cs, const int nspos,
						   float* res)
{
	dtVset(res, 0,0,0);
	
	const float ivmax = 1.0f / vmax;
	
	// Max time of collision to be considered.
	const float maxToi = 2.5f;
	
	float minPenalty = FLT_MAX;
	
	for (int n = 0; n < nspos; ++n)
	{
		float vcand[3];
		dtVcopy(vcand, &spos[n*3]);		
		dtVcopy(&rvo->spos[rvo->ns*3], &spos[n*3]);
		rvo->scs[rvo->ns] = cs;
		
		// Find min time of impact and exit amongst all obstacles.
		float tmin = maxToi;
		float side = 0;
		int nside = 0;
		
		for (int i = 0; i < nobs; ++i)
		{
			const Body* ob = &obs[i];
			float htmin = 0, htmax = 0;
			
			if (ob->type == BODY_CIRCLE)
			{
				float vab[3];
				
				// Moving, use RVO
				dtVscale(vab, vcand, 2);
				dtVsub(vab, vab, agent->vel);
				dtVsub(vab, vab, ob->vel);
				
				// Side
				// NOTE: dp, and dv are constant over the whole calculation,
				// they can be precomputed per object. 
				const float* pa = agent->p;
				const float* pb = ob->p;
				
				const float orig[3] = {0,0};
				float dp[3],dv[3],np[3];
				dtVsub(dp,pb,pa);
				dtVnormalize(dp);
				dtVsub(dv, ob->dvel, agent->dvel);
				
				const float a = dtTriArea2D(orig, dp,dv);
				if (a < 0.01f)
				{
					np[0] = -dp[2];
					np[2] = dp[0];
				}
				else
				{
					np[0] = dp[2];
					np[2] = -dp[0];
				}
				
				side += dtClamp(dtMin(dtVdot2D(dp,vab)*2,dtVdot2D(np,vab)*2), 0.0f, 1.0f);
				nside++;
				
				if (!sweepCircleCircle(agent->p,agent->rad, vab, ob->p,ob->rad, htmin, htmax))
					continue;
				
				// Handle overlapping obstacles.
				if (htmin < 0.0f && htmax > 0.0f)
				{
					// Avoid more when overlapped.
					htmin = -htmin * 0.5f;
				}
			}
			else if (ob->type == BODY_CAPSULE)
			{
				// NOTE: the segments are assumed to come from a navmesh which is shrunken by
				// the agent radius, hence the use of really small radius.
				// This can be handle more efficiently by using seg-seg test instead.
				// If the whole segment is to be treated as obstacle, use agent->rad instead of 0.01f!
				const float r = 0.01f; // agent->rad

				float t;
				if (dtDistancePtSegSqr2D(agent->p, ob->p, ob->q, t) < dtSqr(r+ob->rad))
				{
					float sdir[3], snorm[3];
					dtVsub(sdir, ob->q, ob->p);
					snorm[0] = -sdir[2];
					snorm[2] = sdir[0];
					// If the velocity is pointing towards the segment, no collision.
					if (dtVdot2D(snorm, vcand) < 0.0f)
						continue;
					// Else immediate collision.
					htmin = 0.0f;
					htmax = 10.0f;
				}
				else
				{
					if (!sweepCircleSegment(agent->p, r, vcand, ob->p, ob->q, ob->rad, htmin, htmax))
						continue;
				}
				
				// Avoid less when facing walls.
				htmin *= 2.0f;
			}
			
			if (htmin >= 0.0f)
			{
				// The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
				if (htmin < tmin)
					tmin = htmin;
			}
		}
		
		// Normalize side bias, to prevent it dominating too much.
		if (nside)
			side /= nside;
		
		const float vpen = VEL_WEIGHT * (dtVdist2D(vcand, agent->dvel) * ivmax);
		const float vcpen = CUR_VEL_WEIGHT * (dtVdist2D(vcand, agent->vel) * ivmax);
		const float spen = SIDE_WEIGHT * side;
		const float tpen = TOI_WEIGHT * (1.0f/(0.1f+tmin/maxToi));
		
		const float penalty = vpen + vcpen + spen + tpen;
		
		if (penalty < minPenalty)
		{
			minPenalty = penalty;
			dtVcopy(res, vcand);
		}
		
		// Store different penalties for debug viewing
		rvo->spen[rvo->ns] = penalty;
		rvo->svpen[rvo->ns] = vpen;
		rvo->svcpen[rvo->ns] = vcpen;
		rvo->sspen[rvo->ns] = spen;
		rvo->stpen[rvo->ns] = tpen;
		
		rvo->ns++;
	}
}


void sampleRVO(Body* agent, const float vmax, const Body* obs, const int nobs, RVO* rvo, const float bias, float* nvel)
{
	dtVset(nvel, 0,0,0);
	
	float spos[MAX_RVO_SAMPLES*3];
	int nspos = 0;
	
	const float cvx = agent->dvel[0]*bias;
	const float cvz = agent->dvel[2]*bias;
	const float vrange = vmax*(1-bias);
	const float cs = 1.0f / (float)RVO_SAMPLE_RAD*vrange;
	
	for (int z = -RVO_SAMPLE_RAD; z <= RVO_SAMPLE_RAD; ++z)
	{
		for (int x = -RVO_SAMPLE_RAD; x <= RVO_SAMPLE_RAD; ++x)
		{
			if (nspos < MAX_RVO_SAMPLES)
			{
				const float vx = cvx + (float)(x+0.5f)*cs;
				const float vz = cvz + (float)(z+0.5f)*cs;
				if (dtSqr(vx)+dtSqr(vz) > dtSqr(vmax+cs/2)) continue;
				spos[nspos*3+0] = vx;
				spos[nspos*3+1] = 0;
				spos[nspos*3+2] = vz;
				nspos++;
			}
		}
	}
	
	rvo->ns = 0;
	
	processSamples(agent, vmax, obs, nobs, rvo, spos, cs/2, nspos, nvel);
}



CrowdManager::CrowdManager() :
	m_shortcutIter(0)
{
	reset();
}

CrowdManager::~CrowdManager()
{
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
			
			static const int MAX_COL_POLYS = 32;
			dtPolyRef polys[MAX_COL_POLYS];
			const int npolys = navquery->findLocalNeighbourhood(ag->path[0], ag->pos, ag->colradius, &filter, polys, 0, MAX_COL_POLYS);

			ag->ncolsegs = 0;
			for (int j = 0; j < npolys; ++j)
			{
				float segs[DT_VERTS_PER_POLYGON*3*2];
				const int nsegs = navquery->getPolyWallSegments(polys[j], &filter, segs);
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
			// No corner to steer to, 
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
	static const int MAX_BODIES = 32;
	Body bodies[MAX_BODIES];
	for (int i = 0; i < MAX_AGENTS; ++i)
	{
		if (!m_agents[i].active) continue;
		if (m_agents[i].targetState != AGENT_TARGET_PATH) continue;
		Agent* ag = &m_agents[i];

		if (flags & CROWDMAN_USE_VO)
		{
			int nbodies = 0;

			// Add dynamic obstacles.
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
				if (dist > dtSqr(ag->colradius))
					continue;
				
				if (nbodies < MAX_BODIES)
				{
					setDynCircleBody(&bodies[nbodies], nei->pos, nei->radius, nei->vel, nei->dvel);
					nbodies++;
				}
			}
			
			// Add static obstacles.
			for (int j = 0; j < ag->ncolsegs; ++j)
			{
				const float* s = &ag->colsegs[j*6];
				if (dtTriArea2D(ag->pos, s, s+3) < 0.0f)
					continue;

				if (nbodies < MAX_BODIES)
				{
					setStatCapsuleBody(&bodies[nbodies], s,s+3,0);
					nbodies++;
				}
			}
				
			Body agent;
			setDynCircleBody(&agent, ag->pos, ag->radius, ag->vel, ag->dvel);

			sampleRVO(&agent, ag->maxspeed, bodies, nbodies, &ag->rvo, 0.4f, ag->nvel);
			
			// Normalize samples for debug draw
			normalizeSamples(&ag->rvo);
		}
		else
		{
			dtVcopy(ag->nvel, ag->dvel);
		}
	}
		
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

			ag->disp[0] = ag->disp[1] = ag->disp[2] = 0;

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
		if (ag->npath && ag->ncorners > 1)
		{
			// The target is the corner after the next corner to steer to.
			const float* tgt = &ag->corners[3];
			const float distSqr = dtVdist2DSqr(ag->pos, tgt);
			if (distSqr > dtSqr(0.01f)) // && distSqr < dtSqr(20.0f))
			{
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
	m_expandOptions(true),
	m_anticipateTurns(true),
	m_useVO(true),
	m_drunkMove(false),
	m_run(true),
	m_mode(TOOLMODE_CREATE)
{
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
	
	if (imguiCollapse("Options", m_expandOptions))
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

	if (imguiCollapse("Debug Draw", m_expandDebugDraw))
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
			// Add
			int idx = m_crowd.addAgent(p, m_sample->getAgentRadius(), m_sample->getAgentHeight());
			if (idx != -1 && m_targetPosSet)
				m_crowd.setMoveTarget(idx, m_targetPos);
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

		if (m_showVO)
		{
			// Draw detail about agent sela
			const RVO* rvo = &ag->rvo;

			const float dx = ag->pos[0];
			const float dy = ag->pos[1]+ag->height;
			const float dz = ag->pos[2];
			
			dd.begin(DU_DRAW_QUADS);
			for (int i = 0; i < rvo->ns; ++i)
			{
				const float* p = &rvo->spos[i*3];
				const float sr = rvo->scs[i];
				unsigned int col = duLerpCol(duRGBA(255,255,255,220), duRGBA(0,96,128,220), (int)(rvo->spen[i]*255));
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
}
