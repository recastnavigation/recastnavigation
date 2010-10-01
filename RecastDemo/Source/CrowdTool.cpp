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
#include "CrowdManager.h"

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
	m_showGrid(false),
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
		if (imguiCheck("Show Path Optimization", m_showOpt))
			m_showOpt = !m_showOpt;
		if (imguiCheck("Show Prox Grid", m_showGrid))
			m_showGrid = !m_showGrid;
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
				ag->mover.getBounds(bmin, bmax);
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
			bool single = true;

			const dtNavMesh* navmesh = m_sample->getNavMesh();
			dtNavMeshQuery* navquery = m_sample->getNavMeshQuery();
			
			if (single)
			{
				// Add
				int idx = m_crowd.addAgent(p, m_sample->getAgentRadius(), m_sample->getAgentHeight());
				if (idx != -1 && m_targetPosSet)
					m_crowd.requestMoveTarget(idx, m_targetPos);
			}
			else
			{
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
										m_crowd.requestMoveTarget(idx, m_targetPos);
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
			m_crowd.requestMoveTarget(i, m_targetPos);
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

		m_crowdSampleCount.addSample((float)m_crowd.getSampleCount());
		m_crowdTotalTime.addSample(m_crowd.getTotalTime() / 1000.0f);
		m_crowdRvoTime.addSample(m_crowd.getRVOTime() / 1000.0f);
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
			for (int i = 0; i < ag->mover.m_npath; ++i)
				duDebugDrawNavMeshPoly(&dd, *nmesh, ag->mover.m_path[i], duRGBA(0,0,0,64));
		}
		
		dd.begin(DU_DRAW_LINES,3.0f);
		float prev[3], preva = 1;
		dtVcopy(prev, ag->mover.m_pos);
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
			duDebugDrawArc(&dd, ag->mover.m_pos[0], ag->mover.m_pos[1], ag->mover.m_pos[2],
								ag->mover.m_target[0], ag->mover.m_target[1], ag->mover.m_target[2], 0.25f,
								0, 0.4f, duRGBA(0,0,0,128), 1.0f);
		}

		if (m_showCorners)
		{
			if (ag->mover.m_ncorners)
			{
				dd.begin(DU_DRAW_LINES, 2.0f);
				for (int j = 0; j < ag->mover.m_ncorners; ++j)
				{
					const float* va = j == 0 ? ag->mover.m_pos : &ag->mover.m_cornerVerts[(j-1)*3];
					const float* vb = &ag->mover.m_cornerVerts[j*3];
					dd.vertex(va[0],va[1]+ag->mover.m_radius,va[2], duRGBA(128,0,0,64));
					dd.vertex(vb[0],vb[1]+ag->mover.m_radius,vb[2], duRGBA(128,0,0,64));
				}
				dd.end();

				if (m_anticipateTurns)
				{
/*					float dvel[3], pos[3];
					calcSmoothSteerDirection(ag->pos, ag->cornerVerts, ag->ncorners, dvel);
					pos[0] = ag->pos[0] + dvel[0];
					pos[1] = ag->pos[1] + dvel[1];
					pos[2] = ag->pos[2] + dvel[2];
					
					const float off = ag->radius+0.1f;
					const float* tgt = &ag->cornerVerts[0];
					const float y = ag->pos[1]+off;
					
					dd.begin(DU_DRAW_LINES, 2.0f);
					
					dd.vertex(ag->pos[0],y,ag->pos[2], duRGBA(255,0,0,192));
					dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));

					dd.vertex(pos[0],y,pos[2], duRGBA(255,0,0,192));
					dd.vertex(tgt[0],y,tgt[2], duRGBA(255,0,0,192));

					dd.end();*/
				}
			}
		}
		
		if (m_showCollisionSegments)
		{
			const float off = ag->mover.m_radius;
			duDebugDrawCross(&dd, ag->mover.m_localCenter[0],ag->mover.m_localCenter[1]+off,ag->mover.m_localCenter[2], s, duRGBA(192,0,128,255), 2.0f);
			duDebugDrawCircle(&dd, ag->mover.m_localCenter[0],ag->mover.m_localCenter[1]+off,ag->mover.m_localCenter[2], ag->mover.m_colradius, duRGBA(192,0,128,128), 2.0f);

			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int j = 0; j < ag->mover.m_localSegCount; ++j)
			{
				const float* s = &ag->mover.m_localSegs[j*6];
				unsigned int col = duRGBA(192,0,128,192);
				if (dtTriArea2D(ag->mover.m_pos, s, s+3) < 0.0f)
					col = duDarkenCol(col);
				
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

			const float dx = ag->mover.m_pos[0];
			const float dy = ag->mover.m_pos[1]+ag->mover.m_height;
			const float dz = ag->mover.m_pos[2];
			
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

		const float height = ag->mover.m_height;
		const float radius = ag->mover.m_radius;
		const float* pos = ag->mover.m_pos;
		const float* vel = ag->mover.m_vel;
		const float* dvel = ag->mover.m_dvel;

		
		duDebugDrawArrow(&dd, pos[0],pos[1]+height,pos[2],
							  pos[0]+vel[0],pos[1]+height+vel[1],pos[2]+vel[2],
							  0.0f, 0.4f, duRGBA(0,0,0,192), 2.0f);

		duDebugDrawArrow(&dd, pos[0],pos[1]+height-0.1f,pos[2],
						 pos[0]+dvel[0],pos[1]+height-0.1f+dvel[1],pos[2]+dvel[2],
						 0.0f, 0.4f, duRGBA(0,192,255,192), 1.0f);
		
		duDebugDrawCylinderWire(&dd, pos[0]-radius, pos[1]+radius*0.1f, pos[2]-radius,
								pos[0]+radius, pos[1]+height, pos[2]+radius,
								duRGBA(0,192,255,255), 3.0f);
		
		dd.depthMask(true);
	}
	
	if (m_showGrid)
	{
		float gridy = -FLT_MAX;
		for (int i = 0; i < m_crowd.getAgentCount(); ++i)
		{
			const Agent* ag = m_crowd.getAgent(i);
			if (!ag->active) continue;
			const float* pos = ag->mover.m_pos;
			gridy = dtMax(gridy, pos[1]);
		}
		gridy += 1.0f;
			
		dd.depthMask(false);
		dd.begin(DU_DRAW_QUADS);
		const ProximityGrid* grid = m_crowd.getGrid();
		const int* bounds = grid->getBounds();
		const float cs = grid->getCellSize();
		for (int y = bounds[1]; y <= bounds[3]; ++y)
		{
			for (int x = bounds[0]; x <= bounds[2]; ++x)
			{
				const int count = grid->getItemCountAt(x,y); 
				if (!count) continue;
				unsigned int col = duRGBA(128,0,0,dtMin(count*40,255));
				dd.vertex(x*cs, gridy, y*cs, col);
				dd.vertex(x*cs, gridy, y*cs+cs, col);
				dd.vertex(x*cs+cs, gridy, y*cs+cs, col);
				dd.vertex(x*cs+cs, gridy, y*cs, col);
			}
		}
		dd.end();
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
			const float* pos = ag->mover.m_pos;
			const float h = ag->mover.m_height;
			if (gluProject((GLdouble)pos[0], (GLdouble)pos[1]+h, (GLdouble)pos[2],
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
	drawGraph(&gp, &m_crowdRvoTime, 0, "RVO Sampling", duRGBA(255,0,128,255));
	drawGraph(&gp, &m_crowdTotalTime, 1, "Total", duRGBA(128,255,0,255));
	
	gp.setRect(300, 10, 500, 50, 8);
	gp.setValueRange(0.0f, 2000.0f, 1, "0");
	drawGraph(&gp, &m_crowdSampleCount, 0, "Sample Count", duRGBA(255,255,255,255));
}
