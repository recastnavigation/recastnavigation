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
#include "DetourCrowd.h"
#include "DetourDebugDraw.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "SampleInterfaces.h"

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

static void getAgentBounds(const dtCrowdAgent* ag, float* bmin, float* bmax)
{
	const float* p = ag->npos;
	const float r = ag->radius;
	const float h = ag->height;
	bmin[0] = p[0] - r;
	bmin[1] = p[1];
	bmin[2] = p[2] - r;
	bmax[0] = p[0] + r;
	bmax[1] = p[1] + h;
	bmax[2] = p[2] + r;
}

CrowdTool::CrowdTool() :
	m_sample(0),
	m_oldFlags(0),
	m_targetRef(0),
	m_expandSelectedDebugDraw(true),
	m_showCorners(false),
	m_showCollisionSegments(false),
	m_showPath(false),
	m_showVO(false),
	m_showOpt(false),
	m_expandDebugDraw(false),
	m_showLabels(false),
	m_showGrid(false),
	m_showNodes(false),
	m_showPerfGraph(false),

	m_expandOptions(true),
	m_anticipateTurns(true),
	m_optimizeVis(true),
	m_optimizeTopo(true),
	m_useVO(true),
	m_run(true),
	m_mode(TOOLMODE_CREATE)
{
	memset(m_trails, 0, sizeof(m_trails));
	
	m_vod = dtAllocObstacleAvoidanceDebugData();
	m_vod->init(2048);
	
	memset(&m_agentDebug, 0, sizeof(m_agentDebug));
	m_agentDebug.idx = -1;
	m_agentDebug.vod = m_vod;
}

CrowdTool::~CrowdTool()
{
	if (m_sample)
	{
		m_sample->setNavMeshDrawFlags(m_oldFlags);
	}
	
	dtFreeObstacleAvoidanceDebugData(m_vod);
}

void CrowdTool::init(Sample* sample)
{
	if (m_sample != sample)
	{
		m_sample = sample;
		m_oldFlags = m_sample->getNavMeshDrawFlags();
		m_sample->setNavMeshDrawFlags(m_oldFlags & ~DU_DRAWNAVMESH_CLOSEDLIST);
	}
		
	dtNavMesh* nav = m_sample->getNavMesh();
	if (nav)
		m_crowd.init(MAX_AGENTS, m_sample->getAgentRadius(), nav);

}

void CrowdTool::reset()
{
	m_targetRef = 0;
}

void CrowdTool::handleMenu()
{

	if (imguiCheck("Create Agents", m_mode == TOOLMODE_CREATE))
		m_mode = TOOLMODE_CREATE;
	if (imguiCheck("Move Target", m_mode == TOOLMODE_MOVE_TARGET))
		m_mode = TOOLMODE_MOVE_TARGET;
	if (imguiCheck("Select Agent", m_mode == TOOLMODE_SELECT))
		m_mode = TOOLMODE_SELECT;
	
	imguiSeparatorLine();
		
	if (imguiCollapse("Options", 0, m_expandOptions))
		m_expandOptions = !m_expandOptions;
	
	if (m_expandOptions)
	{
		imguiIndent();
		if (imguiCheck("Optimize Visibility", m_optimizeVis))
			m_optimizeVis = !m_optimizeVis;
		if (imguiCheck("Optimize Topology", m_optimizeTopo))
			m_optimizeTopo = !m_optimizeTopo;
		if (imguiCheck("Anticipate Turns", m_anticipateTurns))
			m_anticipateTurns = !m_anticipateTurns;
		if (imguiCheck("Use VO", m_useVO))
			m_useVO = !m_useVO;
		imguiUnindent();
	}

	if (imguiCollapse("Selected Debug Draw", 0, m_expandSelectedDebugDraw))
		m_expandSelectedDebugDraw = !m_expandSelectedDebugDraw;
		
	if (m_expandSelectedDebugDraw)
	{
		imguiIndent();
		if (imguiCheck("Show Corners", m_showCorners))
			m_showCorners = !m_showCorners;
		if (imguiCheck("Show Collision Segs", m_showCollisionSegments))
			m_showCollisionSegments = !m_showCollisionSegments;
		if (imguiCheck("Show Path", m_showPath))
			m_showPath = !m_showPath;
		if (imguiCheck("Show VO", m_showVO))
			m_showVO = !m_showVO;
		if (imguiCheck("Show Path Optimization", m_showOpt))
			m_showOpt = !m_showOpt;
		imguiUnindent();
	}
		
	if (imguiCollapse("Debug Draw", 0, m_expandDebugDraw))
		m_expandDebugDraw = !m_expandDebugDraw;
	
	if (m_expandDebugDraw)
	{
		imguiIndent();
		if (imguiCheck("Show Labels", m_showLabels))
			m_showLabels = !m_showLabels;
		if (imguiCheck("Show Prox Grid", m_showGrid))
			m_showGrid = !m_showGrid;
		if (imguiCheck("Show Nodes", m_showNodes))
			m_showNodes = !m_showNodes;
		if (imguiCheck("Show Perf Graph", m_showPerfGraph))
			m_showPerfGraph = !m_showPerfGraph;
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
				const dtCrowdAgent* ag = m_crowd.getAgent(i);
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
			dtCrowdAgentParams ap;
			ap.radius = m_sample->getAgentRadius();
			ap.height = m_sample->getAgentHeight();
			ap.maxAcceleration = 8.0f;
			ap.maxSpeed = 3.5f;
			ap.collisionQueryRange = ap.radius * 8.0f;
			ap.pathOptimizationRange = ap.radius * 30.0f;
			
			int idx = m_crowd.addAgent(p, &ap);
			if (idx != -1)
			{
				if (m_targetRef)
					m_crowd.requestMoveTarget(idx, m_targetRef, m_targetPos);

				// Init trail
				AgentTrail* trail = &m_trails[idx];
				for (int i = 0; i < AGENT_MAX_TRAIL; ++i)
					dtVcopy(&trail->trail[i*3], p);
				trail->htrail = 0;
			}
		}
	}
	else if (m_mode == TOOLMODE_MOVE_TARGET)
	{
		// Find nearest point on navmesh and set move request to that location.
		dtNavMeshQuery* navquery = m_sample->getNavMeshQuery();
		const dtQueryFilter* filter = m_crowd.getFilter();
		const float* ext = m_crowd.getQueryExtents();
		
		navquery->findNearestPoly(p, ext, filter, &m_targetRef, m_targetPos);
		
		if (shift)
		{
			// Adjust target using tiny local search.
			for (int i = 0; i < m_crowd.getAgentCount(); ++i)
			{
				const dtCrowdAgent* ag = m_crowd.getAgent(i);
				if (!ag->active) continue;
				m_crowd.adjustMoveTarget(i, m_targetRef, m_targetPos);
			}
		}
		else
		{
			// Move target using paht finder
			for (int i = 0; i < m_crowd.getAgentCount(); ++i)
			{
				const dtCrowdAgent* ag = m_crowd.getAgent(i);
				if (!ag->active) continue;
				m_crowd.requestMoveTarget(i, m_targetRef, m_targetPos);
			}
		}
	}
	else if (m_mode == TOOLMODE_SELECT)
	{
		// Highlight
		m_agentDebug.idx = -1;
		
		float tsel = FLT_MAX;
		for (int i = 0; i < m_crowd.getAgentCount(); ++i)
		{
			const dtCrowdAgent* ag = m_crowd.getAgent(i);
			if (!ag->active) continue;
			float bmin[3], bmax[3];
			getAgentBounds(ag, bmin, bmax);
			float tmin, tmax;
			if (isectSegAABB(s, p, bmin,bmax, tmin, tmax))
			{
				if (tmin > 0 && tmin < tsel)
				{
					m_agentDebug.idx = i;
					tsel = tmin;
				} 
			}
		}
	}
}

void CrowdTool::updateTick(const float dt)
{
	dtNavMesh* nav = m_sample->getNavMesh();
	if (!nav)
		return;

	unsigned int flags = 0;
	
	if (m_anticipateTurns)
		flags |= DT_CROWD_ANTICIPATE_TURNS;
	if (m_useVO)
		flags |= DT_CROWD_USE_VO;
	if (m_optimizeVis)
		flags |= DT_CROWD_OPTIMIZE_VIS;
	if (m_optimizeTopo)
		flags |= DT_CROWD_OPTIMIZE_TOPO;
	
	TimeVal startTime = getPerfTime();
	
	m_crowd.update(dt, flags, &m_agentDebug);
	
	TimeVal endTime = getPerfTime();
	
	// Update agent trails
	for (int i = 0; i < m_crowd.getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = m_crowd.getAgent(i);
		AgentTrail* trail = &m_trails[i];
		if (!ag->active)
			continue;
		// Update agent movement trail.
		trail->htrail = (trail->htrail + 1) % AGENT_MAX_TRAIL;
		dtVcopy(&trail->trail[trail->htrail*3], ag->npos);
	}
	
	m_agentDebug.vod->normalizeSamples();
	
	m_crowdSampleCount.addSample((float)m_crowd.getVelocitySampleCount());
	m_crowdTotalTime.addSample(getPerfDeltaTimeUsec(startTime, endTime) / 1000.0f);
}

void CrowdTool::handleStep()
{
	const float dt = 1.0f/20.0f;
	updateTick(dt);
	m_run = false;
}

void CrowdTool::handleToggle()
{
	m_run = !m_run;
}

void CrowdTool::handleUpdate(const float dt)
{
	if (!m_sample) return;
	if (m_run)
		updateTick(dt);
}

void CrowdTool::handleRender()
{
	DebugDrawGL dd;
	const float s = m_sample->getAgentRadius();
	
	dtNavMesh* nmesh = m_sample->getNavMesh();
	if (!nmesh)
		return;

	if (m_showNodes && m_crowd.getPathQueue())
	{
		const dtNavMeshQuery* navquery = m_crowd.getPathQueue()->getNavQuery();
		if (navquery)
			duDebugDrawNavMeshNodes(&dd, *navquery);
	}
	
	dd.depthMask(false);
	
	// Draw paths
	if (m_showPath)
	{
		if (m_agentDebug.idx != -1)
		{
			const dtCrowdAgent* ag = m_crowd.getAgent(m_agentDebug.idx);
			if (ag->active)
			{
					
				const dtPolyRef* path = ag->corridor.getPath();
				const int npath = ag->corridor.getPathCount();			
				for (int i = 0; i < npath; ++i)
					duDebugDrawNavMeshPoly(&dd, *nmesh, path[i], duRGBA(0,0,0,16));
			}
		}
	}
		
	if (m_targetRef)
		duDebugDrawCross(&dd, m_targetPos[0],m_targetPos[1]+0.1f,m_targetPos[2], s, duRGBA(255,255,255,192), 2.0f);
	
	// Occupancy grid.
	if (m_showGrid)
	{
		float gridy = -FLT_MAX;
		for (int i = 0; i < m_crowd.getAgentCount(); ++i)
		{
			const dtCrowdAgent* ag = m_crowd.getAgent(i);
			if (!ag->active) continue;
			const float* pos = ag->corridor.getPos();
			gridy = dtMax(gridy, pos[1]);
		}
		gridy += 1.0f;
		
		dd.begin(DU_DRAW_QUADS);
		const dtProximityGrid* grid = m_crowd.getGrid();
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
	}
	
	// Trail
	for (int i = 0; i < m_crowd.getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = m_crowd.getAgent(i);
		if (!ag->active) continue;
		
		const AgentTrail* trail = &m_trails[i];
		const float* pos = ag->npos;

		dd.begin(DU_DRAW_LINES,3.0f);
		float prev[3], preva = 1;
		dtVcopy(prev, pos);
		for (int j = 0; j < AGENT_MAX_TRAIL-1; ++j)
		{
			const int idx = (trail->htrail + AGENT_MAX_TRAIL-j) % AGENT_MAX_TRAIL;
			const float* v = &trail->trail[idx*3];
			float a = 1 - j/(float)AGENT_MAX_TRAIL;
			dd.vertex(prev[0],prev[1]+0.1f,prev[2], duRGBA(0,0,0,(int)(128*preva)));
			dd.vertex(v[0],v[1]+0.1f,v[2], duRGBA(0,0,0,(int)(128*a)));
			preva = a;
			dtVcopy(prev, v);
		}
		dd.end();

	}

	// Corners & co
	if (m_agentDebug.idx != -1)
	{
		const dtCrowdAgent* ag = m_crowd.getAgent(m_agentDebug.idx);
		if (ag->active)
		{
		
			const float radius = ag->radius;
			const float* pos = ag->npos;
			
			if (m_showCorners)
			{
				if (ag->ncorners)
				{
					dd.begin(DU_DRAW_LINES, 2.0f);
					for (int j = 0; j < ag->ncorners; ++j)
					{
						const float* va = j == 0 ? pos : &ag->cornerVerts[(j-1)*3];
						const float* vb = &ag->cornerVerts[j*3];
						dd.vertex(va[0],va[1]+radius,va[2], duRGBA(128,0,0,192));
						dd.vertex(vb[0],vb[1]+radius,vb[2], duRGBA(128,0,0,192));
					}
					if (ag->ncorners && ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
					{
						const float* v = &ag->cornerVerts[(ag->ncorners-1)*3];
						dd.vertex(v[0],v[1],v[2], duRGBA(192,0,0,192));
						dd.vertex(v[0],v[1]+radius*2,v[2], duRGBA(192,0,0,192));
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
				const float* center = ag->boundary.getCenter();
				duDebugDrawCross(&dd, center[0],center[1]+radius,center[2], 0.2f, duRGBA(192,0,128,255), 2.0f);
				duDebugDrawCircle(&dd, center[0],center[1]+radius,center[2], ag->collisionQueryRange,
								  duRGBA(192,0,128,128), 2.0f);
				
				dd.begin(DU_DRAW_LINES, 3.0f);
				for (int j = 0; j < ag->boundary.getSegmentCount(); ++j)
				{
					const float* s = ag->boundary.getSegment(j);
					unsigned int col = duRGBA(192,0,128,192);
					if (dtTriArea2D(pos, s, s+3) < 0.0f)
						col = duDarkenCol(col);
					
					duAppendArrow(&dd, s[0],s[1]+0.2f,s[2], s[3],s[4]+0.2f,s[5], 0.0f, 0.3f, col);
				}
				dd.end();
			}
			
			if (m_showOpt)
			{
				dd.begin(DU_DRAW_LINES, 2.0f);
				dd.vertex(m_agentDebug.optStart[0],m_agentDebug.optStart[1]+0.3f,m_agentDebug.optStart[2], duRGBA(0,128,0,192));
				dd.vertex(m_agentDebug.optEnd[0],m_agentDebug.optEnd[1]+0.3f,m_agentDebug.optEnd[2], duRGBA(0,128,0,192));
				dd.end();
			}
		}
	}
	
	// Agent cylinders.
	for (int i = 0; i < m_crowd.getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = m_crowd.getAgent(i);
		if (!ag->active) continue;
		
		const float radius = ag->radius;
		const float* pos = ag->npos;
		
		duDebugDrawCircle(&dd, pos[0], pos[1], pos[2], radius, duRGBA(0,0,0,32), 2.0f);
	}

	for (int i = 0; i < m_crowd.getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = m_crowd.getAgent(i);
		if (!ag->active) continue;
		
		const float height = ag->height;
		const float radius = ag->radius;
		const float* pos = ag->npos;
		
		unsigned int col = duRGBA(220,220,220,128);
		if (m_agentDebug.idx == i)
			col = duRGBA(255,192,0,128);
		
		duDebugDrawCylinder(&dd, pos[0]-radius, pos[1]+radius*0.1f, pos[2]-radius,
							pos[0]+radius, pos[1]+height, pos[2]+radius, col);
	}
	
	
	// Velocity stuff.
	for (int i = 0; i < m_crowd.getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = m_crowd.getAgent(i);
		if (!ag->active) continue;
		
		const float radius = ag->radius;
		const float height = ag->height;
		const float* pos = ag->npos;
		const float* vel = ag->vel;
		const float* dvel = ag->dvel;

		unsigned int col = duRGBA(220,220,220,192);
		if (m_agentDebug.idx == i)
			col = duRGBA(255,192,0,192);
		
		duDebugDrawCircle(&dd, pos[0], pos[1]+height, pos[2], radius, col, 2.0f);

		duDebugDrawArrow(&dd, pos[0],pos[1]+height,pos[2],
						 pos[0]+dvel[0],pos[1]+height+dvel[1],pos[2]+dvel[2],
						 0.0f, 0.4f, duRGBA(0,192,255,192), 1.0f);
		
		duDebugDrawArrow(&dd, pos[0],pos[1]+height,pos[2],
							  pos[0]+vel[0],pos[1]+height+vel[1],pos[2]+vel[2],
							  0.0f, 0.4f, duRGBA(0,0,0,192), 2.0f);
	}

	if (m_agentDebug.idx != -1)
	{
		const dtCrowdAgent* ag = m_crowd.getAgent(m_agentDebug.idx);
		if (ag->active)
		{
			if (m_showVO)
			{
				// Draw detail about agent sela
				const dtObstacleAvoidanceDebugData* vod = m_agentDebug.vod;
				
				const float dx = ag->npos[0];
				const float dy = ag->npos[1]+ag->height;
				const float dz = ag->npos[2];
				
				dd.begin(DU_DRAW_QUADS);
				for (int i = 0; i < vod->getSampleCount(); ++i)
				{
					const float* p = vod->getSampleVelocity(i);
					const float sr = vod->getSampleSize(i);
					const float pen = vod->getSamplePenalty(i);
					const float pen2 = vod->getSamplePreferredSidePenalty(i);
					unsigned int col = duLerpCol(duRGBA(255,255,255,220), duRGBA(128,96,0,220), (int)(pen*255));
					col = duLerpCol(col, duRGBA(128,0,0,220), (int)(pen2*128));
					dd.vertex(dx+p[0]-sr, dy, dz+p[2]-sr, col);
					dd.vertex(dx+p[0]-sr, dy, dz+p[2]+sr, col);
					dd.vertex(dx+p[0]+sr, dy, dz+p[2]+sr, col);
					dd.vertex(dx+p[0]+sr, dy, dz+p[2]-sr, col);
				}
				dd.end();
			}		
		}
	}
	
	dd.depthMask(true);
}

void CrowdTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_targetRef && gluProject((GLdouble)m_targetPos[0], (GLdouble)m_targetPos[1], (GLdouble)m_targetPos[2],
								model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y+25), IMGUI_ALIGN_CENTER, "TARGET", imguiRGBA(0,0,0,220));
	}
	
	if (m_showLabels)
	{
		char label[32];
		for (int i = 0; i < m_crowd.getAgentCount(); ++i)
		{
			const dtCrowdAgent* ag = m_crowd.getAgent(i);
			if (!ag->active) continue;
			const float* pos = ag->npos;
			const float h = ag->height;
			if (gluProject((GLdouble)pos[0], (GLdouble)pos[1]+h, (GLdouble)pos[2],
						   model, proj, view, &x, &y, &z))
			{
				snprintf(label, 32, "%d", i);
				imguiDrawText((int)x, (int)y+15, IMGUI_ALIGN_CENTER, label, imguiRGBA(0,0,0,220));
			}
			
		}
	}
	
	if (m_showPerfGraph)
	{
		GraphParams gp;
		gp.setRect(300, 10, 500, 200, 8);
		gp.setValueRange(0.0f, 2.0f, 4, "ms");

		drawGraphBackground(&gp);
		drawGraph(&gp, &m_crowdTotalTime, 1, "Total", duRGBA(255,128,0,255));
		
		gp.setRect(300, 10, 500, 50, 8);
		gp.setValueRange(0.0f, 2000.0f, 1, "");
		drawGraph(&gp, &m_crowdSampleCount, 0, "Sample Count", duRGBA(96,96,96,128));
	}
	
	// Tool help
	const int h = view[3];
	int ty = h-40;
	
	if (m_mode == TOOLMODE_CREATE)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: add agent.  Shift+LMB: remove agent.", imguiRGBA(255,255,255,192));	
	}
	else if (m_mode == TOOLMODE_MOVE_TARGET)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: set move target.  Shift+LMB: adjust target.", imguiRGBA(255,255,255,192));	
		ty -= 20;
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "Adjusting allows to change the target location in short range without pathfinder.", imguiRGBA(255,255,255,192));	
	}
	else if (m_mode == TOOLMODE_SELECT)
	{
		imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "LMB: select agent.", imguiRGBA(255,255,255,192));	
	}
	ty -= 20;
	imguiDrawText(280, ty, IMGUI_ALIGN_LEFT, "SPACE: Run/Pause simulation.  1: Step simulation.", imguiRGBA(255,255,255,192));	
	
}
