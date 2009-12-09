#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "Sample.h"
#include "Sample_SoloMesh.h"
#include "Recast.h"
#include "RecastTimer.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

inline bool inRange(const float* v1, const float* v2, const float r, const float h)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return (dx*dx + dz*dz) < r*r && fabsf(dy) < h;
}
						

Sample_SoloMesh::Sample_SoloMesh() :
	m_navMesh(0),
	m_toolMode(TOOLMODE_PATHFIND),
	m_startRef(0),
	m_endRef(0),
	m_npolys(0),
	m_nstraightPath(0),
	m_nsmoothPath(0),
	m_distanceToWall(0),
	m_sposSet(false),
	m_eposSet(false)
{
	toolReset();
	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;
}

Sample_SoloMesh::~Sample_SoloMesh()
{
	toolCleanup();
}

void Sample_SoloMesh::handleTools()
{
	if (imguiCheck("Pathfind", m_toolMode == TOOLMODE_PATHFIND))
	{
		m_toolMode = TOOLMODE_PATHFIND;
		toolRecalc();
	}
	if (imguiCheck("Distance to Wall", m_toolMode == TOOLMODE_DISTANCE_TO_WALL))
	{
		m_toolMode = TOOLMODE_DISTANCE_TO_WALL;
		toolRecalc();
	}
	if (imguiCheck("Raycast", m_toolMode == TOOLMODE_RAYCAST))
	{
		m_toolMode = TOOLMODE_RAYCAST;
		toolRecalc();
	}
	if (imguiCheck("Find Polys Around", m_toolMode == TOOLMODE_FIND_POLYS_AROUND))
	{
		m_toolMode = TOOLMODE_FIND_POLYS_AROUND;
		toolRecalc();
	}
}

void Sample_SoloMesh::setToolStartPos(const float* p)
{
	m_sposSet = true;
	vcopy(m_spos, p);
	toolRecalc();
}

void Sample_SoloMesh::setToolEndPos(const float* p)
{
	m_eposSet = true;
	vcopy(m_epos, p);
	toolRecalc();
}

void Sample_SoloMesh::toolCleanup()
{
	delete m_navMesh;
	m_navMesh = 0;
}

void Sample_SoloMesh::toolReset()
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

void Sample_SoloMesh::toolRecalc()
{
	if (!m_navMesh)
		return;

	if (m_sposSet)
		m_startRef = m_navMesh->findNearestPoly(m_spos, m_polyPickExt);
	else
		m_startRef = 0;

	if (m_eposSet)
		m_endRef = m_navMesh->findNearestPoly(m_epos, m_polyPickExt);
	else
		m_endRef = 0;
	
	if (m_toolMode == TOOLMODE_PATHFIND)
	{
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
			m_npolys = m_navMesh->findPath(m_startRef, m_endRef, m_spos, m_epos, m_polys, MAX_POLYS);
			if (m_npolys)
			{
				m_nstraightPath = m_navMesh->findStraightPath(m_spos, m_epos, m_polys, m_npolys, m_straightPath, MAX_POLYS);
				
				// Iterate over the path to find smooth path on the detail mesh surface.
				const dtPolyRef* polys = m_polys; 
				int npolys = m_npolys;
				
				float iterPos[3], targetPos[3];
				m_navMesh->closestPointOnPolyBoundary(m_startRef, m_spos, iterPos);
				m_navMesh->closestPointOnPolyBoundary(polys[npolys-1], m_epos, targetPos);
				
				static const float STEP_SIZE = 0.5f;
				static const float SLOP = 0.01f;
				
				m_nsmoothPath = 0;
				
				vcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
				m_nsmoothPath++;

				while (npolys && m_nsmoothPath < MAX_SMOOTH)
				{
					// Find steer target.
					static const int MAX_STEER = 3;
					float steerPath[MAX_STEER*3];
					int nsteerPath = m_navMesh->findStraightPath(iterPos, m_epos, polys, npolys, steerPath, MAX_STEER);
					if (!nsteerPath)
						break;
					// Find vertex far enough to steer to.
					int ns = 0;
					while (ns < nsteerPath)
					{
						if (!inRange(&steerPath[ns*3], iterPos, SLOP, 1000.0f))
							break;
						ns++;
					}
					if (ns >= nsteerPath)
						break;
					bool endOfPath = inRange(&steerPath[ns*3], targetPos, SLOP*SLOP, 1.0f);
					
					// Find movement delta.
					float delta[3], len;
					vsub(delta, &steerPath[ns*3], iterPos);
					len = sqrtf(vdot(delta,delta));
					if (endOfPath && len < STEP_SIZE)
						len = 1;
					else
						len = STEP_SIZE / len;
					float moveTgt[3];
					vmad(moveTgt, iterPos, delta, len);

					// Move
					float result[3];
					int n = m_navMesh->moveAlongPathCorridor(iterPos, moveTgt, result, polys, npolys);
					float h = 0;
					m_navMesh->getPolyHeight(polys[n], result, &h);
					result[1] = h;
					// Shrink path corridor of possible.
					polys += n;
					npolys -= n;

					vcopy(iterPos, result);
					
					// Close enough to the target.
					if (inRange(iterPos, targetPos, SLOP, 1.0f))
					{
						vcopy(iterPos, targetPos);
						npolys = 0;
					}
					
					// Store results.
					if (m_nsmoothPath < MAX_SMOOTH)
					{
						vcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
						m_nsmoothPath++;
					}
				}

			}
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
			float t = 0;
			m_npolys = 0;
			m_nstraightPath = 2;
			m_straightPath[0] = m_spos[0];
			m_straightPath[1] = m_spos[1];
			m_straightPath[2] = m_spos[2];
			m_npolys = m_navMesh->raycast(m_startRef, m_spos, m_epos, t, m_polys, MAX_POLYS);
			if (t < 1)
			{
				m_straightPath[3] = m_spos[0] + (m_epos[0] - m_spos[0]) * t;
				m_straightPath[4] = m_spos[1] + (m_epos[1] - m_spos[1]) * t;
				m_straightPath[5] = m_spos[2] + (m_epos[2] - m_spos[2]) * t;
			}
			else
			{
				m_straightPath[3] = m_epos[0];
				m_straightPath[4] = m_epos[1];
				m_straightPath[5] = m_epos[2];
			}
		}
	}
	else if (m_toolMode == TOOLMODE_DISTANCE_TO_WALL)
	{
		m_distanceToWall = 0;
		if (m_sposSet && m_startRef)
			m_distanceToWall = m_navMesh->findDistanceToWall(m_startRef, m_spos, 100.0f, m_hitPos, m_hitNormal);
	}
	else if (m_toolMode == TOOLMODE_FIND_POLYS_AROUND)
	{
		if (m_sposSet && m_startRef && m_eposSet)
		{
			const float dx = m_epos[0] - m_spos[0];
			const float dz = m_epos[2] - m_spos[2];
			float dist = sqrtf(dx*dx + dz*dz);
			m_npolys = m_navMesh->findPolysAround(m_startRef, m_spos, dist, m_polys, m_parent, 0, MAX_POLYS);
		}
	}
}

static void getPolyCenter(dtNavMesh* navMesh, dtPolyRef ref, float* center)
{
	const dtPoly* p = navMesh->getPolyByRef(ref);
	if (!p) return;
	const float* verts = navMesh->getPolyVertsByRef(ref);
	center[0] = 0;
	center[1] = 0;
	center[2] = 0;
	for (int i = 0; i < (int)p->nv; ++i)
	{
		const float* v = &verts[p->v[i]*3];
		center[0] += v[0];
		center[1] += v[1];
		center[2] += v[2];
	}
	const float s = 1.0f / p->nv;
	center[0] *= s;
	center[1] *= s;
	center[2] *= s;
}

void Sample_SoloMesh::toolRender(int flags)
{
	if (!m_navMesh)
		return;
	
	DebugDrawGL dd;

	static const float startCol[4] = { 0.5f, 0.1f, 0.0f, 0.75f };
	static const float endCol[4] = { 0.2f, 0.4f, 0.0f, 0.75f };
	static const float pathCol[4] = {0,0,0,0.25f};

	glDepthMask(GL_FALSE);

	if (flags & NAVMESH_POLYS)
		duDebugDrawNavMesh(&dd, m_navMesh, m_toolMode == TOOLMODE_PATHFIND);
	
	if (flags & NAVMESH_BVTREE)
		duDebugDrawNavMeshBVTree(&dd, m_navMesh);
	
	if (flags & NAVMESH_TOOLS)
	{
		if (m_toolMode == TOOLMODE_PATHFIND)
		{
			duDebugDrawNavMeshPoly(&dd, m_navMesh, m_startRef, startCol);
			duDebugDrawNavMeshPoly(&dd, m_navMesh, m_endRef, endCol);
				
			if (m_npolys)
			{
				for (int i = 1; i < m_npolys-1; ++i)
					duDebugDrawNavMeshPoly(&dd, m_navMesh, m_polys[i], pathCol);
			}
			if (m_nstraightPath)
			{
				glColor4ub(64,16,0,64);
				glLineWidth(2.0f);
				glBegin(GL_LINE_STRIP);
				for (int i = 0; i < m_nstraightPath; ++i)
					glVertex3f(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2]);
				glEnd();
				glLineWidth(1.0f);
				glColor4ub(64,16,0,128);
				glPointSize(3.0f);
				glBegin(GL_POINTS);
				for (int i = 0; i < m_nstraightPath; ++i)
					glVertex3f(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2]);
				glEnd();
				glPointSize(1.0f);
			}
			if (m_nsmoothPath)
			{
				glColor4ub(0,0,0,220);
				glLineWidth(3.0f);
				glBegin(GL_LINES);
				for (int i = 0; i < m_nsmoothPath; ++i)
					glVertex3f(m_smoothPath[i*3], m_smoothPath[i*3+1], m_smoothPath[i*3+2]);
				glEnd();
				glLineWidth(1.0f);
			}
		}
		else if (m_toolMode == TOOLMODE_RAYCAST)
		{
			duDebugDrawNavMeshPoly(&dd, m_navMesh, m_startRef, startCol);
			
			if (m_nstraightPath)
			{
				for (int i = 1; i < m_npolys; ++i)
					duDebugDrawNavMeshPoly(&dd, m_navMesh, m_polys[i], pathCol);
				
				glColor4ub(64,16,0,220);
				glLineWidth(3.0f);
				glBegin(GL_LINE_STRIP);
				for (int i = 0; i < m_nstraightPath; ++i)
					glVertex3f(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2]);
				glEnd();
				glLineWidth(1.0f);
				glPointSize(4.0f);
				glBegin(GL_POINTS);
				for (int i = 0; i < m_nstraightPath; ++i)
					glVertex3f(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2]);
				glEnd();
				glPointSize(1.0f);
			}
		}
		else if (m_toolMode == TOOLMODE_DISTANCE_TO_WALL)
		{
			duDebugDrawNavMeshPoly(&dd, m_navMesh, m_startRef, startCol);
			const float col[4] = {1,1,1,0.5f};
			duDebugDrawCylinderWire(&dd, m_spos[0]-m_distanceToWall, m_spos[1]+0.02f, m_spos[2]-m_distanceToWall,
									m_spos[0]+m_distanceToWall, m_spos[1]+m_agentHeight, m_spos[2]+m_distanceToWall, col);
			glLineWidth(3.0f);
			glColor4fv(col);
			glBegin(GL_LINES);
			glVertex3f(m_hitPos[0], m_hitPos[1] + 0.02f, m_hitPos[2]);
			glVertex3f(m_hitPos[0], m_hitPos[1] + m_agentHeight, m_hitPos[2]);
			glEnd();
			glLineWidth(1.0f);
		}
		else if (m_toolMode == TOOLMODE_FIND_POLYS_AROUND)
		{
			const float cola[4] = {0,0,0,0.5f};
			for (int i = 0; i < m_npolys; ++i)
			{
				duDebugDrawNavMeshPoly(&dd, m_navMesh, m_polys[i], pathCol);
				if (m_parent[i])
				{
					float p0[3], p1[3];
					getPolyCenter(m_navMesh, m_polys[i], p0);
					getPolyCenter(m_navMesh, m_parent[i], p1);
					duDebugDrawArc(&dd, p0, p1, cola, 2.0f);
				}
			}
			
			const float dx = m_epos[0] - m_spos[0];
			const float dz = m_epos[2] - m_spos[2];
			float dist = sqrtf(dx*dx + dz*dz);
			const float col[4] = {1,1,1,0.5f};
			duDebugDrawCylinderWire(&dd, m_spos[0]-dist, m_spos[1]+0.02f, m_spos[2]-dist,
									m_spos[0]+dist, m_spos[1]+m_agentHeight, m_spos[2]+dist, col);					
		}
	}
	
	glDepthMask(GL_TRUE);
}

void Sample_SoloMesh::toolRenderOverlay(double* proj, double* model, int* view)
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
}

void Sample_SoloMesh::drawAgent(const float* pos, float r, float h, float c, const float* col)
{
	DebugDrawGL dd;
	
	glDepthMask(GL_FALSE);
	
	// Agent dimensions.	
	glLineWidth(2.0f);
	duDebugDrawCylinderWire(&dd, pos[0]-r, pos[1]+0.02f, pos[2]-r, pos[0]+r, pos[1]+h, pos[2]+r, col);
	glLineWidth(1.0f);
		
	glColor4ub(0,0,0,196);
	glBegin(GL_LINES);
	glVertex3f(pos[0], pos[1]-c, pos[2]);
	glVertex3f(pos[0], pos[1]+c, pos[2]);
	glVertex3f(pos[0]-r/2, pos[1]+0.02f, pos[2]);
	glVertex3f(pos[0]+r/2, pos[1]+0.02f, pos[2]);
	glVertex3f(pos[0], pos[1]+0.02f, pos[2]-r/2);
	glVertex3f(pos[0], pos[1]+0.02f, pos[2]+r/2);
	glEnd();

	glDepthMask(GL_TRUE);
}
