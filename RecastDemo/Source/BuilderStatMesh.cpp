#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "SDL.h"
#include "SDL_Opengl.h"
#include "glfont.h"
#include "imgui.h"
#include "Builder.h"
#include "BuilderStatMesh.h"
#include "Recast.h"
#include "RecastTimer.h"
#include "RecastDebugDraw.h"
#include "DetourStatNavMesh.h"
#include "DetourStatNavMeshBuilder.h"
#include "DetourDebugDraw.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

BuilderStatMesh::BuilderStatMesh() :
	m_navMesh(0),
	m_toolMode(TOOLMODE_PATHFIND),
	m_sposSet(false),
	m_eposSet(false)
{
	toolReset();
	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;
}

BuilderStatMesh::~BuilderStatMesh()
{
	toolCleanup();
}

void BuilderStatMesh::handleTools()
{
	if (imguiCheck(GENID, "Pathfind", m_toolMode == TOOLMODE_PATHFIND))
	{
		m_toolMode = TOOLMODE_PATHFIND;
		toolRecalc();
	}
	if (imguiCheck(GENID, "Distance to Wall", m_toolMode == TOOLMODE_DISTANCE_TO_WALL))
	{
		m_toolMode = TOOLMODE_DISTANCE_TO_WALL;
		toolRecalc();
	}
	if (imguiCheck(GENID, "Raycast", m_toolMode == TOOLMODE_RAYCAST))
	{
		m_toolMode = TOOLMODE_RAYCAST;
		toolRecalc();
	}
	if (imguiCheck(GENID, "Find Polys Around", m_toolMode == TOOLMODE_FIND_POLYS_AROUND))
	{
		m_toolMode = TOOLMODE_FIND_POLYS_AROUND;
		toolRecalc();
	}
}

void BuilderStatMesh::setToolStartPos(const float* p)
{
	m_sposSet = true;
	vcopy(m_spos, p);
	toolRecalc();
}

void BuilderStatMesh::setToolEndPos(const float* p)
{
	m_eposSet = true;
	vcopy(m_epos, p);
	toolRecalc();
}

void BuilderStatMesh::toolCleanup()
{
	delete m_navMesh;
	m_navMesh = 0;
}

void BuilderStatMesh::toolReset()
{
	m_startRef = 0;
	m_endRef = 0;
	m_npolys = 0;
	m_nstraightPath = 0;
	memset(m_hitPos, 0, sizeof(m_hitPos));
	memset(m_hitNormal, 0, sizeof(m_hitNormal));
	m_distanceToWall = 0;
}

void BuilderStatMesh::toolRecalc()
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
			m_npolys = m_navMesh->findPath(m_startRef, m_endRef, m_polys, MAX_POLYS);
			if (m_npolys)
				m_nstraightPath = m_navMesh->findStraightPath(m_spos, m_epos, m_polys, m_npolys, m_straightPath, MAX_POLYS);
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
			if (m_navMesh->raycast(m_startRef, m_spos, m_epos, t, m_polys[0]))
			{
				m_npolys = 1;
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
			m_npolys = m_navMesh->findPolysAround(m_startRef, m_spos, dist, m_polys, m_parent, 0, 0, MAX_POLYS);
		}
	}
}

static void getPolyCenter(dtStatNavMesh* navMesh, dtPolyRef ref, float* center)
{
	const dtPoly* p = navMesh->getPolyByRef(ref);
	if (!p) return;
	center[0] = 0;
	center[1] = 0;
	center[2] = 0;
	for (int i = 0; i < (int)p->nv; ++i)
	{
		const float* v = navMesh->getVertex(p->v[i]);
		center[0] += v[0];
		center[1] += v[1];
		center[2] += v[2];
	}
	const float s = 1.0f / p->nv;
	center[0] *= s;
	center[1] *= s;
	center[2] *= s;
}

void BuilderStatMesh::toolRender(int flags)
{
	if (!m_navMesh)
		return;
	
	static const float startCol[4] = { 0.5f, 0.1f, 0.0f, 0.75f };
	static const float endCol[4] = { 0.2f, 0.4f, 0.0f, 0.75f };
	static const float pathCol[4] = {0,0,0,0.25f};

	glDepthMask(GL_FALSE);

	if (flags & NAVMESH_POLYS)
		dtDebugDrawStatNavMesh(m_navMesh);
	
	if (flags & NAVMESH_BVTREE)
		dtDebugDrawStatNavMeshBVTree(m_navMesh);
	
	if (flags & NAVMESH_TOOLS)
	{
		if (m_toolMode == TOOLMODE_PATHFIND)
		{
			dtDebugDrawStatNavMeshPoly(m_navMesh, m_startRef, startCol);
			dtDebugDrawStatNavMeshPoly(m_navMesh, m_endRef, endCol);
				
			if (m_npolys)
			{
				for (int i = 1; i < m_npolys-1; ++i)
					dtDebugDrawStatNavMeshPoly(m_navMesh, m_polys[i], pathCol);
			}
			if (m_nstraightPath)
			{
				glColor4ub(128,16,0,220);
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
		else if (m_toolMode == TOOLMODE_RAYCAST)
		{
			dtDebugDrawStatNavMeshPoly(m_navMesh, m_startRef, startCol);
			
			if (m_nstraightPath)
			{
				dtDebugDrawStatNavMeshPoly(m_navMesh, m_polys[0], pathCol);
				
				glColor4ub(128,16,0,220);
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
			dtDebugDrawStatNavMeshPoly(m_navMesh, m_startRef, startCol);
			const float col[4] = {1,1,1,0.5f};
			rcDebugDrawCylinderWire(m_spos[0]-m_distanceToWall, m_spos[1]+0.02f, m_spos[2]-m_distanceToWall,
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
			glLineWidth(2.0f);
			for (int i = 0; i < m_npolys; ++i)
			{
				dtDebugDrawStatNavMeshPoly(m_navMesh, m_polys[i], pathCol);
				if (m_parent[i])
				{
					float p0[3], p1[3];
					getPolyCenter(m_navMesh, m_polys[i], p0);
					getPolyCenter(m_navMesh, m_parent[i], p1);
					glColor4ub(0,0,0,128);
					rcDrawArc(p0, p1);
				}
			}
			glLineWidth(1.0f);
			
			const float dx = m_epos[0] - m_spos[0];
			const float dz = m_epos[2] - m_spos[2];
			float dist = sqrtf(dx*dx + dz*dz);
			const float col[4] = {1,1,1,0.5f};
			rcDebugDrawCylinderWire(m_spos[0]-dist, m_spos[1]+0.02f, m_spos[2]-dist,
									m_spos[0]+dist, m_spos[1]+m_agentHeight, m_spos[2]+dist, col);					
		}
	}
	
	glDepthMask(GL_TRUE);
}

void BuilderStatMesh::toolRenderOverlay(class GLFont* font, double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_sposSet && gluProject((GLdouble)m_spos[0], (GLdouble)m_spos[1], (GLdouble)m_spos[2],
							  model, proj, view, &x, &y, &z))
	{
		const float len = font->getTextLength("Start");
		font->drawText((float)x - len/2, (float)y-font->getLineHeight(), "Start", GLFont::RGBA(0,0,0,220));
	}
	if (m_eposSet && gluProject((GLdouble)m_epos[0], (GLdouble)m_epos[1], (GLdouble)m_epos[2],
							  model, proj, view, &x, &y, &z))
	{
		const float len = font->getTextLength("End");
		font->drawText((float)x-len/2, (float)y-font->getLineHeight(), "End", GLFont::RGBA(0,0,0,220));
	}
}

void BuilderStatMesh::drawAgent(const float* pos, float r, float h, float c, const float* col)
{
	glDepthMask(GL_FALSE);
	
	// Agent dimensions.	
	glLineWidth(2.0f);
	rcDebugDrawCylinderWire(pos[0]-r, pos[1]+0.02f, pos[2]-r, pos[0]+r, pos[1]+h, pos[2]+r, col);
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
