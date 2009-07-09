//
// Copyright (c) 2009 Mikko Mononen memon@inside.org
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
#include "SDL_Opengl.h"
#include "imgui.h"
#include "glfont.h"
#include "Builder.h"
#include "BuilderTiledMesh.h"
#include "Recast.h"
#include "RecastTimer.h"
#include "RecastDebugDraw.h"
#include "DetourTiledNavMesh.h"
#include "DetourTiledNavMeshBuilder.h"
#include "DetourDebugDraw.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


BuilderTiledMesh::BuilderTiledMesh() :
	m_tileSize(32),
	m_navMesh(0),
	m_chunkyMesh(0),
	m_keepInterResults(true),
	m_tileBuildTime(0),
	m_tileMemUsage(0),
	m_triflags(0),
	m_solid(0),
	m_chf(0),
	m_cset(0),
	m_polyMesh(0),
	m_tileTriCount(0),
	m_toolMode(TOOLMODE_CREATE_TILES),
	m_startRef(0),
	m_endRef(0),
	m_npolys(0),
	m_nstraightPath(0)
{
	resetCommonSettings();
	memset(m_tileBmin, 0, sizeof(m_tileBmin));
	memset(m_tileBmax, 0, sizeof(m_tileBmax));
	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;
}

BuilderTiledMesh::~BuilderTiledMesh()
{
	cleanup();
	delete m_navMesh;
	delete m_chunkyMesh;
}

void BuilderTiledMesh::cleanup()
{
	delete [] m_triflags;
	m_triflags = 0;
	delete m_solid;
	m_solid = 0;
	delete m_chf;
	m_chf = 0;
	delete m_cset;
	m_cset = 0;
	delete m_polyMesh;
	m_polyMesh = 0;
}

void BuilderTiledMesh::handleSettings()
{
	Builder::handleCommonSettings();

	imguiLabel(GENID, "Tiling");
	imguiSlider(GENID, "TileSize", &m_tileSize, 16.0f, 1024.0f, 16.0f);
	
	char text[64];
	int gw = 0, gh = 0;
	rcCalcGridSize(m_bmin, m_bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;
	snprintf(text, 64, "Tiles  %d x %d", tw, th);
	imguiValue(GENID, text);
}

void BuilderTiledMesh::toolRecalc()
{
	m_startRef = 0;
	if (m_sposSet)
		m_startRef = m_navMesh->findNearestPoly(m_spos, m_polyPickExt);

	m_endRef = 0;
	if (m_eposSet)
		m_endRef = m_navMesh->findNearestPoly(m_epos, m_polyPickExt);

//	if (m_eposSet)
//		m_npolys = m_navMesh->queryPolygons(m_epos, m_polyPickExt, m_polys, MAX_POLYS);

/*	if (m_startRef && m_endRef)
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
*/
	
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
			m_npolys = m_navMesh->raycast(m_startRef, m_spos, m_epos, t, m_polys, MAX_POLYS);
			if (m_npolys && t < 1)
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
			m_npolys = m_navMesh->findPolysAround(m_startRef, m_spos, dist, m_polys, m_parent, 0, 0, MAX_POLYS);
		}
	}
	
}

void BuilderTiledMesh::handleTools()
{
	if (imguiCheck(GENID, "Create Tiles", m_toolMode == TOOLMODE_CREATE_TILES))
	{
		m_toolMode = TOOLMODE_CREATE_TILES;
		toolRecalc();
	}
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

void BuilderTiledMesh::handleDebugMode()
{
}

static void getPolyCenter(dtTiledNavMesh* navMesh, dtTilePolyRef ref, float* center)
{
	const dtTilePoly* p = navMesh->getPolyByRef(ref);
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

void BuilderTiledMesh::handleRender()
{
	if (!m_verts || !m_tris || !m_trinorms)
		return;

	// Draw mesh
	rcDebugDrawMesh(m_verts, m_nverts, m_tris, m_trinorms, m_ntris, 0);

	glDepthMask(GL_FALSE);
	
	// Draw bounds
	float col[4] = {1,1,1,0.5f};
	rcDebugDrawBoxWire(m_bmin[0],m_bmin[1],m_bmin[2], m_bmax[0],m_bmax[1],m_bmax[2], col);

	// Tiling grid.
	const int ts = (int)m_tileSize;
	int gw = 0, gh = 0;
	rcCalcGridSize(m_bmin, m_bmax, m_cellSize, &gw, &gh);
	int tw = (gw + ts-1) / ts;
	int th = (gh + ts-1) / ts;
	const float s = ts*m_cellSize;
	glBegin(GL_LINES);
	glColor4ub(0,0,0,64);
	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			float fx, fy, fz;
			fx = m_bmin[0] + x*s;
			fy = m_bmin[1];
			fz = m_bmin[2] + y*s;
			
			glVertex3f(fx,fy,fz);
			glVertex3f(fx+s,fy,fz);
			glVertex3f(fx,fy,fz);
			glVertex3f(fx,fy,fz+s);
			
			if (x+1 >= tw)
			{
				glVertex3f(fx+s,fy,fz);
				glVertex3f(fx+s,fy,fz+s);
			}
			if (y+1 >= th)
			{
				glVertex3f(fx,fy,fz+s);
				glVertex3f(fx+s,fy,fz+s);
			}
		}
	}
	glEnd();
	
	// Draw active tile
	rcDebugDrawBoxWire(m_tileBmin[0],m_tileBmin[1],m_tileBmin[2], m_tileBmax[0],m_tileBmax[1],m_tileBmax[2], m_tileCol);

/*	if (m_eposSet)
	{
		col[0] = 1; col[1] = 1; col[2] = 1; col[3] = 1;
		const float s = 0.2f;
		rcDebugDrawBoxWire(m_epos[0]-s,m_epos[1]-s,m_epos[2]-s, m_epos[0]+s,m_epos[1]+s,m_epos[2]+s, col);
	}*/

/*	if (m_polyMesh)
		rcDebugDrawPolyMesh(*m_polyMesh);*/
		
	if (m_navMesh)
		dtDebugDrawTiledNavMesh(m_navMesh);
	
	if (m_sposSet)
	{
		const float s = 0.5f;
		glColor4ub(128,0,0,255);
		glBegin(GL_LINES);
		glVertex3f(m_spos[0]-s,m_spos[1],m_spos[2]);
		glVertex3f(m_spos[0]+s,m_spos[1],m_spos[2]);
		glVertex3f(m_spos[0],m_spos[1]-s,m_spos[2]);
		glVertex3f(m_spos[0],m_spos[1]+s,m_spos[2]);
		glVertex3f(m_spos[0],m_spos[1],m_spos[2]-s);
		glVertex3f(m_spos[0],m_spos[1],m_spos[2]+s);
		glEnd();
	}
	if (m_eposSet)
	{
		const float s = 0.5f;
		glColor4ub(0,128,0,255);
		glBegin(GL_LINES);
		glVertex3f(m_epos[0]-s,m_epos[1],m_epos[2]);
		glVertex3f(m_epos[0]+s,m_epos[1],m_epos[2]);
		glVertex3f(m_epos[0],m_epos[1]-s,m_epos[2]);
		glVertex3f(m_epos[0],m_epos[1]+s,m_epos[2]);
		glVertex3f(m_epos[0],m_epos[1],m_epos[2]-s);
		glVertex3f(m_epos[0],m_epos[1],m_epos[2]+s);
		glEnd();
		col[0] = 0; col[1] = 1; col[2] = 0; col[3] = 1;
		rcDebugDrawBoxWire(m_epos[0]-m_polyPickExt[0],m_epos[1]-m_polyPickExt[1],m_epos[2]-m_polyPickExt[2], 
						   m_epos[0]+m_polyPickExt[0],m_epos[1]+m_polyPickExt[1],m_epos[2]+m_polyPickExt[2], col);
	}
	
/*	if (m_startRef && m_navMesh)
	{
		col[0] = 1; col[1] = 0; col[2] = 0; col[3] = 1;
		dtDebugDrawTiledNavMeshPoly(m_navMesh, m_startRef, col);
	}
	if (m_endRef && m_navMesh)
	{
		col[0] = 0; col[1] = 1; col[2] = 0; col[3] = 1;
		dtDebugDrawTiledNavMeshPoly(m_navMesh, m_endRef, col);
		dtTilePolyRef nei[DT_TILE_VERTS_PER_POLYGON*2];
		int nn = m_navMesh->getPolyNeighbours(m_endRef, nei, DT_TILE_VERTS_PER_POLYGON*2);
		if (nn)
		{
			col[0] = 0; col[1] = 0; col[2] = 1; col[3] = 1;
			for (int i = 0; i < nn; ++i)
				dtDebugDrawTiledNavMeshPoly(m_navMesh, nei[i], col);
		}
	}

	if (m_npolys && m_navMesh)
	{
		col[0] = 0; col[1] = 0; col[2] = 0; col[3] = 1;
		for (int i = 0; i < m_npolys; ++i)
			dtDebugDrawTiledNavMeshPoly(m_navMesh, m_polys[i], col);
	}
	if (m_nstraightPath && m_navMesh)
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
	}*/
	
	
	static const float startCol[4] = { 0.5f, 0.1f, 0.0f, 0.75f };
	static const float endCol[4] = { 0.2f, 0.4f, 0.0f, 0.75f };
	static const float pathCol[4] = {0,0,0,0.25f};
	
	if (m_toolMode == TOOLMODE_PATHFIND)
	{
		dtDebugDrawTiledNavMeshPoly(m_navMesh, m_startRef, startCol);
		dtDebugDrawTiledNavMeshPoly(m_navMesh, m_endRef, endCol);
		
		if (m_npolys)
		{
			for (int i = 1; i < m_npolys-1; ++i)
				dtDebugDrawTiledNavMeshPoly(m_navMesh, m_polys[i], pathCol);
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
		dtDebugDrawTiledNavMeshPoly(m_navMesh, m_startRef, startCol);
		
		if (m_nstraightPath)
		{
			for (int i = 1; i < m_npolys; ++i)
				dtDebugDrawTiledNavMeshPoly(m_navMesh, m_polys[i], pathCol);
			
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
		dtDebugDrawTiledNavMeshPoly(m_navMesh, m_startRef, startCol);
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
			dtDebugDrawTiledNavMeshPoly(m_navMesh, m_polys[i], pathCol);
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
	
	glDepthMask(GL_TRUE);
	
}

void BuilderTiledMesh::handleRenderOverlay(class GLFont* font, double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_tileBuildTime > 0.0f && gluProject((GLdouble)(m_tileBmin[0]+m_tileBmax[0])/2, (GLdouble)(m_tileBmin[1]+m_tileBmax[1])/2, (GLdouble)(m_tileBmin[2]+m_tileBmax[2])/2,
								model, proj, view, &x, &y, &z))
	{
		char text[32];
		snprintf(text,32,"%.3fms / %dTris / %.1fkB", m_tileBuildTime, m_tileTriCount, m_tileMemUsage);
		const float len = font->getTextLength(text);
		font->drawText((float)x - len/2, (float)y-font->getLineHeight(), text, GLFont::RGBA(0,0,0,220));
	}
}

void BuilderTiledMesh::handleMeshChanged(const float* verts, int nverts,
								const int* tris, const float* trinorms, int ntris,
								const float* bmin, const float* bmax)
{
	m_verts = verts;
	m_nverts = nverts;
	m_tris = tris;
	m_trinorms = trinorms;
	m_ntris = ntris;
	vcopy(m_bmin, bmin);
	vcopy(m_bmax, bmax);

	delete m_chunkyMesh;
	m_chunkyMesh = 0;
	delete m_navMesh;
	m_navMesh = 0;
	cleanup();
}

void BuilderTiledMesh::setToolStartPos(const float* p)
{
	m_sposSet = true;
	vcopy(m_spos, p);

	if (m_toolMode == TOOLMODE_CREATE_TILES)
		removeTile(m_spos);
	else
		toolRecalc();
}

void BuilderTiledMesh::setToolEndPos(const float* p)
{
	if (!m_navMesh)
		return;
		
	m_eposSet = true;
	vcopy(m_epos, p);
	
	if (m_toolMode == TOOLMODE_CREATE_TILES)
		buildTile(m_epos);
	else
		toolRecalc();
}

bool BuilderTiledMesh::handleBuild()
{
	if (!m_verts || !m_tris)
	{
		printf("No verts or tris\n");
		return false;
	}

	delete m_navMesh;
	m_navMesh = new dtTiledNavMesh;
	if (!m_navMesh)
	{
		printf("Could not allocate navmehs\n");
		return false;
	}
	if (!m_navMesh->init(m_bmin, m_tileSize*m_cellSize, m_agentMaxClimb*m_cellHeight))
	{
		printf("Could not init navmesh\n");
		return false;
	}
		
	// Build chunky mesh.
	delete m_chunkyMesh;
	m_chunkyMesh = new rcChunkyTriMesh;
	if (!m_chunkyMesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
		return false;
	}
	if (!rcCreateChunkyTriMesh(m_verts, m_tris, m_ntris, 256, m_chunkyMesh))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Could not build chunky mesh.");
		return false;
	}

	return true;
}

void BuilderTiledMesh::buildTile(const float* pos)
{
	if (!m_navMesh)
		return;

	const float ts = m_tileSize*m_cellSize;
	const int tx = (int)floorf((pos[0]-m_bmin[0]) / ts);
	const int ty = (int)floorf((pos[2]-m_bmin[2]) / ts);
	if (tx < 0 || ty < 0)
		return;

	m_tileBmin[0] = m_bmin[0] + tx*ts;
	m_tileBmin[1] = m_bmin[1];
	m_tileBmin[2] = m_bmin[2] + ty*ts;

	m_tileBmax[0] = m_bmin[0] + (tx+1)*ts;
	m_tileBmax[1] = m_bmax[1];
	m_tileBmax[2] = m_bmin[2] + (ty+1)*ts;
	
	m_tileCol[0] = 0.2f; m_tileCol[1] = 1; m_tileCol[2] = 0; m_tileCol[3] = 1;
	
	int dataSize = 0;
	unsigned char* data = buildTileMesh(m_tileBmin, m_tileBmax, dataSize);
	
	if (data)
		m_navMesh->addTile(tx,ty,data,dataSize);
}

void BuilderTiledMesh::removeTile(const float* pos)
{
	if (!m_navMesh)
		return;
	
	const float ts = m_tileSize*m_cellSize;
	const int tx = (int)floorf((pos[0]-m_bmin[0]) / ts);
	const int ty = (int)floorf((pos[2]-m_bmin[2]) / ts);
	
	m_tileBmin[0] = m_bmin[0] + tx*ts;
	m_tileBmin[1] = m_bmin[1];
	m_tileBmin[2] = m_bmin[2] + ty*ts;
	
	m_tileBmax[0] = m_bmin[0] + (tx+1)*ts;
	m_tileBmax[1] = m_bmax[1];
	m_tileBmax[2] = m_bmin[2] + (ty+1)*ts;
	
	m_tileCol[0] = 1; m_tileCol[1] = 0; m_tileCol[2] = 0; m_tileCol[3] = 1;
	
	m_navMesh->removeTile(tx,ty);
}

unsigned char* BuilderTiledMesh::buildTileMesh(const float* bmin, const float* bmax, int& dataSize)
{
	if (!m_verts || ! m_tris)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return 0;
	}
	
	cleanup();
	
	// Init build configuration from GUI
	memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.cs = m_cellSize;
	m_cfg.ch = m_cellHeight;
	m_cfg.walkableSlopeAngle = m_agentMaxSlope;
	m_cfg.walkableHeight = (int)ceilf(m_agentHeight / m_cfg.ch);
	m_cfg.walkableClimb = (int)ceilf(m_agentMaxClimb / m_cfg.ch);
	m_cfg.walkableRadius = (int)ceilf(m_agentRadius / m_cfg.cs);
	m_cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	m_cfg.maxSimplificationError = m_edgeMaxError;
	m_cfg.minRegionSize = (int)rcSqr(m_regionMinSize);
	m_cfg.mergeRegionSize = (int)rcSqr(m_regionMergeSize);
	m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	m_cfg.tileSize = (int)m_tileSize;
	m_cfg.borderSize = m_cfg.walkableRadius*2 + 2; // Reserve enough padding.
	m_cfg.width = m_cfg.tileSize + m_cfg.borderSize*2;
	m_cfg.height = m_cfg.tileSize + m_cfg.borderSize*2;
	
/*	if (m_cfg.maxVertsPerPoly == DT_VERTS_PER_POLYGON)
		m_drawMode = DRAWMODE_NAVMESH;
	else
		m_drawMode = DRAWMODE_POLYMESH;*/
	
	vcopy(m_cfg.bmin, bmin);
	vcopy(m_cfg.bmax, bmax);
	m_cfg.bmin[0] -= m_cfg.borderSize*m_cfg.cs;
	m_cfg.bmin[2] -= m_cfg.borderSize*m_cfg.cs;
	m_cfg.bmax[0] += m_cfg.borderSize*m_cfg.cs;
	m_cfg.bmax[2] += m_cfg.borderSize*m_cfg.cs;
	
	// Reset build times gathering.
	memset(&m_buildTimes, 0, sizeof(m_buildTimes));
	rcSetBuildTimes(&m_buildTimes);
	
	// Start the build process.	
	rcTimeVal totStartTime = rcGetPerformanceTimer();
	
	if (rcGetLog())
	{
		rcGetLog()->log(RC_LOG_PROGRESS, "Building navigation:");
		rcGetLog()->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);
		rcGetLog()->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", m_nverts/1000.0f, m_ntris/1000.0f);
	}
	
	// Allocate voxel heighfield where we rasterize our input data to.
	m_solid = new rcHeightfield;
	if (!m_solid)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(*m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}
	
	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	m_triflags = new unsigned char[m_chunkyMesh->maxTrisPerChunk];
	if (!m_triflags)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'triangleFlags' (%d).", m_chunkyMesh->maxTrisPerChunk);
		return 0;
	}
	

	float tbmin[2], tbmax[2];
	tbmin[0] = m_cfg.bmin[0];
	tbmin[1] = m_cfg.bmin[2];
	tbmax[0] = m_cfg.bmax[0];
	tbmax[1] = m_cfg.bmax[2];
	int cid[256];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksInRect(m_chunkyMesh, tbmin, tbmax, cid, 256);
	if (!ncid)
		return 0;

	m_tileTriCount = 0;

	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = m_chunkyMesh->nodes[cid[i]];
		const int* tris = &m_chunkyMesh->tris[node.i*3];
		const int ntris = node.n;

		m_tileTriCount += ntris;
		
		memset(m_triflags, 0, ntris*sizeof(unsigned char));
		rcMarkWalkableTriangles(m_cfg.walkableSlopeAngle,
								m_verts, m_nverts, tris, ntris, m_triflags);
		
		rcRasterizeTriangles(m_verts, m_nverts, tris, m_triflags, ntris, *m_solid);
	}
	
	if (!m_keepInterResults)
	{
		delete [] m_triflags;
		m_triflags = 0;
	}
	
	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	rcFilterLedgeSpans(m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
	rcFilterWalkableLowHeightSpans(m_cfg.walkableHeight, *m_solid);
	
	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	m_chf = new rcCompactHeightfield;
	if (!m_chf)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(m_cfg.walkableHeight, m_cfg.walkableClimb, RC_WALKABLE, *m_solid, *m_chf))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}
	
	if (!m_keepInterResults)
	{
		delete m_solid;
		m_solid = 0;
	}
	
	// Prepare for region partitioning, by calculating distance field along the walkable surface.
	if (!rcBuildDistanceField(*m_chf))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
		return 0;
	}
	
	// Partition the walkable surface into simple regions without holes.
	if (!rcBuildRegions(*m_chf, m_cfg.walkableRadius, m_cfg.borderSize, m_cfg.minRegionSize, m_cfg.mergeRegionSize))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
		return 0;
	}
	
	// Create contours.
	m_cset = new rcContourSet;
	if (!m_cset)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return 0;
	}
	if (!rcBuildContours(*m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return 0;
	}
	
	if (!m_keepInterResults)
	{
		delete m_chf;
		m_chf = 0;
	}
	
	// Build polygon navmesh from the contours.
	m_polyMesh = new rcPolyMesh;
	if (!m_polyMesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'polyMesh'.");
		return 0;
	}
	if (!rcBuildPolyMesh(*m_cset, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch, m_cfg.maxVertsPerPoly, *m_polyMesh))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return 0;
	}
	
	if (!m_keepInterResults)
	{
		delete m_cset;
		m_cset = 0;
	}
	
	unsigned char* navData = 0;
	int navDataSize = 0;
	if (m_cfg.maxVertsPerPoly == DT_TILE_VERTS_PER_POLYGON)
	{
		// Remove padding from the polymesh data.
		for (int i = 0; i < m_polyMesh->nverts; ++i)
		{
			unsigned short* v = &m_polyMesh->verts[i*3];
			v[0] -= (unsigned short)m_cfg.borderSize;
			v[2] -= (unsigned short)m_cfg.borderSize;
		}
	
		if (!dtCreateNavMeshTileData(m_polyMesh->verts, m_polyMesh->nverts,
									 m_polyMesh->polys, m_polyMesh->npolys, m_polyMesh->nvp,
									 bmin, bmax, m_cfg.cs, m_cfg.ch, m_cfg.tileSize, m_cfg.walkableClimb, &navData, &navDataSize))
		{
			if (rcGetLog())
				rcGetLog()->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return 0;
		}
	}
	m_tileMemUsage = navDataSize/1024.0f;
	
	rcTimeVal totEndTime = rcGetPerformanceTimer();
	
	// Show performance stats.
	if (rcGetLog())
	{
		const float pc = 100.0f / rcGetDeltaTimeUsec(totStartTime, totEndTime);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Rasterize: %.1fms (%.1f%%)", m_buildTimes.rasterizeTriangles/1000.0f, m_buildTimes.rasterizeTriangles*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Compact: %.1fms (%.1f%%)", m_buildTimes.buildCompact/1000.0f, m_buildTimes.buildCompact*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Border: %.1fms (%.1f%%)", m_buildTimes.filterBorder/1000.0f, m_buildTimes.filterBorder*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Walkable: %.1fms (%.1f%%)", m_buildTimes.filterWalkable/1000.0f, m_buildTimes.filterWalkable*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Reachable: %.1fms (%.1f%%)", m_buildTimes.filterMarkReachable/1000.0f, m_buildTimes.filterMarkReachable*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Distancefield: %.1fms (%.1f%%)", m_buildTimes.buildDistanceField/1000.0f, m_buildTimes.buildDistanceField*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - distance: %.1fms (%.1f%%)", m_buildTimes.buildDistanceFieldDist/1000.0f, m_buildTimes.buildDistanceFieldDist*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - blur: %.1fms (%.1f%%)", m_buildTimes.buildDistanceFieldBlur/1000.0f, m_buildTimes.buildDistanceFieldBlur*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Regions: %.1fms (%.1f%%)", m_buildTimes.buildRegions/1000.0f, m_buildTimes.buildRegions*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - watershed: %.1fms (%.1f%%)", m_buildTimes.buildRegionsReg/1000.0f, m_buildTimes.buildRegionsReg*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "    - expand: %.1fms (%.1f%%)", m_buildTimes.buildRegionsExp/1000.0f, m_buildTimes.buildRegionsExp*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "    - find catchment basins: %.1fms (%.1f%%)", m_buildTimes.buildRegionsFlood/1000.0f, m_buildTimes.buildRegionsFlood*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - filter: %.1fms (%.1f%%)", m_buildTimes.buildRegionsFilter/1000.0f, m_buildTimes.buildRegionsFilter*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Contours: %.1fms (%.1f%%)", m_buildTimes.buildContours/1000.0f, m_buildTimes.buildContours*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - trace: %.1fms (%.1f%%)", m_buildTimes.buildContoursTrace/1000.0f, m_buildTimes.buildContoursTrace*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - simplify: %.1fms (%.1f%%)", m_buildTimes.buildContoursSimplify/1000.0f, m_buildTimes.buildContoursSimplify*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Fixup contours: %.1fms (%.1f%%)", m_buildTimes.fixupContours/1000.0f, m_buildTimes.fixupContours*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Polymesh: %.1fms (%.1f%%)", m_buildTimes.buildPolymesh/1000.0f, m_buildTimes.buildPolymesh*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Polymesh: Verts:%d  Polys:%d", m_polyMesh->nverts, m_polyMesh->npolys);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "TOTAL: %.1fms", rcGetDeltaTimeUsec(totStartTime, totEndTime)/1000.0f);
	}
	
	m_tileBuildTime = rcGetDeltaTimeUsec(totStartTime, totEndTime)/1000.0f;
	
	dataSize = navDataSize;
	return navData;
}
