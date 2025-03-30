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

#include "Sample_TileMesh.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif

#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "imgui.h"
#include "InputGeom.h"
#include "NavMeshPruneTool.h"
#include "NavMeshTesterTool.h"
#include "OffMeshConnectionTool.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "Sample.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

namespace
{
	unsigned int nextPow2(unsigned int v)
	{
		v--;
		v |= v >> 1;
		v |= v >> 2;
		v |= v >> 4;
		v |= v >> 8;
		v |= v >> 16;
		v++;
		return v;
	}

	unsigned int ilog2(unsigned int v)
	{
		unsigned int r = (v > 0xffff) << 4; v >>= r;
		unsigned int shift = (v > 0xff) << 3; v >>= shift; r |= shift;
		shift = (v > 0xf) << 2; v >>= shift; r |= shift;
		shift = (v > 0x3) << 1; v >>= shift; r |= shift;
		r |= (v >> 1);
		return r;
	}
}

class NavMeshTileTool : public SampleTool
{
	Sample_TileMesh* m_sample = nullptr;
	float m_hitPos[3] = {0, 0, 0};
	bool m_hitPosSet = false;

public:
	~NavMeshTileTool() override = default;

	int type() override { return TOOL_TILE_EDIT; }
	void init(Sample* sample) override { m_sample = (Sample_TileMesh*)sample; }
	void reset() override {}

	void handleMenu() override
	{
		imguiLabel("Create Tiles");
		if (imguiButton("Create All"))
		{
			if (m_sample)
				m_sample->buildAllTiles();
		}
		if (imguiButton("Remove All"))
		{
			if (m_sample)
				m_sample->removeAllTiles();
		}
	}

	void handleClick(const float* /*s*/, const float* p, bool shift) override
	{
		m_hitPosSet = true;
		rcVcopy(m_hitPos,p);
		if (m_sample)
		{
			if (shift)
				m_sample->removeTile(m_hitPos);
			else
				m_sample->buildTile(m_hitPos);
		}
	}

	void handleToggle() override {}

	void handleStep() override {}

	void handleUpdate(const float /*dt*/) override {}

	void handleRender() override
	{
		if (!m_hitPosSet)
		{
			return;
		}

		const float s = m_sample->getAgentRadius();
		glColor4ub(0,0,0,128);
		glLineWidth(2.0f);
		glBegin(GL_LINES);
		glVertex3f(m_hitPos[0]-s,m_hitPos[1]+0.1f,m_hitPos[2]);
		glVertex3f(m_hitPos[0]+s,m_hitPos[1]+0.1f,m_hitPos[2]);
		glVertex3f(m_hitPos[0],m_hitPos[1]-s+0.1f,m_hitPos[2]);
		glVertex3f(m_hitPos[0],m_hitPos[1]+s+0.1f,m_hitPos[2]);
		glVertex3f(m_hitPos[0],m_hitPos[1]+0.1f,m_hitPos[2]-s);
		glVertex3f(m_hitPos[0],m_hitPos[1]+0.1f,m_hitPos[2]+s);
		glEnd();
		glLineWidth(1.0f);
	}

	void handleRenderOverlay(double* proj, double* model, int* view) override
	{
		GLdouble x, y, z;
		if (m_hitPosSet && gluProject(m_hitPos[0], m_hitPos[1], m_hitPos[2], model, proj, view, &x, &y, &z))
		{
			int tx=0, ty=0;
			m_sample->getTilePos(m_hitPos, tx, ty);
			char text[32];
			snprintf(text,32,"(%d,%d)", tx,ty);
			imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
		}

		// Tool help
		const int h = view[3];
		imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB: Rebuild hit tile.  Shift+LMB: Clear hit tile.", imguiRGBA(255,255,255,192));
	}
};

Sample_TileMesh::Sample_TileMesh()
{
	resetCommonSettings();
	setTool(new NavMeshTileTool);
}

Sample_TileMesh::~Sample_TileMesh()
{
	cleanup();
	dtFreeNavMesh(m_navMesh); m_navMesh = 0;
}

void Sample_TileMesh::cleanup()
{
	delete [] m_triareas; m_triareas = 0;
	rcFreeHeightField(m_heightfield); m_heightfield = 0;
	rcFreeCompactHeightfield(m_compactHeightfield); m_compactHeightfield = 0;
	rcFreeContourSet(m_contourSet); m_contourSet = 0;
	rcFreePolyMesh(m_polyMesh); m_polyMesh = 0;
	rcFreePolyMeshDetail(m_detailPolyMesh); m_detailPolyMesh = 0;
}

void Sample_TileMesh::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_keepIntermediateResults)) { m_keepIntermediateResults = !m_keepIntermediateResults; }
	if (imguiCheck("Build All Tiles", m_buildAll)) { m_buildAll = !m_buildAll; }

	imguiLabel("Tiling");
	imguiSlider("TileSize", &m_tileSize, 16.0f, 1024.0f, 16.0f);

	if (m_geom)
	{
		char text[64];
		int gw = 0, gh = 0;
		const float* bmin = m_geom->getNavMeshBoundsMin();
		const float* bmax = m_geom->getNavMeshBoundsMax();
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
		const int ts = (int)m_tileSize;
		const int tw = (gw + ts-1) / ts;
		const int th = (gh + ts-1) / ts;
		snprintf(text, 64, "Tiles  %d x %d", tw, th);
		imguiValue(text);

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)ilog2(nextPow2(tw*th)), 14);
		if (tileBits > 14) { tileBits = 14; }
		int polyBits = 22 - tileBits;
		m_maxTiles = 1 << tileBits;
		m_maxPolysPerTile = 1 << polyBits;
		snprintf(text, 64, "Max Tiles  %d", m_maxTiles);
		imguiValue(text);
		snprintf(text, 64, "Max Polys  %d", m_maxPolysPerTile);
		imguiValue(text);
	}
	else
	{
		m_maxTiles = 0;
		m_maxPolysPerTile = 0;
	}

	imguiSeparator();

	imguiIndent();
	imguiIndent();

	if (imguiButton("Save"))
	{
		Sample::saveAll("all_tiles_navmesh.bin", m_navMesh);
	}

	if (imguiButton("Load"))
	{
		dtFreeNavMesh(m_navMesh);
		m_navMesh = Sample::loadAll("all_tiles_navmesh.bin");
		m_navQuery->init(m_navMesh, 2048);
	}

	imguiUnindent();
	imguiUnindent();

	char msg[64];
	snprintf(msg, 64, "Build Time: %.1fms", m_totalBuildTimeMs);
	imguiLabel(msg);

	imguiSeparator();
	imguiSeparator();
}

void Sample_TileMesh::handleTools()
{
	const int type = !m_tool ? TOOL_NONE : m_tool->type();

	if (imguiCheck("Test Navmesh", type == TOOL_NAVMESH_TESTER)) { setTool(new NavMeshTesterTool); }
	if (imguiCheck("Prune Navmesh", type == TOOL_NAVMESH_PRUNE)) { setTool(new NavMeshPruneTool); }
	if (imguiCheck("Create Tiles", type == TOOL_TILE_EDIT)) { setTool(new NavMeshTileTool); }
	if (imguiCheck("Create Off-Mesh Links", type == TOOL_OFFMESH_CONNECTION)) { setTool(new OffMeshConnectionTool); }
	if (imguiCheck("Create Convex Volumes", type == TOOL_CONVEX_VOLUME)) { setTool(new ConvexVolumeTool); }
	if (imguiCheck("Create Crowds", type == TOOL_CROWD)) { setTool(new CrowdTool); }

	imguiSeparatorLine();

	imguiIndent();

	if (m_tool)
	{
		m_tool->handleMenu();
	}

	imguiUnindent();
}

void Sample_TileMesh::UI_DrawModeOption(const char* name, DrawMode drawMode, bool enabled)
{
	if (imguiCheck(name, m_drawMode == drawMode, enabled))
	{
		m_drawMode = drawMode;
	}
}

void Sample_TileMesh::handleDebugMode()
{
	imguiLabel("Draw");
	UI_DrawModeOption("Input Mesh", DRAWMODE_MESH, true);
	UI_DrawModeOption("Navmesh", DRAWMODE_NAVMESH, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh Invis", DRAWMODE_NAVMESH_INVIS, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh Trans", DRAWMODE_NAVMESH_TRANS, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh BVTree", DRAWMODE_NAVMESH_BVTREE, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh Nodes", DRAWMODE_NAVMESH_NODES, m_navQuery != nullptr);
	UI_DrawModeOption("Navmesh Portals", DRAWMODE_NAVMESH_PORTALS, m_navMesh != nullptr);
	UI_DrawModeOption("Voxels", DRAWMODE_VOXELS, m_heightfield != nullptr);
	UI_DrawModeOption("Walkable Voxels", DRAWMODE_VOXELS_WALKABLE, m_heightfield != nullptr);
	UI_DrawModeOption("Compact", DRAWMODE_COMPACT, m_compactHeightfield != nullptr);
	UI_DrawModeOption("Compact Distance", DRAWMODE_COMPACT_DISTANCE, m_compactHeightfield != nullptr);
	UI_DrawModeOption("Compact Regions", DRAWMODE_COMPACT_REGIONS, m_compactHeightfield != nullptr);
	UI_DrawModeOption("Region Connections", DRAWMODE_REGION_CONNECTIONS, m_contourSet != nullptr);
	UI_DrawModeOption("Raw Contours", DRAWMODE_RAW_CONTOURS, m_contourSet != nullptr);
	UI_DrawModeOption("Both Contours", DRAWMODE_BOTH_CONTOURS, m_contourSet != nullptr);
	UI_DrawModeOption("Contours", DRAWMODE_CONTOURS, m_contourSet != nullptr);
	UI_DrawModeOption("Poly Mesh", DRAWMODE_POLYMESH, m_polyMesh != nullptr);
	UI_DrawModeOption("Poly Mesh Detail", DRAWMODE_POLYMESH_DETAIL, m_detailPolyMesh != nullptr);
}

void Sample_TileMesh::handleRender()
{
	if (!m_geom || !m_geom->getMesh())
	{
		return;
	}

	const float texScale = 1.0f / (m_cellSize * 10.0f);

	// Draw mesh
	if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(&m_dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
								m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(),
								m_agentMaxSlope, texScale);
		m_geom->drawOffMeshConnections(&m_dd);
	}

	glDepthMask(GL_FALSE);

	// Draw bounds
	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	duDebugDrawBoxWire(&m_dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);

	// Tiling grid.
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int tw = (gw + (int)m_tileSize-1) / (int)m_tileSize;
	const int th = (gh + (int)m_tileSize-1) / (int)m_tileSize;
	const float s = m_tileSize*m_cellSize;
	duDebugDrawGridXZ(&m_dd, bmin[0],bmin[1],bmin[2], tw,th, s, duRGBA(0,0,0,64), 1.0f);

	// Draw active tile
	duDebugDrawBoxWire(&m_dd, m_lastBuiltTileBmin[0],m_lastBuiltTileBmin[1],m_lastBuiltTileBmin[2],
					   m_lastBuiltTileBmax[0],m_lastBuiltTileBmax[1],m_lastBuiltTileBmax[2], m_tileCol, 1.0f);

	if (m_navMesh && m_navQuery &&
		(m_drawMode == DRAWMODE_NAVMESH ||
		 m_drawMode == DRAWMODE_NAVMESH_TRANS ||
		 m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
		 m_drawMode == DRAWMODE_NAVMESH_NODES ||
		 m_drawMode == DRAWMODE_NAVMESH_PORTALS ||
		 m_drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
		{
			duDebugDrawNavMeshWithClosedList(&m_dd, *m_navMesh, *m_navQuery, m_navMeshDrawFlags);
		}
		if (m_drawMode == DRAWMODE_NAVMESH_BVTREE)
		{
			duDebugDrawNavMeshBVTree(&m_dd, *m_navMesh);
		}
		if (m_drawMode == DRAWMODE_NAVMESH_PORTALS)
		{
			duDebugDrawNavMeshPortals(&m_dd, *m_navMesh);
		}
		if (m_drawMode == DRAWMODE_NAVMESH_NODES)
		{
			duDebugDrawNavMeshNodes(&m_dd, *m_navQuery);
		}
		duDebugDrawNavMeshPolysWithFlags(&m_dd, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
	}


	glDepthMask(GL_TRUE);

	if (m_compactHeightfield && m_drawMode == DRAWMODE_COMPACT)
	{
		duDebugDrawCompactHeightfieldSolid(&m_dd, *m_compactHeightfield);
	}

	if (m_compactHeightfield && m_drawMode == DRAWMODE_COMPACT_DISTANCE)
	{
		duDebugDrawCompactHeightfieldDistance(&m_dd, *m_compactHeightfield);
	}
	if (m_compactHeightfield && m_drawMode == DRAWMODE_COMPACT_REGIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_compactHeightfield);
	}
	if (m_heightfield && m_drawMode == DRAWMODE_VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&m_dd, *m_heightfield);
		glDisable(GL_FOG);
	}
	if (m_heightfield && m_drawMode == DRAWMODE_VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&m_dd, *m_heightfield);
		glDisable(GL_FOG);
	}

	if (m_contourSet && m_drawMode == DRAWMODE_RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_contourSet);
		glDepthMask(GL_TRUE);
	}

	if (m_contourSet && m_drawMode == DRAWMODE_BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_contourSet, 0.5f);
		duDebugDrawContours(&m_dd, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_contourSet && m_drawMode == DRAWMODE_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&m_dd, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_compactHeightfield && m_contourSet && m_drawMode == DRAWMODE_REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_compactHeightfield);

		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&m_dd, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_polyMesh && m_drawMode == DRAWMODE_POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&m_dd, *m_polyMesh);
		glDepthMask(GL_TRUE);
	}
	if (m_detailPolyMesh && m_drawMode == DRAWMODE_POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&m_dd, *m_detailPolyMesh);
		glDepthMask(GL_TRUE);
	}

	m_geom->drawConvexVolumes(&m_dd);

	if (m_tool)
	{
		m_tool->handleRender();
	}
	renderToolStates();

	glDepthMask(GL_TRUE);
}

void Sample_TileMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;

	// Draw start and end point labels
	if (m_tileBuildTime > 0.0f && gluProject((GLdouble)(m_lastBuiltTileBmin[0]+m_lastBuiltTileBmax[0])/2, (GLdouble)(m_lastBuiltTileBmin[1]+m_lastBuiltTileBmax[1])/2, (GLdouble)(m_lastBuiltTileBmin[2]+m_lastBuiltTileBmax[2])/2,
											 model, proj, view, &x, &y, &z))
	{
		char text[32];
		snprintf(text,32,"%.3fms / %dTris / %.1fkB", m_tileBuildTime, m_tileTriCount, m_tileMemUsage);
		imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
	}

	if (m_tool)
	{
		m_tool->handleRenderOverlay(proj, model, view);
	}
	renderOverlayToolStates(proj, model, view);
}

void Sample_TileMesh::handleMeshChanged(InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

	const BuildSettings* buildSettings = geom->getBuildSettings();
	if (buildSettings && buildSettings->tileSize > 0)
		m_tileSize = buildSettings->tileSize;

	cleanup();

	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

bool Sample_TileMesh::handleBuild()
{
	if (!m_geom || !m_geom->getMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return false;
	}

	dtFreeNavMesh(m_navMesh);

	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return false;
	}

	dtNavMeshParams params;
	rcVcopy(params.orig, m_geom->getNavMeshBoundsMin());
	params.tileWidth = m_tileSize*m_cellSize;
	params.tileHeight = m_tileSize*m_cellSize;
	params.maxTiles = m_maxTiles;
	params.maxPolys = m_maxPolysPerTile;

	dtStatus status;

	status = m_navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return false;
	}

	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return false;
	}

	if (m_buildAll)
	{
		buildAllTiles();
	}

	if (m_tool)
	{
		m_tool->init(this);
	}
	initToolStates(this);

	return true;
}

void Sample_TileMesh::collectSettings(BuildSettings& settings)
{
	Sample::collectSettings(settings);

	settings.tileSize = m_tileSize;
}

void Sample_TileMesh::buildTile(const float* pos)
{
	if (!m_geom) { return; }
	if (!m_navMesh) { return; }

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();

	const float ts = m_tileSize*m_cellSize;
	const int tx = (int)((pos[0] - bmin[0]) / ts);
	const int ty = (int)((pos[2] - bmin[2]) / ts);

	m_lastBuiltTileBmin[0] = bmin[0] + tx*ts;
	m_lastBuiltTileBmin[1] = bmin[1];
	m_lastBuiltTileBmin[2] = bmin[2] + ty*ts;

	m_lastBuiltTileBmax[0] = bmin[0] + (tx+1)*ts;
	m_lastBuiltTileBmax[1] = bmax[1];
	m_lastBuiltTileBmax[2] = bmin[2] + (ty+1)*ts;

	m_tileCol = duRGBA(255,255,255,64);

	m_ctx->resetLog();

	int dataSize = 0;
	unsigned char* data = buildTileMesh(tx, ty, m_lastBuiltTileBmin, m_lastBuiltTileBmax, dataSize);

	// Remove any previous data (navmesh owns and deletes the data).
	m_navMesh->removeTile(m_navMesh->getTileRefAt(tx,ty,0),0,0);

	// Add tile, or leave the location empty.
	if (data)
	{
		// Let the navmesh own the data.
		dtStatus status = m_navMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
		if (dtStatusFailed(status))
			dtFree(data);
	}

	m_ctx->dumpLog("Build Tile (%d,%d):", tx,ty);
}

void Sample_TileMesh::getTilePos(const float* pos, int& tx, int& ty)
{
	if (!m_geom) { return; }

	const float* bmin = m_geom->getNavMeshBoundsMin();

	const float ts = m_tileSize*m_cellSize;
	tx = static_cast<int>((pos[0] - bmin[0]) / ts);
	ty = static_cast<int>((pos[2] - bmin[2]) / ts);
}

void Sample_TileMesh::removeTile(const float* pos)
{
	if (!m_geom) { return; }
	if (!m_navMesh) { return; }

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();

	const float ts = m_tileSize * m_cellSize;
	const int tx = static_cast<int>((pos[0] - bmin[0]) / ts);
	const int ty = static_cast<int>((pos[2] - bmin[2]) / ts);

	m_lastBuiltTileBmin[0] = bmin[0] + tx * ts;
	m_lastBuiltTileBmin[1] = bmin[1];
	m_lastBuiltTileBmin[2] = bmin[2] + ty * ts;

	m_lastBuiltTileBmax[0] = bmin[0] + (tx + 1) * ts;
	m_lastBuiltTileBmax[1] = bmax[1];
	m_lastBuiltTileBmax[2] = bmin[2] + (ty + 1) * ts;

	m_tileCol = duRGBA(128, 32, 16, 64);

	m_navMesh->removeTile(m_navMesh->getTileRefAt(tx,ty,0),0,0);
}

void Sample_TileMesh::buildAllTiles()
{
	if (!m_geom) { return; }
	if (!m_navMesh) { return; }

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;
	const float tcs = m_tileSize*m_cellSize;

	// Start the build process.
	m_ctx->startTimer(RC_TIMER_TEMP);

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			m_lastBuiltTileBmin[0] = bmin[0] + x*tcs;
			m_lastBuiltTileBmin[1] = bmin[1];
			m_lastBuiltTileBmin[2] = bmin[2] + y*tcs;

			m_lastBuiltTileBmax[0] = bmin[0] + (x+1)*tcs;
			m_lastBuiltTileBmax[1] = bmax[1];
			m_lastBuiltTileBmax[2] = bmin[2] + (y+1)*tcs;

			int dataSize = 0;
			unsigned char* data = buildTileMesh(x, y, m_lastBuiltTileBmin, m_lastBuiltTileBmax, dataSize);
			if (data)
			{
				// Remove any previous data (navmesh owns and deletes the data).
				m_navMesh->removeTile(m_navMesh->getTileRefAt(x,y,0),0,0);
				// Let the navmesh own the data.
				dtStatus status = m_navMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
				if (dtStatusFailed(status))
				{
					dtFree(data);
				}
			}
		}
	}

	// Start the build process.
	m_ctx->stopTimer(RC_TIMER_TEMP);

	m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TEMP)/1000.0f;

}

void Sample_TileMesh::removeAllTiles() const
{
	if (!m_geom || !m_navMesh) { return; }

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			m_navMesh->removeTile(m_navMesh->getTileRefAt(x,y,0),0,0);
		}
	}
}

unsigned char* Sample_TileMesh::buildTileMesh(const int tx, const int ty, const float* bmin, const float* bmax, int& dataSize)
{
	if (!m_geom || !m_geom->getMesh() || !m_geom->getChunkyMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return 0;
	}

	m_tileMemUsage = 0;
	m_tileBuildTime = 0;

	cleanup();

	const float* verts = m_geom->getMesh()->getVerts();
	const int nverts = m_geom->getMesh()->getVertCount();
	const int ntris = m_geom->getMesh()->getTriCount();
	const rcChunkyTriMesh* chunkyMesh = m_geom->getChunkyMesh();

	// Init build configuration from GUI
	memset(&m_config, 0, sizeof(m_config));
	m_config.cs = m_cellSize;
	m_config.ch = m_cellHeight;
	m_config.walkableSlopeAngle = m_agentMaxSlope;
	m_config.walkableHeight = (int)ceilf(m_agentHeight / m_config.ch);
	m_config.walkableClimb = (int)floorf(m_agentMaxClimb / m_config.ch);
	m_config.walkableRadius = (int)ceilf(m_agentRadius / m_config.cs);
	m_config.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	m_config.maxSimplificationError = m_edgeMaxError;
	m_config.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	m_config.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	m_config.maxVertsPerPoly = (int)m_vertsPerPoly;
	m_config.tileSize = (int)m_tileSize;
	m_config.borderSize = m_config.walkableRadius + 3; // Reserve enough padding.
	m_config.width = m_config.tileSize + m_config.borderSize*2;
	m_config.height = m_config.tileSize + m_config.borderSize*2;
	m_config.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_config.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

	// Expand the heighfield bounding box by border size to find the extents of geometry we need to build this tile.
	//
	// This is done in order to make sure that the navmesh tiles connect correctly at the borders,
	// and the obstacles close to the border work correctly with the dilation process.
	// No polygons (or contours) will be created on the border area.
	//
	// IMPORTANT!
	//
	//   :''''''''':
	//   : +-----+ :
	//   : |     | :
	//   : |     |<--- tile to build
	//   : |     | :
	//   : +-----+ :<-- geometry needed
	//   :.........:
	//
	// You should use this bounding box to query your input geometry.
	//
	// For example if you build a navmesh for terrain, and want the navmesh tiles to match the terrain tile size
	// you will need to pass in data from neighbour terrain tiles too! In a simple case, just pass in all the 8 neighbours,
	// or use the bounding box below to only pass in a sliver of each of the 8 neighbours.
	rcVcopy(m_config.bmin, bmin);
	rcVcopy(m_config.bmax, bmax);
	m_config.bmin[0] -= m_config.borderSize*m_config.cs;
	m_config.bmin[2] -= m_config.borderSize*m_config.cs;
	m_config.bmax[0] += m_config.borderSize*m_config.cs;
	m_config.bmax[2] += m_config.borderSize*m_config.cs;

	// Reset build times gathering.
	m_ctx->resetTimers();

	// Start the build process.
	m_ctx->startTimer(RC_TIMER_TOTAL);

	m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
	m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_config.width, m_config.height);
	m_ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts/1000.0f, ntris/1000.0f);

	// Allocate voxel heightfield where we rasterize our input data to.
	m_heightfield = rcAllocHeightfield();
	if (!m_heightfield)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(m_ctx, *m_heightfield, m_config.width, m_config.height, m_config.bmin, m_config.bmax, m_config.cs, m_config.ch))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}

	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	m_triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!m_triareas)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
		return 0;
	}

	float tbmin[2], tbmax[2];
	tbmin[0] = m_config.bmin[0];
	tbmin[1] = m_config.bmin[2];
	tbmax[0] = m_config.bmax[0];
	tbmax[1] = m_config.bmax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid)
	{
		return 0;
	}

	m_tileTriCount = 0;

	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* ctris = &chunkyMesh->tris[node.i*3];
		const int nctris = node.n;

		m_tileTriCount += nctris;

		memset(m_triareas, 0, nctris*sizeof(unsigned char));
		rcMarkWalkableTriangles(m_ctx, m_config.walkableSlopeAngle,
								verts, nverts, ctris, nctris, m_triareas);

		if (!rcRasterizeTriangles(m_ctx, verts, nverts, ctris, m_triareas, nctris, *m_heightfield, m_config.walkableClimb))
		{
			return 0;
		}
	}

	if (!m_keepIntermediateResults)
	{
		delete [] m_triareas;
		m_triareas = 0;
	}

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (m_filterLowHangingObstacles)
	{
		rcFilterLowHangingWalkableObstacles(m_ctx, m_config.walkableClimb, *m_heightfield);
	}
	if (m_filterLedgeSpans)
	{
		rcFilterLedgeSpans(m_ctx, m_config.walkableHeight, m_config.walkableClimb, *m_heightfield);
	}
	if (m_filterWalkableLowHeightSpans)
	{
		rcFilterWalkableLowHeightSpans(m_ctx, m_config.walkableHeight, *m_heightfield);
	}

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	m_compactHeightfield = rcAllocCompactHeightfield();
	if (!m_compactHeightfield)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(m_ctx, m_config.walkableHeight, m_config.walkableClimb, *m_heightfield, *m_compactHeightfield))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}

	if (!m_keepIntermediateResults)
	{
		rcFreeHeightField(m_heightfield);
		m_heightfield = 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_ctx, m_config.walkableRadius, *m_compactHeightfield))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}

	// (Optional) Mark areas.
	const ConvexVolume* vols = m_geom->getConvexVolumes();
	for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
	{
		rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_compactHeightfield);
	}


	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	// There are 3 martitioning methods, each with some pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or overlaps
	//   - the are some corner cases where this method creates produces holes and overlaps
	//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
	// 2) Monotone partioning
	//   - fastest
	//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
	//   - creates long thin polygons, which sometimes causes paths with detours
	//   * use this if you want fast navmesh generation
	// 3) Layer partitoining
	//   - quite fast
	//   - partitions the heighfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a bit ugly tessellation (still better than monotone)
	//     if you have large open areas with small obstacles (not a problem if you use tiles)
	//   * good choice to use for tiled navmesh with medium and small sized tiles

	if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(m_ctx, *m_compactHeightfield))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return 0;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(m_ctx, *m_compactHeightfield, m_config.borderSize, m_config.minRegionArea, m_config.mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return 0;
		}
	}
	else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(m_ctx, *m_compactHeightfield, m_config.borderSize, m_config.minRegionArea, m_config.mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return 0;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(m_ctx, *m_compactHeightfield, m_config.borderSize, m_config.minRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return 0;
		}
	}

	// Create contours.
	m_contourSet = rcAllocContourSet();
	if (!m_contourSet)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return 0;
	}
	if (!rcBuildContours(m_ctx, *m_compactHeightfield, m_config.maxSimplificationError, m_config.maxEdgeLen, *m_contourSet))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return 0;
	}

	if (m_contourSet->nconts == 0)
	{
		return 0;
	}

	// Build polygon navmesh from the contours.
	m_polyMesh = rcAllocPolyMesh();
	if (!m_polyMesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return 0;
	}
	if (!rcBuildPolyMesh(m_ctx, *m_contourSet, m_config.maxVertsPerPoly, *m_polyMesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return 0;
	}

	// Build detail mesh.
	m_detailPolyMesh = rcAllocPolyMeshDetail();
	if (!m_detailPolyMesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'dmesh'.");
		return 0;
	}

	if (!rcBuildPolyMeshDetail(m_ctx, *m_polyMesh, *m_compactHeightfield,
							   m_config.detailSampleDist, m_config.detailSampleMaxError,
							   *m_detailPolyMesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could build polymesh detail.");
		return 0;
	}

	if (!m_keepIntermediateResults)
	{
		rcFreeCompactHeightfield(m_compactHeightfield);
		m_compactHeightfield = 0;
		rcFreeContourSet(m_contourSet);
		m_contourSet = 0;
	}

	unsigned char* navData = 0;
	int navDataSize = 0;
	if (m_config.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (m_polyMesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			m_ctx->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", m_polyMesh->nverts, 0xffff);
			return 0;
		}

		// Update poly flags from areas.
		for (int i = 0; i < m_polyMesh->npolys; ++i)
		{
			if (m_polyMesh->areas[i] == RC_WALKABLE_AREA)
			{
				m_polyMesh->areas[i] = SAMPLE_POLYAREA_GROUND;
			}

			if (m_polyMesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
			    m_polyMesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
			    m_polyMesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				m_polyMesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (m_polyMesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				m_polyMesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (m_polyMesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				m_polyMesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_polyMesh->verts;
		params.vertCount = m_polyMesh->nverts;
		params.polys = m_polyMesh->polys;
		params.polyAreas = m_polyMesh->areas;
		params.polyFlags = m_polyMesh->flags;
		params.polyCount = m_polyMesh->npolys;
		params.nvp = m_polyMesh->nvp;
		params.detailMeshes = m_detailPolyMesh->meshes;
		params.detailVerts = m_detailPolyMesh->verts;
		params.detailVertsCount = m_detailPolyMesh->nverts;
		params.detailTris = m_detailPolyMesh->tris;
		params.detailTriCount = m_detailPolyMesh->ntris;
		params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
		params.offMeshConRad = m_geom->getOffMeshConnectionRads();
		params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
		params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
		params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
		params.offMeshConUserID = m_geom->getOffMeshConnectionId();
		params.offMeshConCount = m_geom->getOffMeshConnectionCount();
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		params.tileX = tx;
		params.tileY = ty;
		params.tileLayer = 0;
		rcVcopy(params.bmin, m_polyMesh->bmin);
		rcVcopy(params.bmax, m_polyMesh->bmax);
		params.cs = m_config.cs;
		params.ch = m_config.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return 0;
		}
	}
	m_tileMemUsage = navDataSize/1024.0f;

	m_ctx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_polyMesh->nverts, m_polyMesh->npolys);

	m_tileBuildTime = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;

	dataSize = navDataSize;
	return navData;
}
