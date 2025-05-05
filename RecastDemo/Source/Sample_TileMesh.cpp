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
	float m_hitPos[3] = { 0, 0, 0 };
	bool m_hitPosSet = false;

public:
	~NavMeshTileTool() override = default;

	SampleToolType type() override { return SampleToolType::TILE_EDIT; }
	void init(Sample* sample) override { m_sample = static_cast<Sample_TileMesh*>(sample); }
	void reset() override {}

	void handleMenu() override
	{
		imguiLabel("Create Tiles");
		if (imguiButton("Create All"))
		{
			if (m_sample)
			{
				m_sample->buildAllTiles();
			}
		}
		if (imguiButton("Remove All"))
		{
			if (m_sample)
			{
				m_sample->removeAllTiles();
			}
		}
	}

	void handleClick(const float* /*s*/, const float* p, bool shift) override
	{
		m_hitPosSet = true;
		rcVcopy(m_hitPos, p);
		if (m_sample)
		{
			if (shift)
			{
				m_sample->removeTile(m_hitPos);
			}
			else
			{
				m_sample->buildTile(m_hitPos);
			}
		}
	}

	void handleToggle() override {}

	void handleStep() override {}

	void handleUpdate(const float /*dt*/) override {}

	void handleRender() override
	{
		if (!m_hitPosSet) { return; }

		const float s = m_sample->getAgentRadius();
		glColor4ub(0, 0, 0, 128);
		glLineWidth(2.0f);
		glBegin(GL_LINES);
		glVertex3f(m_hitPos[0] - s, m_hitPos[1] + 0.1f, m_hitPos[2]);
		glVertex3f(m_hitPos[0] + s, m_hitPos[1] + 0.1f, m_hitPos[2]);
		glVertex3f(m_hitPos[0], m_hitPos[1] - s + 0.1f, m_hitPos[2]);
		glVertex3f(m_hitPos[0], m_hitPos[1] + s + 0.1f, m_hitPos[2]);
		glVertex3f(m_hitPos[0], m_hitPos[1] + 0.1f, m_hitPos[2] - s);
		glVertex3f(m_hitPos[0], m_hitPos[1] + 0.1f, m_hitPos[2] + s);
		glEnd();
		glLineWidth(1.0f);
	}

	void handleRenderOverlay(double* proj, double* model, int* view) override
	{
		GLdouble x, y, z;
		if (m_hitPosSet && gluProject(m_hitPos[0], m_hitPos[1], m_hitPos[2], model, proj, view, &x, &y, &z))
		{
			int tx = 0, ty = 0;
			m_sample->getTilePos(m_hitPos, tx, ty);
			char text[32];
			snprintf(text, 32, "(%d,%d)", tx, ty);
			imguiDrawText(static_cast<int>(x), static_cast<int>(y) - 25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0, 0, 0, 220));
		}

		// Tool help
		const int h = view[3];
		imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Rebuild hit tile.  Shift+LMB: Clear hit tile.", imguiRGBA(255, 255, 255, 192));
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
	delete[] m_triareas; m_triareas = nullptr;
	rcFreeHeightField(m_heightfield); m_heightfield = nullptr;
	rcFreeCompactHeightfield(m_compactHeightfield); m_compactHeightfield = nullptr;
	rcFreeContourSet(m_contourSet); m_contourSet = nullptr;
	rcFreePolyMesh(m_polyMesh); m_polyMesh = nullptr;
	rcFreePolyMeshDetail(m_detailPolyMesh); m_detailPolyMesh = nullptr;
}

void Sample_TileMesh::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Build All Tiles", m_buildAll)) { m_buildAll = !m_buildAll; }

	imguiLabel("Tiling");
	imguiSlider("TileSize", &m_tileSize, 16.0f, 1024.0f, 16.0f);

	if (m_inputGeometry)
	{
		const float* navMeshBoundsMin = m_inputGeometry->getNavMeshBoundsMin();
		const float* navMeshBoundsMax = m_inputGeometry->getNavMeshBoundsMax();
		int gridWidth = 0;
		int gridHeight = 0;
		rcCalcGridSize(navMeshBoundsMin, navMeshBoundsMax, m_cellSize, &gridWidth, &gridHeight);
		const int tileSize = static_cast<int>(m_tileSize);
		const int tileWidth = (gridWidth + tileSize - 1) / tileSize;
		const int tileHeight = (gridHeight + tileSize - 1) / tileSize;

		char text[64];
		snprintf(text, 64, "Tiles  %d x %d", tileWidth, tileHeight);
		imguiValue(text);

		// Max tiles and max polys affect how the tile IDs are calculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)ilog2(nextPow2(tileWidth * tileHeight)), 14);
		tileBits = rcMin(tileBits, 14);
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
	const SampleToolType type = !m_tool ? SampleToolType::NONE : m_tool->type();

	if (imguiCheck("Test Navmesh", type == SampleToolType::NAVMESH_TESTER)) { setTool(new NavMeshTesterTool); }
	if (imguiCheck("Prune Navmesh", type == SampleToolType::NAVMESH_PRUNE)) { setTool(new NavMeshPruneTool); }
	if (imguiCheck("Create Tiles", type == SampleToolType::TILE_EDIT)) { setTool(new NavMeshTileTool); }
	if (imguiCheck("Create Off-Mesh Links", type == SampleToolType::OFFMESH_CONNECTION)) { setTool(new OffMeshConnectionTool); }
	if (imguiCheck("Create Convex Volumes", type == SampleToolType::CONVEX_VOLUME)) { setTool(new ConvexVolumeTool); }
	if (imguiCheck("Create Crowds", type == SampleToolType::CROWD)) { setTool(new CrowdTool); }

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
	UI_DrawModeOption("Input Mesh", DrawMode::MESH, true);
	UI_DrawModeOption("Navmesh", DrawMode::NAVMESH, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh Invis", DrawMode::NAVMESH_INVIS, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh Trans", DrawMode::NAVMESH_TRANS, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh BVTree", DrawMode::NAVMESH_BVTREE, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh Nodes", DrawMode::NAVMESH_NODES, m_navQuery != nullptr);
	UI_DrawModeOption("Navmesh Portals", DrawMode::NAVMESH_PORTALS, m_navMesh != nullptr);
	UI_DrawModeOption("Voxels", DrawMode::VOXELS, m_heightfield != nullptr);
	UI_DrawModeOption("Walkable Voxels", DrawMode::VOXELS_WALKABLE, m_heightfield != nullptr);
	UI_DrawModeOption("Compact", DrawMode::COMPACT, m_compactHeightfield != nullptr);
	UI_DrawModeOption("Compact Distance", DrawMode::COMPACT_DISTANCE, m_compactHeightfield != nullptr);
	UI_DrawModeOption("Compact Regions", DrawMode::COMPACT_REGIONS, m_compactHeightfield != nullptr);
	UI_DrawModeOption("Region Connections", DrawMode::REGION_CONNECTIONS, m_contourSet != nullptr);
	UI_DrawModeOption("Raw Contours", DrawMode::RAW_CONTOURS, m_contourSet != nullptr);
	UI_DrawModeOption("Both Contours", DrawMode::BOTH_CONTOURS, m_contourSet != nullptr);
	UI_DrawModeOption("Contours", DrawMode::CONTOURS, m_contourSet != nullptr);
	UI_DrawModeOption("Poly Mesh", DrawMode::POLYMESH, m_polyMesh != nullptr);
	UI_DrawModeOption("Poly Mesh Detail", DrawMode::POLYMESH_DETAIL, m_detailPolyMesh != nullptr);
}

void Sample_TileMesh::handleRender()
{
	if (!m_inputGeometry || !m_inputGeometry->getMesh()) { return; }

	const float texScale = 1.0f / (m_cellSize * 10.0f);

	// Draw mesh
	if (m_drawMode != DrawMode::NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(
			&m_debugDraw,
			m_inputGeometry->getMesh()->getVerts(),
			m_inputGeometry->getMesh()->getVertCount(),
			m_inputGeometry->getMesh()->getTris(),
			m_inputGeometry->getMesh()->getNormals(),
			m_inputGeometry->getMesh()->getTriCount(),
			m_agentMaxSlope,
			texScale);
		m_inputGeometry->drawOffMeshConnections(&m_debugDraw);
	}

	glDepthMask(GL_FALSE);

	// Draw bounds
	const float* navMeshBoundsMin = m_inputGeometry->getNavMeshBoundsMin();
	const float* navMeshBoundsMax = m_inputGeometry->getNavMeshBoundsMax();
	duDebugDrawBoxWire(&m_debugDraw,
		navMeshBoundsMin[0],
		navMeshBoundsMin[1],
		navMeshBoundsMin[2],
		navMeshBoundsMax[0],
		navMeshBoundsMax[1],
		navMeshBoundsMax[2],
		duRGBA(255, 255, 255, 128), 1.0f);

	// Tiling grid.
	int gridWith = 0;
	int gridHeight = 0;
	rcCalcGridSize(navMeshBoundsMin, navMeshBoundsMax, m_cellSize, &gridWith, &gridHeight);
	const int tileWidth = (gridWith + static_cast<int>(m_tileSize) - 1) / static_cast<int>(m_tileSize);
	const int tileHeight = (gridHeight + static_cast<int>(m_tileSize) - 1) / static_cast<int>(m_tileSize);
	const float size = m_tileSize * m_cellSize;
	duDebugDrawGridXZ(&m_debugDraw,
		navMeshBoundsMin[0],
		navMeshBoundsMin[1],
		navMeshBoundsMin[2],
		tileWidth,
		tileHeight,
		size,
		duRGBA(0, 0, 0, 64),
		1.0f);

	// Draw active tile
	duDebugDrawBoxWire(
		&m_debugDraw,
		m_lastBuiltTileBoundsMin[0],
		m_lastBuiltTileBoundsMin[1],
		m_lastBuiltTileBoundsMin[2],
		m_lastBuiltTileBoundsMax[0],
		m_lastBuiltTileBoundsMax[1],
		m_lastBuiltTileBoundsMax[2],
		m_tileColor,
		1.0f);

	if (m_navMesh && m_navQuery &&
	    (m_drawMode == DrawMode::NAVMESH ||
	     m_drawMode == DrawMode::NAVMESH_TRANS ||
	     m_drawMode == DrawMode::NAVMESH_BVTREE ||
	     m_drawMode == DrawMode::NAVMESH_NODES ||
	     m_drawMode == DrawMode::NAVMESH_PORTALS ||
	     m_drawMode == DrawMode::NAVMESH_INVIS))
	{
		if (m_drawMode != DrawMode::NAVMESH_INVIS)
		{
			duDebugDrawNavMeshWithClosedList(&m_debugDraw, *m_navMesh, *m_navQuery, m_navMeshDrawFlags);
		}
		if (m_drawMode == DrawMode::NAVMESH_BVTREE)
		{
			duDebugDrawNavMeshBVTree(&m_debugDraw, *m_navMesh);
		}
		if (m_drawMode == DrawMode::NAVMESH_PORTALS)
		{
			duDebugDrawNavMeshPortals(&m_debugDraw, *m_navMesh);
		}
		if (m_drawMode == DrawMode::NAVMESH_NODES)
		{
			duDebugDrawNavMeshNodes(&m_debugDraw, *m_navQuery);
		}
		duDebugDrawNavMeshPolysWithFlags(&m_debugDraw, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0, 0, 0, 128));
	}

	glDepthMask(GL_TRUE);

	if (m_compactHeightfield && m_drawMode == DrawMode::COMPACT)
	{
		duDebugDrawCompactHeightfieldSolid(&m_debugDraw, *m_compactHeightfield);
	}

	if (m_compactHeightfield && m_drawMode == DrawMode::COMPACT_DISTANCE)
	{
		duDebugDrawCompactHeightfieldDistance(&m_debugDraw, *m_compactHeightfield);
	}
	if (m_compactHeightfield && m_drawMode == DrawMode::COMPACT_REGIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&m_debugDraw, *m_compactHeightfield);
	}
	if (m_heightfield && m_drawMode == DrawMode::VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&m_debugDraw, *m_heightfield);
		glDisable(GL_FOG);
	}
	if (m_heightfield && m_drawMode == DrawMode::VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&m_debugDraw, *m_heightfield);
		glDisable(GL_FOG);
	}

	if (m_contourSet && m_drawMode == DrawMode::RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}

	if (m_contourSet && m_drawMode == DrawMode::BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_debugDraw, *m_contourSet, 0.5f);
		duDebugDrawContours(&m_debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_contourSet && m_drawMode == DrawMode::CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&m_debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_compactHeightfield && m_contourSet && m_drawMode == DrawMode::REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&m_debugDraw, *m_compactHeightfield);

		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&m_debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_polyMesh && m_drawMode == DrawMode::POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&m_debugDraw, *m_polyMesh);
		glDepthMask(GL_TRUE);
	}
	if (m_detailPolyMesh && m_drawMode == DrawMode::POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&m_debugDraw, *m_detailPolyMesh);
		glDepthMask(GL_TRUE);
	}

	m_inputGeometry->drawConvexVolumes(&m_debugDraw);

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
	const int projectResult = gluProject(
		static_cast<GLdouble>(m_lastBuiltTileBoundsMin[0] + m_lastBuiltTileBoundsMax[0]) / 2,
		static_cast<GLdouble>(m_lastBuiltTileBoundsMin[1] + m_lastBuiltTileBoundsMax[1]) / 2,
		static_cast<GLdouble>(m_lastBuiltTileBoundsMin[2] + m_lastBuiltTileBoundsMax[2]) / 2,
		model,
		proj,
		view,
		&x,
		&y,
		&z);
	if (m_tileBuildTime > 0.0f && projectResult == GL_TRUE)
	{
		char text[32];
		snprintf(text, 32, "%.3fms / %dTris / %.1fkB", m_tileBuildTime, m_tileTriCount, m_tileMemUsage);
		imguiDrawText(static_cast<int>(x), static_cast<int>(y) - 25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0, 0, 0, 220));
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
	{
		m_tileSize = buildSettings->tileSize;
	}

	cleanup();

	dtFreeNavMesh(m_navMesh); m_navMesh = nullptr;

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
	if (!m_inputGeometry || !m_inputGeometry->getMesh())
	{
		m_buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return false;
	}

	dtFreeNavMesh(m_navMesh);

	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return false;
	}

	dtNavMeshParams params;
	rcVcopy(params.orig, m_inputGeometry->getNavMeshBoundsMin());
	params.tileWidth = m_tileSize * m_cellSize;
	params.tileHeight = m_tileSize * m_cellSize;
	params.maxTiles = m_maxTiles;
	params.maxPolys = m_maxPolysPerTile;

	dtStatus status = m_navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return false;
	}

	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
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
	if (!m_inputGeometry) { return; }
	if (!m_navMesh) { return; }

	const float* navMeshBoundsMin = m_inputGeometry->getNavMeshBoundsMin();
	const float* navMeshBoundsMax = m_inputGeometry->getNavMeshBoundsMax();

	const float tileSize = m_tileSize * m_cellSize;
	const int tileX = static_cast<int>((pos[0] - navMeshBoundsMin[0]) / tileSize);
	const int tileY = static_cast<int>((pos[2] - navMeshBoundsMin[2]) / tileSize);

	m_lastBuiltTileBoundsMin[0] = navMeshBoundsMin[0] + static_cast<float>(tileX) * tileSize;
	m_lastBuiltTileBoundsMin[1] = navMeshBoundsMin[1];
	m_lastBuiltTileBoundsMin[2] = navMeshBoundsMin[2] + static_cast<float>(tileY) * tileSize;

	m_lastBuiltTileBoundsMax[0] = navMeshBoundsMin[0] + static_cast<float>(tileX + 1) * tileSize;
	m_lastBuiltTileBoundsMax[1] = navMeshBoundsMax[1];
	m_lastBuiltTileBoundsMax[2] = navMeshBoundsMin[2] + static_cast<float>(tileY + 1) * tileSize;

	m_tileColor = duRGBA(255, 255, 255, 64);

	m_buildContext->resetLog();

	int tileMeshDataSize = 0;
	unsigned char* tileMeshData = buildTileMesh(tileX, tileY, m_lastBuiltTileBoundsMin, m_lastBuiltTileBoundsMax, tileMeshDataSize);

	// Remove any previous data (navmesh owns and deletes the data).
	m_navMesh->removeTile(m_navMesh->getTileRefAt(tileX, tileY, 0), 0, 0);

	// Add tile, or leave the location empty.
	if (tileMeshData)
	{
		// Let the navmesh own the data.
		const dtStatus status = m_navMesh->addTile(tileMeshData, tileMeshDataSize, DT_TILE_FREE_DATA, 0, 0);
		if (dtStatusFailed(status))
		{
			dtFree(tileMeshData);
		}
	}

	m_buildContext->dumpLog("Build Tile (%d,%d):", tileX, tileY);
}

void Sample_TileMesh::getTilePos(const float* pos, int& tileX, int& tileY) const
{
	if (!m_inputGeometry) { return; }

	const float* navMeshBoundsMin = m_inputGeometry->getNavMeshBoundsMin();

	const float ts = m_tileSize * m_cellSize;
	tileX = static_cast<int>((pos[0] - navMeshBoundsMin[0]) / ts);
	tileY = static_cast<int>((pos[2] - navMeshBoundsMin[2]) / ts);
}

void Sample_TileMesh::removeTile(const float* pos)
{
	if (!m_inputGeometry) { return; }
	if (!m_navMesh) { return; }

	const float* navMeshBoundsMin = m_inputGeometry->getNavMeshBoundsMin();
	const float* navmeshBoundsMax = m_inputGeometry->getNavMeshBoundsMax();

	const float tileSize = m_tileSize * m_cellSize;
	const int tileX = static_cast<int>((pos[0] - navMeshBoundsMin[0]) / tileSize);
	const int tileY = static_cast<int>((pos[2] - navMeshBoundsMin[2]) / tileSize);

	m_lastBuiltTileBoundsMin[0] = navMeshBoundsMin[0] + static_cast<float>(tileX) * tileSize;
	m_lastBuiltTileBoundsMin[1] = navMeshBoundsMin[1];
	m_lastBuiltTileBoundsMin[2] = navMeshBoundsMin[2] + static_cast<float>(tileY) * tileSize;

	m_lastBuiltTileBoundsMax[0] = navMeshBoundsMin[0] + static_cast<float>(tileX + 1) * tileSize;
	m_lastBuiltTileBoundsMax[1] = navmeshBoundsMax[1];
	m_lastBuiltTileBoundsMax[2] = navMeshBoundsMin[2] + static_cast<float>(tileY + 1) * tileSize;

	m_tileColor = duRGBA(128, 32, 16, 64);

	m_navMesh->removeTile(m_navMesh->getTileRefAt(tileX, tileY, 0), 0, 0);
}

void Sample_TileMesh::buildAllTiles()
{
	if (!m_inputGeometry) { return; }
	if (!m_navMesh) { return; }

	const float* navMeshBoundsMin = m_inputGeometry->getNavMeshBoundsMin();
	const float* navMeshBoundsMax = m_inputGeometry->getNavMeshBoundsMax();
	int gridWidth = 0;
	int gridHeight = 0;
	rcCalcGridSize(navMeshBoundsMin, navMeshBoundsMax, m_cellSize, &gridWidth, &gridHeight);
	const int tileSize = static_cast<int>(m_tileSize);
	const int tileWidth = (gridWidth + tileSize - 1) / tileSize;
	const int tileHeight = (gridHeight + tileSize - 1) / tileSize;
	const float tileCellSize = m_tileSize * m_cellSize;

	// Start the build process.
	m_buildContext->startTimer(RC_TIMER_TEMP);

	for (int y = 0; y < tileHeight; ++y)
	{
		for (int x = 0; x < tileWidth; ++x)
		{
			m_lastBuiltTileBoundsMin[0] = navMeshBoundsMin[0] + static_cast<float>(x) * tileCellSize;
			m_lastBuiltTileBoundsMin[1] = navMeshBoundsMin[1];
			m_lastBuiltTileBoundsMin[2] = navMeshBoundsMin[2] + static_cast<float>(y) * tileCellSize;

			m_lastBuiltTileBoundsMax[0] = navMeshBoundsMin[0] + static_cast<float>(x + 1) * tileCellSize;
			m_lastBuiltTileBoundsMax[1] = navMeshBoundsMax[1];
			m_lastBuiltTileBoundsMax[2] = navMeshBoundsMin[2] + static_cast<float>(y + 1) * tileCellSize;

			int tileMeshDataSize = 0;
			unsigned char* tileMeshData = buildTileMesh(x, y, m_lastBuiltTileBoundsMin, m_lastBuiltTileBoundsMax, tileMeshDataSize);
			if (!tileMeshData) { continue; }

			// Remove any previous data (navmesh owns and deletes the data).
			m_navMesh->removeTile(m_navMesh->getTileRefAt(x, y, 0), 0, 0);
			// Let the navmesh own the data.
			const dtStatus status = m_navMesh->addTile(tileMeshData, tileMeshDataSize, DT_TILE_FREE_DATA, 0, 0);
			if (dtStatusFailed(status))
			{
				dtFree(tileMeshData);
			}
		}
	}

	// Record the total build time.
	m_buildContext->stopTimer(RC_TIMER_TEMP);
	m_totalBuildTimeMs = static_cast<float>(m_buildContext->getAccumulatedTime(RC_TIMER_TEMP)) / 1000.0f;
}

void Sample_TileMesh::removeAllTiles() const
{
	if (!m_inputGeometry || !m_navMesh) { return; }

	const float* navMeshBoundsMin = m_inputGeometry->getNavMeshBoundsMin();
	const float* navMeshBoundsMax = m_inputGeometry->getNavMeshBoundsMax();
	int gridWidth = 0;
	int gridHeight = 0;
	rcCalcGridSize(navMeshBoundsMin, navMeshBoundsMax, m_cellSize, &gridWidth, &gridHeight);
	const int tileSize = static_cast<int>(m_tileSize);
	const int tileWidth = (gridWidth + tileSize - 1) / tileSize;
	const int tileHeight = (gridHeight + tileSize - 1) / tileSize;

	for (int y = 0; y < tileHeight; ++y)
	{
		for (int x = 0; x < tileWidth; ++x)
		{
			m_navMesh->removeTile(m_navMesh->getTileRefAt(x, y, 0), 0, 0);
		}
	}
}

unsigned char* Sample_TileMesh::buildTileMesh(const int tileX, const int tileY, const float* boundsMin, const float* boundsMax, int& outDataSize)
{
	if (!m_inputGeometry || !m_inputGeometry->getMesh() || !m_inputGeometry->getChunkyMesh())
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return 0;
	}

	m_tileMemUsage = 0;
	m_tileBuildTime = 0;

	cleanup();

	const float* verts = m_inputGeometry->getMesh()->getVerts();
	const int numVerts = m_inputGeometry->getMesh()->getVertCount();
	const int numTris = m_inputGeometry->getMesh()->getTriCount();
	const rcChunkyTriMesh* chunkyMesh = m_inputGeometry->getChunkyMesh();

	// Init build configuration from GUI
	memset(&m_config, 0, sizeof(m_config));
	m_config.cs = m_cellSize;
	m_config.ch = m_cellHeight;
	m_config.walkableSlopeAngle = m_agentMaxSlope;
	m_config.walkableHeight = static_cast<int>(ceilf(m_agentHeight / m_config.ch));
	m_config.walkableClimb = static_cast<int>(floorf(m_agentMaxClimb / m_config.ch));
	m_config.walkableRadius = static_cast<int>(ceilf(m_agentRadius / m_config.cs));
	m_config.maxEdgeLen = static_cast<int>(m_edgeMaxLen / m_cellSize);
	m_config.maxSimplificationError = m_edgeMaxError;
	m_config.minRegionArea = static_cast<int>(rcSqr(m_regionMinSize));		// Note: area = size*size
	m_config.mergeRegionArea = static_cast<int>(rcSqr(m_regionMergeSize));	// Note: area = size*size
	m_config.maxVertsPerPoly = static_cast<int>(m_vertsPerPoly);
	m_config.tileSize = static_cast<int>(m_tileSize);
	m_config.borderSize = m_config.walkableRadius + 3; // Reserve enough padding.
	m_config.width = m_config.tileSize + m_config.borderSize * 2;
	m_config.height = m_config.tileSize + m_config.borderSize * 2;
	m_config.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_config.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

	// Expand the heightfield bounding box by border size to find the extents of geometry we need to build this tile.
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
	rcVcopy(m_config.bmin, boundsMin);
	rcVcopy(m_config.bmax, boundsMax);
	m_config.bmin[0] -= static_cast<float>(m_config.borderSize) * m_config.cs;
	m_config.bmin[2] -= static_cast<float>(m_config.borderSize) * m_config.cs;
	m_config.bmax[0] += static_cast<float>(m_config.borderSize) * m_config.cs;
	m_config.bmax[2] += static_cast<float>(m_config.borderSize) * m_config.cs;

	// Reset build times gathering.
	m_buildContext->resetTimers();

	// Start the build process.
	m_buildContext->startTimer(RC_TIMER_TOTAL);

	m_buildContext->log(RC_LOG_PROGRESS, "Building navigation:");
	m_buildContext->log(RC_LOG_PROGRESS, " - %d x %d cells", m_config.width, m_config.height);
	m_buildContext->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", static_cast<float>(numVerts) / 1000.0f, static_cast<float>(numTris) / 1000.0f);

	// Allocate voxel heightfield where we rasterize our input data to.
	m_heightfield = rcAllocHeightfield();
	if (!m_heightfield)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(m_buildContext, *m_heightfield, m_config.width, m_config.height, m_config.bmin, m_config.bmax, m_config.cs, m_config.ch))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}

	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	m_triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!m_triareas)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
		return 0;
	}

	float tbmin[2];
	float tbmax[2];
	tbmin[0] = m_config.bmin[0];
	tbmin[1] = m_config.bmin[2];
	tbmax[0] = m_config.bmax[0];
	tbmax[1] = m_config.bmax[2];
	int overlappingChunkIndexes[512];// TODO: Make grow when returning too many items.
	const int numOverlappingChunks = chunkyMesh->GetChunksOverlappingRect(tbmin, tbmax, overlappingChunkIndexes, 512);
	if (!numOverlappingChunks)
	{
		return 0;
	}

	m_tileTriCount = 0;

	for (int i = 0; i < numOverlappingChunks; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[overlappingChunkIndexes[i]];
		const int* nodeTris = &chunkyMesh->tris[node.i * 3];
		const int numNodeTris = node.n;

		m_tileTriCount += numNodeTris;

		memset(m_triareas, 0, numNodeTris * sizeof(unsigned char));
		rcMarkWalkableTriangles(m_buildContext, m_config.walkableSlopeAngle, verts, numVerts, nodeTris, numNodeTris, m_triareas);

		if (!rcRasterizeTriangles(m_buildContext, verts, numVerts, nodeTris, m_triareas, numNodeTris, *m_heightfield, m_config.walkableClimb))
		{
			return 0;
		}
	}

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (m_filterLowHangingObstacles)
	{
		rcFilterLowHangingWalkableObstacles(m_buildContext, m_config.walkableClimb, *m_heightfield);
	}
	if (m_filterLedgeSpans)
	{
		rcFilterLedgeSpans(m_buildContext, m_config.walkableHeight, m_config.walkableClimb, *m_heightfield);
	}
	if (m_filterWalkableLowHeightSpans)
	{
		rcFilterWalkableLowHeightSpans(m_buildContext, m_config.walkableHeight, *m_heightfield);
	}

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	m_compactHeightfield = rcAllocCompactHeightfield();
	if (!m_compactHeightfield)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(m_buildContext, m_config.walkableHeight, m_config.walkableClimb, *m_heightfield, *m_compactHeightfield))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_buildContext, m_config.walkableRadius, *m_compactHeightfield))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}

	// (Optional) Mark areas.
	const ConvexVolume* convexVolumes = m_inputGeometry->getConvexVolumes();
	for (int i = 0; i < m_inputGeometry->getConvexVolumeCount(); ++i)
	{
		rcMarkConvexPolyArea(m_buildContext, convexVolumes[i].verts, convexVolumes[i].nverts, convexVolumes[i].hmin, convexVolumes[i].hmax, static_cast<unsigned char>(convexVolumes[i].area), *m_compactHeightfield);
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
		if (!rcBuildDistanceField(m_buildContext, *m_compactHeightfield))
		{
			m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return 0;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(m_buildContext, *m_compactHeightfield, m_config.borderSize, m_config.minRegionArea, m_config.mergeRegionArea))
		{
			m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return 0;
		}
	}
	else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(m_buildContext, *m_compactHeightfield, m_config.borderSize, m_config.minRegionArea, m_config.mergeRegionArea))
		{
			m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return 0;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(m_buildContext, *m_compactHeightfield, m_config.borderSize, m_config.minRegionArea))
		{
			m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return 0;
		}
	}

	// Create contours.
	m_contourSet = rcAllocContourSet();
	if (!m_contourSet)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return 0;
	}
	if (!rcBuildContours(m_buildContext, *m_compactHeightfield, m_config.maxSimplificationError, m_config.maxEdgeLen, *m_contourSet))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
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
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return 0;
	}
	if (!rcBuildPolyMesh(m_buildContext, *m_contourSet, m_config.maxVertsPerPoly, *m_polyMesh))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return 0;
	}

	// Build detail mesh.
	m_detailPolyMesh = rcAllocPolyMeshDetail();
	if (!m_detailPolyMesh)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'dmesh'.");
		return 0;
	}

	if (!rcBuildPolyMeshDetail(m_buildContext, *m_polyMesh, *m_compactHeightfield, m_config.detailSampleDist, m_config.detailSampleMaxError, *m_detailPolyMesh))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could build polymesh detail.");
		return 0;
	}

	unsigned char* navData = 0;
	int navDataSize = 0;
	if (m_config.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (m_polyMesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			m_buildContext->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", m_polyMesh->nverts, 0xffff);
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
		params.offMeshConVerts = m_inputGeometry->getOffMeshConnectionVerts();
		params.offMeshConRad = m_inputGeometry->getOffMeshConnectionRads();
		params.offMeshConDir = m_inputGeometry->getOffMeshConnectionDirs();
		params.offMeshConAreas = m_inputGeometry->getOffMeshConnectionAreas();
		params.offMeshConFlags = m_inputGeometry->getOffMeshConnectionFlags();
		params.offMeshConUserID = m_inputGeometry->getOffMeshConnectionId();
		params.offMeshConCount = m_inputGeometry->getOffMeshConnectionCount();
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		params.tileX = tileX;
		params.tileY = tileY;
		params.tileLayer = 0;
		rcVcopy(params.bmin, m_polyMesh->bmin);
		rcVcopy(params.bmax, m_polyMesh->bmax);
		params.cs = m_config.cs;
		params.ch = m_config.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_buildContext->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return 0;
		}
	}
	m_tileMemUsage = static_cast<float>(navDataSize) / 1024.0f;

	m_buildContext->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*m_buildContext, m_buildContext->getAccumulatedTime(RC_TIMER_TOTAL));
	m_buildContext->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_polyMesh->nverts, m_polyMesh->npolys);

	m_tileBuildTime = static_cast<float>(m_buildContext->getAccumulatedTime(RC_TIMER_TOTAL)) / 1000.0f;

	outDataSize = navDataSize;
	return navData;
}
