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

#include "SDL_opengl.h"
#include "imguiHelpers.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include "PartitionedMesh.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "InputGeom.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "Sample.h"
#include "Tool_ConvexVolume.h"
#include "Tool_Crowd.h"
#include "Tool_NavMeshPrune.h"
#include "Tool_NavMeshTester.h"
#include "Tool_OffMeshConnection.h"

#include <imgui.h>

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
	unsigned int r = (v > 0xffff) << 4;
	v >>= r;
	unsigned int shift = (v > 0xff) << 3;
	v >>= shift;
	r |= shift;
	shift = (v > 0xf) << 2;
	v >>= shift;
	r |= shift;
	shift = (v > 0x3) << 1;
	v >>= shift;
	r |= shift;
	r |= (v >> 1);
	return r;
}
}

const char* Sample_TileMesh::drawModeNames[]{
	"Input Mesh",
	"Navmesh",
	"Navmesh Invis",
	"Navmesh Trans",
	"Navmesh BVTree",
	"Navmesh Nodes",
	"Navmesh Portals",
	"Voxels",
	"Walkable Voxels",
	"Compact",
	"Compact Distance",
	"Compact Regions",
	"Region Connections",
	"Raw Contours",
	"Both Contours",
	"Contours",
	"Poly Mesh",
	"Poly Mesh Detail"};

class NavMeshTileTool : public SampleTool
{
	Sample_TileMesh* m_sample = nullptr;
	float m_hitPos[3] = {0, 0, 0};
	bool m_hitPosSet = false;

public:
	~NavMeshTileTool() override = default;

	SampleToolType type() override { return SampleToolType::TILE_EDIT; }
	void init(Sample* sample) override { m_sample = static_cast<Sample_TileMesh*>(sample); }
	void reset() override {}

	void drawMenuUI() override
	{
		ImGui::Text("Create Tiles");
		if (ImGui::Button("Create All"))
		{
			if (m_sample)
			{
				m_sample->buildAllTiles();
			}
		}
		if (ImGui::Button("Remove All"))
		{
			if (m_sample)
			{
				m_sample->removeAllTiles();
			}
		}
	}

	void onClick(const float* /*s*/, const float* p, bool shift) override
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

	void onToggle() override {}

	void singleStep() override {}

	void update(const float /*dt*/) override {}

	void render() override
	{
		if (!m_hitPosSet)
		{
			return;
		}

		const float s = m_sample->agentRadius;
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

	void drawOverlayUI() override
	{
		if (m_hitPosSet)
		{
			int tx = 0;
			int ty = 0;
			m_sample->getTilePos(m_hitPos, tx, ty);
			char text[32];
			snprintf(text, 32, "(%d,%d)", tx, ty);
			DrawWorldspaceText(m_hitPos[0], m_hitPos[1], m_hitPos[2], IM_COL32(0, 0, 0, 220), text);
		}

		// Tool help
		DrawScreenspaceText(280, 40, IM_COL32(255, 255, 255, 192), "LMB: Rebuild selected tile.  Shift+LMB: Clear tile selection.");
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
	dtFreeNavMesh(navMesh);
	navMesh = 0;
}

void Sample_TileMesh::cleanup()
{
	delete[] triareas;
	triareas = nullptr;
	rcFreeHeightField(heightfield);
	heightfield = nullptr;
	rcFreeCompactHeightfield(compactHeightfield);
	compactHeightfield = nullptr;
	rcFreeContourSet(contourSet);
	contourSet = nullptr;
	rcFreePolyMesh(polyMesh);
	polyMesh = nullptr;
	rcFreePolyMeshDetail(detailPolyMesh);
	detailPolyMesh = nullptr;
}

void Sample_TileMesh::drawSettingsUI()
{
	drawCommonSettingsUI();

	ImGui::Checkbox("Build All Tiles", &buildAll);

	ImGui::Text("Tiling");

	if (ImGui::SliderInt("TileSize", &tileSize, 16, 1024))
	{
		// Snap to multiples of 16
		tileSize = static_cast<int>(roundf(static_cast<float>(tileSize) / 16.0f)) * 16;
	}

	if (inputGeometry)
	{
		const float* navMeshBoundsMin = inputGeometry->getNavMeshBoundsMin();
		const float* navMeshBoundsMax = inputGeometry->getNavMeshBoundsMax();
		int gridWidth = 0;
		int gridHeight = 0;
		rcCalcGridSize(navMeshBoundsMin, navMeshBoundsMax, cellSize, &gridWidth, &gridHeight);
		const int tileWidth = (gridWidth + tileSize - 1) / tileSize;
		const int tileHeight = (gridHeight + tileSize - 1) / tileSize;

		ImGui::Text("Tiles  %d x %d", tileWidth, tileHeight);

		// Max tiles and max polys affect how the tile IDs are calculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)ilog2(nextPow2(tileWidth * tileHeight)), 14);
		tileBits = rcMin(tileBits, 14);
		int polyBits = 22 - tileBits;
		maxTiles = 1 << tileBits;
		maxPolysPerTile = 1 << polyBits;
		ImGui::Text("Max Tiles  %d", maxTiles);
		ImGui::Text("Max Polys  %d", maxPolysPerTile);
	}
	else
	{
		maxTiles = 0;
		maxPolysPerTile = 0;
	}

	ImGui::Separator();

	ImGui::Indent();

	if (ImGui::Button("Save"))
	{
		Sample::saveAll("all_tiles_navmesh.bin", navMesh);
	}

	if (ImGui::Button("Load"))
	{
		dtFreeNavMesh(navMesh);
		navMesh = Sample::loadAll("all_tiles_navmesh.bin");
		navQuery->init(navMesh, 2048);
	}

	ImGui::Unindent();

	ImGui::Text("Build Time: %.1fms", totalBuildTimeMs);

	ImGui::Separator();
}

void Sample_TileMesh::drawToolsUI()
{
	const SampleToolType currentType = !tool ? SampleToolType::NONE : tool->type();
#define TOOL(toolType, toolClass)                                      \
	if (ImGui::RadioButton(                                            \
			toolNames[static_cast<uint8_t>(SampleToolType::toolType)], \
			currentType == SampleToolType::toolType))                  \
	{                                                                  \
		setTool(new toolClass{});                                      \
	}
	TOOL(NAVMESH_TESTER, NavMeshTesterTool)
	TOOL(NAVMESH_PRUNE, NavMeshPruneTool)
	TOOL(TILE_EDIT, NavMeshTileTool)
	TOOL(OFFMESH_CONNECTION, OffMeshConnectionTool)
	TOOL(CONVEX_VOLUME, ConvexVolumeTool)
	TOOL(CROWD, CrowdTool)
#undef TOOL

	ImGui::Separator();

	if (tool)
	{
		tool->drawMenuUI();
	}
}

void Sample_TileMesh::UI_DrawModeOption(DrawMode drawMode, bool enabled)
{
	ImGui::BeginDisabled(!enabled);
	const bool is_selected = this->drawMode == drawMode;
	if (ImGui::Selectable(drawModeNames[static_cast<int>(drawMode)], is_selected))
	{
		this->drawMode = drawMode;
	}

	// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
	if (is_selected)
	{
		ImGui::SetItemDefaultFocus();
	}
	ImGui::EndDisabled();
}

void Sample_TileMesh::drawDebugUI()
{
	ImGui::Text("Draw");
	if (ImGui::BeginCombo("##drawMode", drawModeNames[static_cast<int>(this->drawMode)], 0))
	{
		UI_DrawModeOption(DrawMode::MESH, true);
		UI_DrawModeOption(DrawMode::NAVMESH, navMesh != nullptr);
		UI_DrawModeOption(DrawMode::NAVMESH_INVIS, navMesh != nullptr);
		UI_DrawModeOption(DrawMode::NAVMESH_TRANS, navMesh != nullptr);
		UI_DrawModeOption(DrawMode::NAVMESH_BVTREE, navMesh != nullptr);
		UI_DrawModeOption(DrawMode::NAVMESH_NODES, navQuery != nullptr);
		UI_DrawModeOption(DrawMode::NAVMESH_PORTALS, navMesh != nullptr);
		UI_DrawModeOption(DrawMode::VOXELS, heightfield != nullptr);
		UI_DrawModeOption(DrawMode::VOXELS_WALKABLE, heightfield != nullptr);
		UI_DrawModeOption(DrawMode::COMPACT, compactHeightfield != nullptr);
		UI_DrawModeOption(DrawMode::COMPACT_DISTANCE, compactHeightfield != nullptr);
		UI_DrawModeOption(DrawMode::COMPACT_REGIONS, compactHeightfield != nullptr);
		UI_DrawModeOption(DrawMode::REGION_CONNECTIONS, contourSet != nullptr);
		UI_DrawModeOption(DrawMode::RAW_CONTOURS, contourSet != nullptr);
		UI_DrawModeOption(DrawMode::BOTH_CONTOURS, contourSet != nullptr);
		UI_DrawModeOption(DrawMode::CONTOURS, contourSet != nullptr);
		UI_DrawModeOption(DrawMode::POLYMESH, polyMesh != nullptr);
		UI_DrawModeOption(DrawMode::POLYMESH_DETAIL, detailPolyMesh != nullptr);
		ImGui::EndCombo();
	}
}

void Sample_TileMesh::render()
{
	if (!inputGeometry || inputGeometry->mesh.getVertCount() == 0)
	{
		return;
	}

	const float texScale = 1.0f / (cellSize * 10.0f);

	// Draw mesh
	if (drawMode != DrawMode::NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(
			&debugDraw,
			inputGeometry->mesh.verts.data(),
			inputGeometry->mesh.getVertCount(),
			inputGeometry->mesh.tris.data(),
			inputGeometry->mesh.normals.data(),
			inputGeometry->mesh.getTriCount(),
			agentMaxSlope,
			texScale);
		inputGeometry->drawOffMeshConnections(&debugDraw);
	}

	glDepthMask(GL_FALSE);

	// Draw bounds
	const float* navMeshBoundsMin = inputGeometry->getNavMeshBoundsMin();
	const float* navMeshBoundsMax = inputGeometry->getNavMeshBoundsMax();
	duDebugDrawBoxWire(
		&debugDraw,
		navMeshBoundsMin[0],
		navMeshBoundsMin[1],
		navMeshBoundsMin[2],
		navMeshBoundsMax[0],
		navMeshBoundsMax[1],
		navMeshBoundsMax[2],
		duRGBA(255, 255, 255, 128),
		1.0f);

	// Tiling grid.
	int gridWith = 0;
	int gridHeight = 0;
	rcCalcGridSize(navMeshBoundsMin, navMeshBoundsMax, cellSize, &gridWith, &gridHeight);
	const int tileWidth = (gridWith + tileSize - 1) / tileSize;
	const int tileHeight = (gridHeight + tileSize - 1) / tileSize;
	const float size = tileSize * cellSize;
	duDebugDrawGridXZ(
		&debugDraw,
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
		&debugDraw,
		lastBuiltTileBoundsMin[0],
		lastBuiltTileBoundsMin[1],
		lastBuiltTileBoundsMin[2],
		lastBuiltTileBoundsMax[0],
		lastBuiltTileBoundsMax[1],
		lastBuiltTileBoundsMax[2],
		tileColor,
		1.0f);

	if (navMesh && navQuery &&
	    (drawMode == DrawMode::NAVMESH || drawMode == DrawMode::NAVMESH_TRANS || drawMode == DrawMode::NAVMESH_BVTREE ||
	     drawMode == DrawMode::NAVMESH_NODES || drawMode == DrawMode::NAVMESH_PORTALS || drawMode == DrawMode::NAVMESH_INVIS))
	{
		if (drawMode != DrawMode::NAVMESH_INVIS)
		{
			duDebugDrawNavMeshWithClosedList(&debugDraw, *navMesh, *navQuery, navMeshDrawFlags);
		}
		if (drawMode == DrawMode::NAVMESH_BVTREE)
		{
			duDebugDrawNavMeshBVTree(&debugDraw, *navMesh);
		}
		if (drawMode == DrawMode::NAVMESH_PORTALS)
		{
			duDebugDrawNavMeshPortals(&debugDraw, *navMesh);
		}
		if (drawMode == DrawMode::NAVMESH_NODES)
		{
			duDebugDrawNavMeshNodes(&debugDraw, *navQuery);
		}
		duDebugDrawNavMeshPolysWithFlags(&debugDraw, *navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0, 0, 0, 128));
	}

	glDepthMask(GL_TRUE);

	if (compactHeightfield && drawMode == DrawMode::COMPACT)
	{
		duDebugDrawCompactHeightfieldSolid(&debugDraw, *compactHeightfield);
	}

	if (compactHeightfield && drawMode == DrawMode::COMPACT_DISTANCE)
	{
		duDebugDrawCompactHeightfieldDistance(&debugDraw, *compactHeightfield);
	}
	if (compactHeightfield && drawMode == DrawMode::COMPACT_REGIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&debugDraw, *compactHeightfield);
	}
	if (heightfield && drawMode == DrawMode::VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&debugDraw, *heightfield);
		glDisable(GL_FOG);
	}
	if (heightfield && drawMode == DrawMode::VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&debugDraw, *heightfield);
		glDisable(GL_FOG);
	}

	if (contourSet && drawMode == DrawMode::RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&debugDraw, *contourSet);
		glDepthMask(GL_TRUE);
	}

	if (contourSet && drawMode == DrawMode::BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&debugDraw, *contourSet, 0.5f);
		duDebugDrawContours(&debugDraw, *contourSet);
		glDepthMask(GL_TRUE);
	}
	if (contourSet && drawMode == DrawMode::CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&debugDraw, *contourSet);
		glDepthMask(GL_TRUE);
	}
	if (compactHeightfield && contourSet && drawMode == DrawMode::REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&debugDraw, *compactHeightfield);

		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&debugDraw, *contourSet);
		glDepthMask(GL_TRUE);
	}
	if (polyMesh && drawMode == DrawMode::POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&debugDraw, *polyMesh);
		glDepthMask(GL_TRUE);
	}
	if (detailPolyMesh && drawMode == DrawMode::POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&debugDraw, *detailPolyMesh);
		glDepthMask(GL_TRUE);
	}

	inputGeometry->drawConvexVolumes(&debugDraw);

	if (tool)
	{
		tool->render();
	}
	renderToolStates();

	glDepthMask(GL_TRUE);
}

void Sample_TileMesh::renderOverlay()
{
	// Draw start and end point labels
	if (tileBuildTime > 0.0f)
	{
		char text[64];
		snprintf(text, 64, "%.3fms / %dTris / %.1fkB", tileBuildTime, tileTriCount, tileMemUsage);
		DrawWorldspaceText(
			(lastBuiltTileBoundsMin[0] + lastBuiltTileBoundsMax[0]) / 2,
			(lastBuiltTileBoundsMin[1] + lastBuiltTileBoundsMax[1]) / 2,
			(lastBuiltTileBoundsMin[2] + lastBuiltTileBoundsMax[2]) / 2,
			IM_COL32(0, 0, 0, 220),
			text);
	}

	if (tool)
	{
		tool->drawOverlayUI();
	}
	renderOverlayToolStates();
}

void Sample_TileMesh::onMeshChanged(InputGeom* geom)
{
	Sample::onMeshChanged(geom);

	const BuildSettings* buildSettings = geom->getBuildSettings();
	if (buildSettings && buildSettings->tileSize > 0)
	{
		tileSize = static_cast<int>(buildSettings->tileSize);
	}

	cleanup();

	dtFreeNavMesh(navMesh);
	navMesh = nullptr;

	if (tool)
	{
		tool->reset();
		tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

bool Sample_TileMesh::build()
{
	if (!inputGeometry || inputGeometry->mesh.getVertCount() == 0)
	{
		buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return false;
	}

	dtFreeNavMesh(navMesh);

	navMesh = dtAllocNavMesh();
	if (!navMesh)
	{
		buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return false;
	}

	dtNavMeshParams params;
	rcVcopy(params.orig, inputGeometry->getNavMeshBoundsMin());
	params.tileWidth = tileSize * cellSize;
	params.tileHeight = tileSize * cellSize;
	params.maxTiles = maxTiles;
	params.maxPolys = maxPolysPerTile;

	dtStatus status = navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return false;
	}

	status = navQuery->init(navMesh, 2048);
	if (dtStatusFailed(status))
	{
		buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return false;
	}

	if (buildAll)
	{
		buildAllTiles();
	}

	if (tool)
	{
		tool->init(this);
	}
	initToolStates(this);

	return true;
}

void Sample_TileMesh::collectSettings(BuildSettings& settings)
{
	Sample::collectSettings(settings);

	settings.tileSize = static_cast<float>(tileSize);
}

void Sample_TileMesh::buildTile(const float* pos)
{
	if (!inputGeometry)
	{
		return;
	}
	if (!navMesh)
	{
		return;
	}

	const float* navMeshBoundsMin = inputGeometry->getNavMeshBoundsMin();
	const float* navMeshBoundsMax = inputGeometry->getNavMeshBoundsMax();

	const float tileWorldSize = tileSize * cellSize;
	const int tileX = static_cast<int>((pos[0] - navMeshBoundsMin[0]) / tileWorldSize);
	const int tileY = static_cast<int>((pos[2] - navMeshBoundsMin[2]) / tileWorldSize);

	lastBuiltTileBoundsMin[0] = navMeshBoundsMin[0] + static_cast<float>(tileX) * tileWorldSize;
	lastBuiltTileBoundsMin[1] = navMeshBoundsMin[1];
	lastBuiltTileBoundsMin[2] = navMeshBoundsMin[2] + static_cast<float>(tileY) * tileWorldSize;

	lastBuiltTileBoundsMax[0] = navMeshBoundsMin[0] + static_cast<float>(tileX + 1) * tileWorldSize;
	lastBuiltTileBoundsMax[1] = navMeshBoundsMax[1];
	lastBuiltTileBoundsMax[2] = navMeshBoundsMin[2] + static_cast<float>(tileY + 1) * tileWorldSize;

	tileColor = duRGBA(255, 255, 255, 64);

	buildContext->resetLog();

	int tileMeshDataSize = 0;
	unsigned char* tileMeshData = buildTileMesh(tileX, tileY, lastBuiltTileBoundsMin, lastBuiltTileBoundsMax, tileMeshDataSize);

	// Remove any previous data (navmesh owns and deletes the data).
	navMesh->removeTile(navMesh->getTileRefAt(tileX, tileY, 0), 0, 0);

	// Add tile, or leave the location empty.
	if (tileMeshData)
	{
		// Let the navmesh own the data.
		const dtStatus status = navMesh->addTile(tileMeshData, tileMeshDataSize, DT_TILE_FREE_DATA, 0, 0);
		if (dtStatusFailed(status))
		{
			dtFree(tileMeshData);
		}
	}

	buildContext->dumpLog("Build Tile (%d,%d):", tileX, tileY);
}

void Sample_TileMesh::getTilePos(const float* pos, int& outTileX, int& outTileY) const
{
	if (!inputGeometry)
	{
		return;
	}

	const float* navMeshBoundsMin = inputGeometry->getNavMeshBoundsMin();

	const float tileWorldSize = tileSize * cellSize;
	outTileX = static_cast<int>((pos[0] - navMeshBoundsMin[0]) / tileWorldSize);
	outTileY = static_cast<int>((pos[2] - navMeshBoundsMin[2]) / tileWorldSize);
}

void Sample_TileMesh::removeTile(const float* pos)
{
	if (!inputGeometry)
	{
		return;
	}
	if (!navMesh)
	{
		return;
	}

	const float* navMeshBoundsMin = inputGeometry->getNavMeshBoundsMin();
	const float* navmeshBoundsMax = inputGeometry->getNavMeshBoundsMax();

	const float tileWorldSize = tileSize * cellSize;
	const int tileX = static_cast<int>((pos[0] - navMeshBoundsMin[0]) / tileWorldSize);
	const int tileY = static_cast<int>((pos[2] - navMeshBoundsMin[2]) / tileWorldSize);

	lastBuiltTileBoundsMin[0] = navMeshBoundsMin[0] + static_cast<float>(tileX) * tileWorldSize;
	lastBuiltTileBoundsMin[1] = navMeshBoundsMin[1];
	lastBuiltTileBoundsMin[2] = navMeshBoundsMin[2] + static_cast<float>(tileY) * tileWorldSize;

	lastBuiltTileBoundsMax[0] = navMeshBoundsMin[0] + static_cast<float>(tileX + 1) * tileWorldSize;
	lastBuiltTileBoundsMax[1] = navmeshBoundsMax[1];
	lastBuiltTileBoundsMax[2] = navMeshBoundsMin[2] + static_cast<float>(tileY + 1) * tileWorldSize;

	tileColor = duRGBA(128, 32, 16, 64);

	navMesh->removeTile(navMesh->getTileRefAt(tileX, tileY, 0), 0, 0);
}

void Sample_TileMesh::buildAllTiles()
{
	if (!inputGeometry)
	{
		return;
	}
	if (!navMesh)
	{
		return;
	}

	const float* navMeshBoundsMin = inputGeometry->getNavMeshBoundsMin();
	const float* navMeshBoundsMax = inputGeometry->getNavMeshBoundsMax();
	int gridWidth = 0;
	int gridHeight = 0;
	rcCalcGridSize(navMeshBoundsMin, navMeshBoundsMax, cellSize, &gridWidth, &gridHeight);
	const int tileWidth = (gridWidth + tileSize - 1) / tileSize;
	const int tileHeight = (gridHeight + tileSize - 1) / tileSize;
	const float tileCellSize = tileSize * cellSize;

	// Start the build process.
	buildContext->startTimer(RC_TIMER_TEMP);

	for (int y = 0; y < tileHeight; ++y)
	{
		for (int x = 0; x < tileWidth; ++x)
		{
			lastBuiltTileBoundsMin[0] = navMeshBoundsMin[0] + static_cast<float>(x) * tileCellSize;
			lastBuiltTileBoundsMin[1] = navMeshBoundsMin[1];
			lastBuiltTileBoundsMin[2] = navMeshBoundsMin[2] + static_cast<float>(y) * tileCellSize;

			lastBuiltTileBoundsMax[0] = navMeshBoundsMin[0] + static_cast<float>(x + 1) * tileCellSize;
			lastBuiltTileBoundsMax[1] = navMeshBoundsMax[1];
			lastBuiltTileBoundsMax[2] = navMeshBoundsMin[2] + static_cast<float>(y + 1) * tileCellSize;

			int tileMeshDataSize = 0;
			unsigned char* tileMeshData = buildTileMesh(x, y, lastBuiltTileBoundsMin, lastBuiltTileBoundsMax, tileMeshDataSize);
			if (!tileMeshData)
			{
				continue;
			}

			// Remove any previous data (navmesh owns and deletes the data).
			navMesh->removeTile(navMesh->getTileRefAt(x, y, 0), 0, 0);
			// Let the navmesh own the data.
			const dtStatus status = navMesh->addTile(tileMeshData, tileMeshDataSize, DT_TILE_FREE_DATA, 0, 0);
			if (dtStatusFailed(status))
			{
				dtFree(tileMeshData);
			}
		}
	}

	// Record the total build time.
	buildContext->stopTimer(RC_TIMER_TEMP);
	totalBuildTimeMs = static_cast<float>(buildContext->getAccumulatedTime(RC_TIMER_TEMP)) / 1000.0f;
}

void Sample_TileMesh::removeAllTiles() const
{
	if (inputGeometry == nullptr)
	{
		return;
	}
	if (navMesh == nullptr)
	{
		return;
	}

	const float* navMeshBoundsMin = inputGeometry->getNavMeshBoundsMin();
	const float* navMeshBoundsMax = inputGeometry->getNavMeshBoundsMax();
	int gridWidth = 0;
	int gridHeight = 0;
	rcCalcGridSize(navMeshBoundsMin, navMeshBoundsMax, cellSize, &gridWidth, &gridHeight);
	const int tileWidth = (gridWidth + tileSize - 1) / tileSize;
	const int tileHeight = (gridHeight + tileSize - 1) / tileSize;

	for (int tileY = 0; tileY < tileHeight; ++tileY)
	{
		for (int tileX = 0; tileX < tileWidth; ++tileX)
		{
			navMesh->removeTile(navMesh->getTileRefAt(tileX, tileY, 0), 0, 0);
		}
	}
}

unsigned char* Sample_TileMesh::buildTileMesh(
	const int tileX,
	const int tileY,
	const float* boundsMin,
	const float* boundsMax,
	int& outDataSize)
{
	if (!inputGeometry || inputGeometry->mesh.getVertCount() == 0 || !inputGeometry->partitionedMesh)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return 0;
	}

	tileMemUsage = 0;
	tileBuildTime = 0;

	cleanup();

	const float* verts = inputGeometry->mesh.verts.data();
	const int numVerts = inputGeometry->mesh.getVertCount();
	const int numTris = inputGeometry->mesh.getTriCount();
	const PartitionedMesh* partitionedMesh = inputGeometry->partitionedMesh;

	// Init build configuration from GUI
	memset(&config, 0, sizeof(config));
	config.cs = cellSize;
	config.ch = cellHeight;
	config.walkableSlopeAngle = agentMaxSlope;
	config.walkableHeight = static_cast<int>(ceilf(agentHeight / config.ch));
	config.walkableClimb = static_cast<int>(floorf(agentMaxClimb / config.ch));
	config.walkableRadius = static_cast<int>(ceilf(agentRadius / config.cs));
	config.maxEdgeLen = static_cast<int>(edgeMaxLen / cellSize);
	config.maxSimplificationError = edgeMaxError;
	config.minRegionArea = static_cast<int>(rcSqr(regionMinSize));      // Note: area = size*size
	config.mergeRegionArea = static_cast<int>(rcSqr(regionMergeSize));  // Note: area = size*size
	config.maxVertsPerPoly = static_cast<int>(vertsPerPoly);
	config.tileSize = tileSize;
	config.borderSize = config.walkableRadius + 3;  // Reserve enough padding.
	config.width = config.tileSize + config.borderSize * 2;
	config.height = config.tileSize + config.borderSize * 2;
	config.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
	config.detailSampleMaxError = cellHeight * detailSampleMaxError;

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
	rcVcopy(config.bmin, boundsMin);
	rcVcopy(config.bmax, boundsMax);
	config.bmin[0] -= static_cast<float>(config.borderSize) * config.cs;
	config.bmin[2] -= static_cast<float>(config.borderSize) * config.cs;
	config.bmax[0] += static_cast<float>(config.borderSize) * config.cs;
	config.bmax[2] += static_cast<float>(config.borderSize) * config.cs;

	// Reset build times gathering.
	buildContext->resetTimers();

	// Start the build process.
	buildContext->startTimer(RC_TIMER_TOTAL);

	buildContext->log(RC_LOG_PROGRESS, "Building navigation:");
	buildContext->log(RC_LOG_PROGRESS, " - %d x %d cells", config.width, config.height);
	buildContext->log(
		RC_LOG_PROGRESS,
		" - %.1fK verts, %.1fK tris",
		static_cast<float>(numVerts) / 1000.0f,
		static_cast<float>(numTris) / 1000.0f);

	// Allocate voxel heightfield where we rasterize our input data to.
	heightfield = rcAllocHeightfield();
	if (!heightfield)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(
			buildContext,
			*heightfield,
			config.width,
			config.height,
			config.bmin,
			config.bmax,
			config.cs,
			config.ch))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}

	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	triareas = new unsigned char[partitionedMesh->maxTrisPerChunk];
	if (!triareas)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'triareas' (%d).", partitionedMesh->maxTrisPerChunk);
		return 0;
	}

	float tileBoundsMin[2];
	float tileBoundsMax[2];
	tileBoundsMin[0] = config.bmin[0];
	tileBoundsMin[1] = config.bmin[2];
	tileBoundsMax[0] = config.bmax[0];
	tileBoundsMax[1] = config.bmax[2];
	std::vector<int> overlappingNodes;
	partitionedMesh->GetNodesOverlappingRect(tileBoundsMin, tileBoundsMax, overlappingNodes);
	if (overlappingNodes.empty())
	{
		return 0;
	}

	tileTriCount = 0;

	for (int nodeIndex : overlappingNodes)
	{
		const PartitionedMesh::Node& node = partitionedMesh->nodes[nodeIndex];
		const int* nodeTris = &partitionedMesh->tris[node.triIndex * 3];
		const int numNodeTris = node.numTris;

		tileTriCount += numNodeTris;

		memset(triareas, 0, numNodeTris * sizeof(unsigned char));
		rcMarkWalkableTriangles(buildContext, config.walkableSlopeAngle, verts, numVerts, nodeTris, numNodeTris, triareas);

		if (!rcRasterizeTriangles(
				buildContext,
				verts,
				numVerts,
				nodeTris,
				triareas,
				numNodeTris,
				*heightfield,
				config.walkableClimb))
		{
			return 0;
		}
	}

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (filterLowHangingObstacles)
	{
		rcFilterLowHangingWalkableObstacles(buildContext, config.walkableClimb, *heightfield);
	}
	if (filterLedgeSpans)
	{
		rcFilterLedgeSpans(buildContext, config.walkableHeight, config.walkableClimb, *heightfield);
	}
	if (filterWalkableLowHeightSpans)
	{
		rcFilterWalkableLowHeightSpans(buildContext, config.walkableHeight, *heightfield);
	}

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	compactHeightfield = rcAllocCompactHeightfield();
	if (!compactHeightfield)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(
			buildContext,
			config.walkableHeight,
			config.walkableClimb,
			*heightfield,
			*compactHeightfield))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(buildContext, config.walkableRadius, *compactHeightfield))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}

	// (Optional) Mark areas.
	const ConvexVolume* convexVolumes = inputGeometry->convexVolumes;
	for (int i = 0; i < inputGeometry->convexVolumeCount; ++i)
	{
		rcMarkConvexPolyArea(
			buildContext,
			convexVolumes[i].verts,
			convexVolumes[i].nverts,
			convexVolumes[i].hmin,
			convexVolumes[i].hmax,
			static_cast<unsigned char>(convexVolumes[i].area),
			*compactHeightfield);
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

	if (partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(buildContext, *compactHeightfield))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return 0;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(buildContext, *compactHeightfield, config.borderSize, config.minRegionArea, config.mergeRegionArea))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return 0;
		}
	}
	else if (partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(
				buildContext,
				*compactHeightfield,
				config.borderSize,
				config.minRegionArea,
				config.mergeRegionArea))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return 0;
		}
	}
	else  // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(buildContext, *compactHeightfield, config.borderSize, config.minRegionArea))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return 0;
		}
	}

	// Create contours.
	contourSet = rcAllocContourSet();
	if (!contourSet)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return 0;
	}
	if (!rcBuildContours(buildContext, *compactHeightfield, config.maxSimplificationError, config.maxEdgeLen, *contourSet))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return 0;
	}

	if (contourSet->nconts == 0)
	{
		return 0;
	}

	// Build polygon navmesh from the contours.
	polyMesh = rcAllocPolyMesh();
	if (!polyMesh)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return 0;
	}
	if (!rcBuildPolyMesh(buildContext, *contourSet, config.maxVertsPerPoly, *polyMesh))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return 0;
	}

	// Build detail mesh.
	detailPolyMesh = rcAllocPolyMeshDetail();
	if (!detailPolyMesh)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'dmesh'.");
		return 0;
	}

	if (!rcBuildPolyMeshDetail(
			buildContext,
			*polyMesh,
			*compactHeightfield,
			config.detailSampleDist,
			config.detailSampleMaxError,
			*detailPolyMesh))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could build polymesh detail.");
		return 0;
	}

	unsigned char* navData = 0;
	int navDataSize = 0;
	if (config.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (polyMesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			buildContext->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", polyMesh->nverts, 0xffff);
			return 0;
		}

		// Update poly flags from areas.
		for (int i = 0; i < polyMesh->npolys; ++i)
		{
			if (polyMesh->areas[i] == RC_WALKABLE_AREA)
			{
				polyMesh->areas[i] = SAMPLE_POLYAREA_GROUND;
			}

			if (polyMesh->areas[i] == SAMPLE_POLYAREA_GROUND || polyMesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
			    polyMesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				polyMesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (polyMesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				polyMesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (polyMesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				polyMesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = polyMesh->verts;
		params.vertCount = polyMesh->nverts;
		params.polys = polyMesh->polys;
		params.polyAreas = polyMesh->areas;
		params.polyFlags = polyMesh->flags;
		params.polyCount = polyMesh->npolys;
		params.nvp = polyMesh->nvp;
		params.detailMeshes = detailPolyMesh->meshes;
		params.detailVerts = detailPolyMesh->verts;
		params.detailVertsCount = detailPolyMesh->nverts;
		params.detailTris = detailPolyMesh->tris;
		params.detailTriCount = detailPolyMesh->ntris;
		params.offMeshConVerts = inputGeometry->offMeshConVerts;
		params.offMeshConRad = inputGeometry->offMeshConRads;
		params.offMeshConDir = inputGeometry->offMeshConDirs;
		params.offMeshConAreas = inputGeometry->offMeshConAreas;
		params.offMeshConFlags = inputGeometry->offMeshConFlags;
		params.offMeshConUserID = inputGeometry->offMeshConId;
		params.offMeshConCount = inputGeometry->offMeshConCount;
		params.walkableHeight = agentHeight;
		params.walkableRadius = agentRadius;
		params.walkableClimb = agentMaxClimb;
		params.tileX = tileX;
		params.tileY = tileY;
		params.tileLayer = 0;
		rcVcopy(params.bmin, polyMesh->bmin);
		rcVcopy(params.bmax, polyMesh->bmax);
		params.cs = config.cs;
		params.ch = config.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			buildContext->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return 0;
		}
	}
	tileMemUsage = static_cast<float>(navDataSize) / 1024.0f;

	buildContext->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*buildContext, buildContext->getAccumulatedTime(RC_TIMER_TOTAL));
	buildContext->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", polyMesh->nverts, polyMesh->npolys);

	tileBuildTime = static_cast<float>(buildContext->getAccumulatedTime(RC_TIMER_TOTAL)) / 1000.0f;

	outDataSize = navDataSize;
	return navData;
}
