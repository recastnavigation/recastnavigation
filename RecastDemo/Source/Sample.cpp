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

#include "Sample.h"

#include "DetourCrowd.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "InputGeom.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "imguiHelpers.h"

#include <imgui.h>

#include <cstdio>

#ifdef WIN32
#	define snprintf _snprintf
#endif

const char* toolNames[] = {
	"None",
	"Create Tiles",
	"Highlight Tile Cache",
	"Create Temp Obstacles",
	"Test Navmesh",
	"Prune Navmesh",
	"Create Off-Mesh Connections",
	"Create Convex Volumes",
	"Create Crowds",
};

namespace
{

constexpr int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';  //'MSET';
constexpr int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

}

unsigned int SampleDebugDraw::areaToCol(unsigned int area)
{
	switch (area)
	{
	// Ground (0) : light blue
	case SAMPLE_POLYAREA_GROUND:
		return duRGBA(0, 192, 255, 255);
	// Water : blue
	case SAMPLE_POLYAREA_WATER:
		return duRGBA(0, 0, 255, 255);
	// Road : brown
	case SAMPLE_POLYAREA_ROAD:
		return duRGBA(50, 20, 12, 255);
	// Door : cyan
	case SAMPLE_POLYAREA_DOOR:
		return duRGBA(0, 255, 255, 255);
	// Grass : green
	case SAMPLE_POLYAREA_GRASS:
		return duRGBA(0, 255, 0, 255);
	// Jump : yellow
	case SAMPLE_POLYAREA_JUMP:
		return duRGBA(255, 255, 0, 255);
	// Unexpected : red
	default:
		return duRGBA(255, 0, 0, 255);
	}
}

Sample::Sample() : navMeshDrawFlags(DU_DRAWNAVMESH_OFFMESHCONS | DU_DRAWNAVMESH_CLOSEDLIST)
{
	resetCommonSettings();
	navQuery = dtAllocNavMeshQuery();
	crowd = dtAllocCrowd();
}

Sample::~Sample()
{
	dtFreeNavMeshQuery(navQuery);
	dtFreeNavMesh(navMesh);
	dtFreeCrowd(crowd);
	delete tool;
	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		delete toolStates[i];
	}
}

void Sample::setTool(SampleTool* newTool)
{
	delete tool;
	tool = newTool;
	if (tool)
	{
		tool->init(this);
	}
}

void Sample::drawSettingsUI() {}

void Sample::drawToolsUI() {}

void Sample::drawDebugUI() {}

void Sample::render()
{
	if (!inputGeometry)
	{
		return;
	}

	// Draw mesh
	duDebugDrawTriMesh(
		&debugDraw,
		inputGeometry->mesh.verts.data(),
		inputGeometry->mesh.getVertCount(),
		inputGeometry->mesh.tris.data(),
		inputGeometry->mesh.normals.data(),
		inputGeometry->mesh.getTriCount(),
		0,
		1.0f);
	// Draw bounds
	duDebugDrawBoxWire(
		&debugDraw,
		inputGeometry->meshBoundsMin[0],
		inputGeometry->meshBoundsMin[1],
		inputGeometry->meshBoundsMin[2],
		inputGeometry->meshBoundsMax[0],
		inputGeometry->meshBoundsMax[1],
		inputGeometry->meshBoundsMax[2],
		duRGBA(255, 255, 255, 128),
		1.0f);
}

void Sample::renderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/) {}

void Sample::onMeshChanged(InputGeom* geom)
{
	inputGeometry = geom;

	if (const BuildSettings* buildSettings = geom->getBuildSettings())
	{
		cellSize = buildSettings->cellSize;
		cellHeight = buildSettings->cellHeight;
		agentHeight = buildSettings->agentHeight;
		agentRadius = buildSettings->agentRadius;
		agentMaxClimb = buildSettings->agentMaxClimb;
		agentMaxSlope = buildSettings->agentMaxSlope;
		regionMinSize = buildSettings->regionMinSize;
		regionMergeSize = buildSettings->regionMergeSize;
		edgeMaxLen = buildSettings->edgeMaxLen;
		edgeMaxError = buildSettings->edgeMaxError;
		detailSampleDist = buildSettings->detailSampleDist;
		detailSampleMaxError = buildSettings->detailSampleMaxError;
		partitionType = buildSettings->partitionType;
	}
}

void Sample::collectSettings(BuildSettings& settings)
{
	settings.cellSize = cellSize;
	settings.cellHeight = cellHeight;
	settings.agentHeight = agentHeight;
	settings.agentRadius = agentRadius;
	settings.agentMaxClimb = agentMaxClimb;
	settings.agentMaxSlope = agentMaxSlope;
	settings.regionMinSize = regionMinSize;
	settings.regionMergeSize = regionMergeSize;
	settings.edgeMaxLen = edgeMaxLen;
	settings.edgeMaxError = edgeMaxError;
	settings.detailSampleDist = detailSampleDist;
	settings.detailSampleMaxError = detailSampleMaxError;
	settings.partitionType = partitionType;
}

void Sample::resetCommonSettings()
{
	cellSize = 0.3f;
	cellHeight = 0.2f;
	agentHeight = 2.0f;
	agentRadius = 0.6f;
	agentMaxClimb = 0.9f;
	agentMaxSlope = 45.0f;
	regionMinSize = 8;
	regionMergeSize = 20;
	edgeMaxLen = 12.0f;
	edgeMaxError = 1.3f;
	vertsPerPoly = 6;
	detailSampleDist = 6.0f;
	detailSampleMaxError = 1.0f;
	partitionType = SAMPLE_PARTITION_WATERSHED;
}

void Sample::drawCommonSettingsUI()
{
	ImGui::SeparatorText("Rasterization");
	DrawFloatSlider(&cellSize, 0.1f, 1.0f, "##Cell Size", "Cell Size", "%.2f");
	DrawFloatSlider(&cellHeight, 0.1f, 1.0f, "##Cell Height", "Cell Height", "%.2f");

	if (inputGeometry)
	{
		int gridWidth = 0;
		int gridHeight = 0;
		rcCalcGridSize(
			inputGeometry->getNavMeshBoundsMin(),
			inputGeometry->getNavMeshBoundsMax(),
			cellSize,
			&gridWidth,
			&gridHeight);
		DrawRightAlignedText("Voxels  %d x %d", gridWidth, gridHeight);
	}

	ImGui::SeparatorText("Agent");
	DrawFloatSlider(&agentHeight, 0.1f, 5.0f, "##Height", "Height");
	DrawFloatSlider(&agentRadius, 0.001f, 5.0f, "##Radius", "Radius");
	DrawFloatSlider(&agentMaxClimb, 0.1f, 5.0f,  "##MaxClimb", "Max Climb");
	DrawFloatSlider(&agentMaxSlope, 0.0f, 90.0f,  "##MaxSlope", "Max Slope");

	ImGui::SeparatorText("Region");
	DrawFloatSlider(&regionMinSize, 0.0f, 150.0f, "##Min Region Size", "Min Region Size");
	DrawFloatSlider(&regionMergeSize, 0.0f, 150.0f, "##Merged Region Size", "Merged Region Size");

	ImGui::SeparatorText("Partitioning");
	if (ImGui::RadioButton("Watershed", partitionType == SAMPLE_PARTITION_WATERSHED))
	{
		partitionType = SAMPLE_PARTITION_WATERSHED;
	}
	if (ImGui::RadioButton("Monotone", partitionType == SAMPLE_PARTITION_MONOTONE))
	{
		partitionType = SAMPLE_PARTITION_MONOTONE;
	}
	if (ImGui::RadioButton("Layers", partitionType == SAMPLE_PARTITION_LAYERS))
	{
		partitionType = SAMPLE_PARTITION_LAYERS;
	}

	ImGui::SeparatorText("Filtering");
	ImGui::Checkbox("Low Hanging Obstacles", &filterLowHangingObstacles);
	ImGui::Checkbox("Ledge Spans", &filterLedgeSpans);
	ImGui::Checkbox("Walkable Low Height Spans", &filterWalkableLowHeightSpans);

	ImGui::SeparatorText("Polygonization");
	DrawFloatSlider(&edgeMaxLen, 0.0f, 50.0f, "##Max Edge Length", "Max Edge Length");
	DrawFloatSlider(&edgeMaxError, 0.1f, 3.0f, "##Max Edge Error", "Max Edge Error");
	DrawIntSlider(&vertsPerPoly, 3, 12, "##Verts Per Poly", "Verts Per Poly");

	ImGui::SeparatorText("Detail Mesh");
	DrawFloatSlider(&detailSampleDist, 0.0f, 16.0f, "##Sample Distance", "Sample Distance");
	DrawFloatSlider(&detailSampleMaxError, 0.0f, 16.0f, "##Max Sample Error", "Max Sample Error");

	ImGui::Separator();
}

void Sample::onClick(const float* rayStartPos, const float* rayHitPos, bool shift)
{
	if (tool)
	{
		tool->onClick(rayStartPos, rayHitPos, shift);
	}
}

void Sample::onToggle()
{
	if (tool)
	{
		tool->onToggle();
	}
}

void Sample::singleStep()
{
	if (tool)
	{
		tool->singleStep();
	}
}

bool Sample::build()
{
	return true;
}

void Sample::update(const float dt)
{
	if (tool)
	{
		tool->update(dt);
	}
	updateToolStates(dt);
}

void Sample::updateToolStates(const float dt) const
{
	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		if (toolStates[i])
		{
			toolStates[i]->handleUpdate(dt);
		}
	}
}

void Sample::initToolStates(Sample* sample) const
{
	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		if (toolStates[i])
		{
			toolStates[i]->init(sample);
		}
	}
}

void Sample::resetToolStates() const
{
	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		if (toolStates[i])
		{
			toolStates[i]->reset();
		}
	}
}

void Sample::renderToolStates() const
{
	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		if (toolStates[i])
		{
			toolStates[i]->handleRender();
		}
	}
}

void Sample::renderOverlayToolStates(double* proj, double* model, int* view) const
{
	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		if (toolStates[i])
		{
			toolStates[i]->handleRenderOverlay(proj, model, view);
		}
	}
}

dtNavMesh* Sample::loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp)
	{
		return 0;
	}

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		return nullptr;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return nullptr;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return nullptr;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return nullptr;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return nullptr;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			fclose(fp);
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
		{
			break;
		}

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data)
		{
			break;
		}
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data);
			fclose(fp);
			return 0;
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);

	return mesh;
}

void Sample::saveAll(const char* path, const dtNavMesh* mesh)
{
	if (!mesh)
	{
		return;
	}

	FILE* fp = fopen(path, "wb");
	if (!fp)
	{
		return;
	}

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize)
		{
			continue;
		}
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize)
		{
			continue;
		}

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}
