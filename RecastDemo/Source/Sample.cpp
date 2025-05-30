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
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"

#include <math.h>
#include <stdio.h>

#ifdef WIN32
#	define snprintf _snprintf
#endif

SampleTool::~SampleTool()
{
	// Defined out of line to fix the weak v-tables warning
}

SampleToolState::~SampleToolState()
{
	// Defined out of line to fix the weak v-tables warning
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

	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		toolStates[i] = 0;
	}
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

void Sample::setTool(SampleTool* tool)
{
	delete tool;
	tool = tool;
	if (tool)
	{
		tool->init(this);
	}
}

void Sample::handleSettings() {}

void Sample::handleTools() {}

void Sample::handleDebugMode() {}

void Sample::handleRender()
{
	if (!inputGeometry)
	{
		return;
	}

	// Draw mesh
	duDebugDrawTriMesh(
		&debugDraw,
		inputGeometry->getMesh()->getVerts(),
		inputGeometry->getMesh()->getVertCount(),
		inputGeometry->getMesh()->getTris(),
		inputGeometry->getMesh()->getNormals(),
		inputGeometry->getMesh()->getTriCount(),
		0,
		1.0f);
	// Draw bounds
	const float* bmin = inputGeometry->getMeshBoundsMin();
	const float* bmax = inputGeometry->getMeshBoundsMax();
	duDebugDrawBoxWire(&debugDraw, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], duRGBA(255, 255, 255, 128), 1.0f);
}

void Sample::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/) {}

void Sample::handleMeshChanged(InputGeom* geom)
{
	inputGeometry = geom;

	const BuildSettings* buildSettings = geom->getBuildSettings();
	if (buildSettings)
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
		vertsPerPoly = buildSettings->vertsPerPoly;
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
	settings.vertsPerPoly = vertsPerPoly;
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
	vertsPerPoly = 6.0f;
	detailSampleDist = 6.0f;
	detailSampleMaxError = 1.0f;
	partitionType = SAMPLE_PARTITION_WATERSHED;
}

void Sample::handleCommonSettings()
{
	imguiLabel("Rasterization");
	imguiSlider("Cell Size", &cellSize, 0.1f, 1.0f, 0.01f);
	imguiSlider("Cell Height", &cellHeight, 0.1f, 1.0f, 0.01f);

	if (inputGeometry)
	{
		const float* bmin = inputGeometry->getNavMeshBoundsMin();
		const float* bmax = inputGeometry->getNavMeshBoundsMax();
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, cellSize, &gw, &gh);
		char text[64];
		snprintf(text, 64, "Voxels  %d x %d", gw, gh);
		imguiValue(text);
	}

	imguiSeparator();
	imguiLabel("Agent");
	imguiSlider("Height", &agentHeight, 0.1f, 5.0f, 0.1f);
	imguiSlider("Radius", &agentRadius, 0.0f, 5.0f, 0.1f);
	imguiSlider("Max Climb", &agentMaxClimb, 0.1f, 5.0f, 0.1f);
	imguiSlider("Max Slope", &agentMaxSlope, 0.0f, 90.0f, 1.0f);

	imguiSeparator();
	imguiLabel("Region");
	imguiSlider("Min Region Size", &regionMinSize, 0.0f, 150.0f, 1.0f);
	imguiSlider("Merged Region Size", &regionMergeSize, 0.0f, 150.0f, 1.0f);

	imguiSeparator();
	imguiLabel("Partitioning");
	if (imguiCheck("Watershed", partitionType == SAMPLE_PARTITION_WATERSHED))
	{
		partitionType = SAMPLE_PARTITION_WATERSHED;
	}
	if (imguiCheck("Monotone", partitionType == SAMPLE_PARTITION_MONOTONE))
	{
		partitionType = SAMPLE_PARTITION_MONOTONE;
	}
	if (imguiCheck("Layers", partitionType == SAMPLE_PARTITION_LAYERS))
	{
		partitionType = SAMPLE_PARTITION_LAYERS;
	}

	imguiSeparator();
	imguiLabel("Filtering");
	if (imguiCheck("Low Hanging Obstacles", filterLowHangingObstacles))
	{
		filterLowHangingObstacles = !filterLowHangingObstacles;
	}
	if (imguiCheck("Ledge Spans", filterLedgeSpans))
	{
		filterLedgeSpans = !filterLedgeSpans;
	}
	if (imguiCheck("Walkable Low Height Spans", filterWalkableLowHeightSpans))
	{
		filterWalkableLowHeightSpans = !filterWalkableLowHeightSpans;
	}

	imguiSeparator();
	imguiLabel("Polygonization");
	imguiSlider("Max Edge Length", &edgeMaxLen, 0.0f, 50.0f, 1.0f);
	imguiSlider("Max Edge Error", &edgeMaxError, 0.1f, 3.0f, 0.1f);
	imguiSlider("Verts Per Poly", &vertsPerPoly, 3.0f, 12.0f, 1.0f);

	imguiSeparator();
	imguiLabel("Detail Mesh");
	imguiSlider("Sample Distance", &detailSampleDist, 0.0f, 16.0f, 1.0f);
	imguiSlider("Max Sample Error", &detailSampleMaxError, 0.0f, 16.0f, 1.0f);

	imguiSeparator();
}

void Sample::handleClick(const float* s, const float* p, bool shift)
{
	if (tool)
	{
		tool->handleClick(s, p, shift);
	}
}

void Sample::handleToggle()
{
	if (tool)
	{
		tool->handleToggle();
	}
}

void Sample::handleStep()
{
	if (tool)
	{
		tool->handleStep();
	}
}

bool Sample::handleBuild()
{
	return true;
}

void Sample::handleUpdate(const float dt)
{
	if (tool)
	{
		tool->handleUpdate(dt);
	}
	updateToolStates(dt);
}

void Sample::updateToolStates(const float dt)
{
	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		if (toolStates[i])
		{
			toolStates[i]->handleUpdate(dt);
		}
	}
}

void Sample::initToolStates(Sample* sample)
{
	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		if (toolStates[i])
		{
			toolStates[i]->init(sample);
		}
	}
}

void Sample::resetToolStates()
{
	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		if (toolStates[i])
		{
			toolStates[i]->reset();
		}
	}
}

void Sample::renderToolStates()
{
	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		if (toolStates[i])
		{
			toolStates[i]->handleRender();
		}
	}
}

void Sample::renderOverlayToolStates(double* proj, double* model, int* view)
{
	for (int i = 0; i < static_cast<int>(SampleToolType::MAX_TOOLS); i++)
	{
		if (toolStates[i])
		{
			toolStates[i]->handleRenderOverlay(proj, model, view);
		}
	}
}

static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';  //'MSET';
static const int NAVMESHSET_VERSION = 1;

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
