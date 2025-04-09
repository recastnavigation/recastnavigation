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

#pragma once

#include <cstdint>

#include "Recast.h"
#include "Sample.h"

class Sample_TileMesh : public Sample
{
protected:
	bool m_buildAll = true;
	float m_totalBuildTimeMs = 0.0f;

	unsigned char* m_triareas = nullptr;
	rcHeightfield* m_heightfield = nullptr;
	rcCompactHeightfield* m_compactHeightfield = nullptr;
	rcContourSet* m_contourSet = nullptr;
	rcPolyMesh* m_polyMesh = nullptr;
	rcPolyMeshDetail* m_detailPolyMesh = nullptr;
	rcConfig m_config {};

	enum class DrawMode : uint8_t
	{
		NAVMESH,
		NAVMESH_TRANS,
		NAVMESH_BVTREE,
		NAVMESH_NODES,
		NAVMESH_PORTALS,
		NAVMESH_INVIS,
		MESH,
		VOXELS,
		VOXELS_WALKABLE,
		COMPACT,
		COMPACT_DISTANCE,
		COMPACT_REGIONS,
		REGION_CONNECTIONS,
		RAW_CONTOURS,
		BOTH_CONTOURS,
		CONTOURS,
		POLYMESH,
		POLYMESH_DETAIL
	};
	DrawMode m_drawMode = DrawMode::NAVMESH;

	int m_maxTiles = 0;
	int m_maxPolysPerTile = 0;
	float m_tileSize = 32.0f;

	unsigned int m_tileColor = duRGBA(0,0,0,32);
	float m_lastBuiltTileBoundsMin[3] = { 0.0f, 0.0f, 0.0f };
	float m_lastBuiltTileBoundsMax[3] = { 0.0f, 0.0f, 0.0f };
	float m_tileBuildTime = 0.0f;
	float m_tileMemUsage = 0.0f;
	int m_tileTriCount = 0;

	unsigned char* buildTileMesh(int tileX, int tileY, const float* boundsMin, const float* boundsMax, int& outDataSize);

	void cleanup();
	void UI_DrawModeOption(const char* name, DrawMode drawMode, bool enabled);

public:
	Sample_TileMesh();
	~Sample_TileMesh() override;
	Sample_TileMesh(const Sample_TileMesh&) = delete;
	Sample_TileMesh& operator=(const Sample_TileMesh&) = delete;
	Sample_TileMesh(const Sample_TileMesh&&) = delete;
	Sample_TileMesh& operator=(const Sample_TileMesh&&) = delete;

	void handleSettings() override;
	void handleTools() override;
	void handleDebugMode() override;
	void handleRender() override;
	void handleRenderOverlay(double* proj, double* model, int* view) override;
	void handleMeshChanged(InputGeom* geom) override;
	bool handleBuild() override;
	void collectSettings(BuildSettings& settings) override;

	void getTilePos(const float* pos, int& tileX, int& tileY) const;

	void buildTile(const float* pos);
	void removeTile(const float* pos);
	void buildAllTiles();
	void removeAllTiles() const;
};
