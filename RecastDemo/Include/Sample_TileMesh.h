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

#include "Recast.h"
#include "Sample.h"

#include <cstdint>

class Sample_TileMesh : public Sample
{
private:
	bool buildAll = true;
	float totalBuildTimeMs = 0.0f;

	// Recast state
	rcConfig config{};
	unsigned char* triareas = nullptr;
	rcHeightfield* heightfield = nullptr;
	rcCompactHeightfield* compactHeightfield = nullptr;
	rcContourSet* contourSet = nullptr;
	rcPolyMesh* polyMesh = nullptr;
	rcPolyMeshDetail* detailPolyMesh = nullptr;

	enum class DrawMode : uint8_t
	{
		MESH,
		NAVMESH,
		NAVMESH_INVIS,
		NAVMESH_TRANS,
		NAVMESH_BVTREE,
		NAVMESH_NODES,
		NAVMESH_PORTALS,
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
	DrawMode drawMode = DrawMode::NAVMESH;
	static const char* drawModeNames[];

	int maxTiles = 0;
	int maxPolysPerTile = 0;
	/// In cells
	int tileSize = 32;

	unsigned int tileColor = duRGBA(0, 0, 0, 32);
	float lastBuiltTileBoundsMin[3] = {0.0f, 0.0f, 0.0f};
	float lastBuiltTileBoundsMax[3] = {0.0f, 0.0f, 0.0f};
	float tileBuildTime = 0.0f;
	float tileMemUsage = 0.0f;
	int tileTriCount = 0;

	unsigned char* buildTileMesh(int tileX, int tileY, const float* boundsMin, const float* boundsMax, int& outDataSize);

	void cleanup();
	void UI_DrawModeOption(DrawMode drawMode, bool enabled);

public:
	Sample_TileMesh();
	~Sample_TileMesh() override;
	Sample_TileMesh(const Sample_TileMesh&) = delete;
	Sample_TileMesh& operator=(const Sample_TileMesh&) = delete;
	Sample_TileMesh(const Sample_TileMesh&&) = delete;
	Sample_TileMesh& operator=(const Sample_TileMesh&&) = delete;

	// Sample methods
	void handleSettings() override;
	void handleTools() override;
	void drawDebugUI() override;
	void render() override;
	void renderOverlay(double* proj, double* model, int* view) override;
	void onMeshChanged(InputGeom* geom) override;
	bool build() override;
	void collectSettings(BuildSettings& settings) override;

	void getTilePos(const float* pos, int& outTileX, int& outTileY) const;

	void buildTile(const float* pos);
	void removeTile(const float* pos);
	void buildAllTiles();
	void removeAllTiles() const;
};
