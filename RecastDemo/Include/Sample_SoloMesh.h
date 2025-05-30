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

class Sample_SoloMesh : public Sample
{
protected:
	float totalBuildTimeMs = 0;

	rcConfig config {};

	unsigned char* triareas = nullptr;
	rcHeightfield* heightfield = nullptr;
	rcCompactHeightfield* compactHeightfield = nullptr;
	rcContourSet* contourSet = nullptr;
	rcPolyMesh* polyMesh = nullptr;
	rcPolyMeshDetail* detailMesh = nullptr;

	enum class DrawMode : uint8_t
	{
		NAVMESH,
		NAVMESH_TRANS,
		NAVMESH_BVTREE,
		NAVMESH_NODES,
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
	DrawMode currentDrawMode = DrawMode::NAVMESH;

	void cleanup();

public:
	Sample_SoloMesh();
	~Sample_SoloMesh() override;
	Sample_SoloMesh(const Sample_SoloMesh&) = delete;
	Sample_SoloMesh& operator=(const Sample_SoloMesh&) = delete;
	Sample_SoloMesh(const Sample_SoloMesh&&) = delete;
	Sample_SoloMesh& operator=(const Sample_SoloMesh&&) = delete;

	void handleSettings() override;
	void handleTools() override;
	void handleDebugMode() override;

	void handleRender() override;
	void handleRenderOverlay(double* proj, double* model, int* view) override;
	void handleMeshChanged(InputGeom* geom) override;
	bool handleBuild() override;

private:
	void UI_DrawModeOption(const char* name, DrawMode drawMode, bool enabled);
};
