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

#include "SampleInterfaces.h"

#include <cstdint>

class Sample;
class InputGeom;
class dtNavMesh;
class dtNavMeshQuery;
class dtCrowd;

/// Tool types.
enum class SampleToolType : uint8_t
{
	NONE = 0,

	TILE_EDIT = 1,
	TILE_HIGHLIGHT = 2,
	TEMP_OBSTACLE = 3,
	NAVMESH_TESTER = 4,
	NAVMESH_PRUNE = 5,
	OFFMESH_CONNECTION = 6,
	CONVEX_VOLUME = 7,
	CROWD = 8,

	MAX_TOOLS
};

static const char* toolNames[] = {
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

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP
};
enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK = 1 << 0,      // 0x01,		// Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM = 1 << 1,      // 0x02,		// Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR = 1 << 2,      // 0x04,		// Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP = 1 << 3,      // 0x08,		// Ability to jump.
	SAMPLE_POLYFLAGS_DISABLED = 1 << 4,  // 0x10,		// Disabled polygon
	SAMPLE_POLYFLAGS_ALL = ~0            // 0xffff	// All abilities.
};

enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED,
	SAMPLE_PARTITION_MONOTONE,
	SAMPLE_PARTITION_LAYERS
};

class SampleDebugDraw : public DebugDrawGL
{
public:
	unsigned int areaToCol(unsigned int area) override;
};

struct SampleTool
{
	virtual ~SampleTool() = default;

	virtual SampleToolType type() = 0;
	virtual void init(Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleMenu() = 0;
	virtual void handleClick(const float* rayStartPos, const float* rayHitPos, bool shift) = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleToggle() = 0;
	virtual void handleStep() = 0;
	virtual void handleUpdate(float dt) = 0;
};

struct SampleToolState
{
	virtual ~SampleToolState() = default;
	virtual void init(Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleUpdate(float dt) = 0;
};

class Sample
{
public:
	InputGeom* inputGeometry = nullptr;
	dtNavMesh* navMesh = nullptr;
	dtNavMeshQuery* navQuery = nullptr;
	dtCrowd* crowd = nullptr;
	SampleDebugDraw debugDraw;

	unsigned char navMeshDrawFlags;

	float cellSize;
	float cellHeight;
	float agentHeight;
	float agentRadius;
	float agentMaxClimb;
	float agentMaxSlope;
	float regionMinSize;
	float regionMergeSize;
	float edgeMaxLen;
	float edgeMaxError;
	int vertsPerPoly;
	float detailSampleDist;
	float detailSampleMaxError;
	int partitionType;

	bool filterLowHangingObstacles = true;
	bool filterLedgeSpans = true;
	bool filterWalkableLowHeightSpans = true;

	SampleTool* tool = nullptr;
	SampleToolState* toolStates[static_cast<size_t>(SampleToolType::MAX_TOOLS)] = {};

	BuildContext* buildContext = nullptr;

	dtNavMesh* loadAll(const char* path);
	void saveAll(const char* path, const dtNavMesh* mesh);

	Sample();
	virtual ~Sample();
	Sample(const Sample&) = delete;
	Sample(const Sample&&) = delete;
	Sample& operator=(const Sample&) = delete;
	Sample& operator=(const Sample&&) = delete;

	void setTool(SampleTool* tool);

	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	virtual void handleClick(const float* rayStartPos, const float* rayHitPos, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(InputGeom* geom);
	virtual bool handleBuild();
	virtual void handleUpdate(float dt);
	virtual void collectSettings(struct BuildSettings& settings);

	void updateToolStates(float dt) const;
	void initToolStates(Sample* sample) const;
	void resetToolStates() const;
	void renderToolStates() const;
	void renderOverlayToolStates(double* proj, double* model, int* view) const;

	void resetCommonSettings();
	void handleCommonSettings();
};
