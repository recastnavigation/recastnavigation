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

#include <string>
#include <vector>

struct PartitionedMesh;
class rcContext;
struct duDebugDraw;

static constexpr int MAX_CONVEXVOL_PTS = 12;
struct ConvexVolume
{
	float verts[MAX_CONVEXVOL_PTS * 3];
	float hmin;
	float hmax;
	int nverts;
	int area;
};

struct BuildSettings
{
	/// Cell size in world units
	float cellSize = 0;
	/// Cell height in world units
	float cellHeight = 0;
	/// Agent height in world units
	float agentHeight = 0;
	/// Agent radius in world units
	float agentRadius = 0;
	/// Agent max climb in world units
	float agentMaxClimb = 0;
	/// Agent max slope in degrees
	float agentMaxSlope = 0;
	/// Region minimum size in voxels.
	float regionMinSize = 0;
	/// Region merge size in voxels. regionMergeSize = sqrt(regionMergeArea)
	float regionMergeSize = 0;
	/// Edge max length in world units
	float edgeMaxLen = 0;
	/// Edge max error in voxels
	float edgeMaxError = 0;
	int vertsPerPoly = 0;
	/// Detail sample distance in voxels
	float detailSampleDist = 0;
	/// Detail sample max error in voxel heights.
	float detailSampleMaxError = 0;
	/// Partition type, see SamplePartitionType
	int partitionType = 0;
	/// Bounds of the area to mesh
	float navMeshBMin[3]{};
	float navMeshBMax[3]{};
	/// Size of the tiles in voxels
	float tileSize = 0;
};

struct Mesh
{
	std::vector<float> verts;
	std::vector<int> tris;
	std::vector<float> normals;

	[[nodiscard]] int getVertCount() const { return static_cast<int>(verts.size()) / 3; }
	[[nodiscard]] int getTriCount() const { return static_cast<int>(tris.size()) / 3; }
};

class InputGeom
{
public:
	std::string filename;
	Mesh mesh;

	PartitionedMesh* partitionedMesh = nullptr;
	float meshBoundsMin[3] = {};
	float meshBoundsMax[3] = {};

private:
	BuildSettings buildSettings;
	bool hasBuildSettings = false;

public:
	/// @name Off-Mesh connections.
	///@{
	static constexpr int MAX_OFFMESH_CONNECTIONS = 256;
	float offMeshConVerts[MAX_OFFMESH_CONNECTIONS * 3 * 2];
	float offMeshConRads[MAX_OFFMESH_CONNECTIONS];
	unsigned char offMeshConDirs[MAX_OFFMESH_CONNECTIONS];
	unsigned char offMeshConAreas[MAX_OFFMESH_CONNECTIONS];
	unsigned short offMeshConFlags[MAX_OFFMESH_CONNECTIONS];
	unsigned int offMeshConId[MAX_OFFMESH_CONNECTIONS];
	int offMeshConCount = 0;
	///@}

	/// @name Convex Volumes.
	///@{
	static const int MAX_VOLUMES = 256;
	ConvexVolume convexVolumes[MAX_VOLUMES];
	int convexVolumeCount = 0;
	///@}

	InputGeom() = default;
	~InputGeom();
	InputGeom(const InputGeom&) = delete;
	InputGeom& operator=(const InputGeom&) = delete;
	InputGeom(InputGeom&&) = delete;
	InputGeom& operator=(InputGeom&&) = delete;

	bool load(rcContext* ctx, const std::string& filepath);
	bool saveGeomSet(const BuildSettings* settings);

	/// Method to return static mesh data.
	[[nodiscard]] const float* getNavMeshBoundsMin() const { return hasBuildSettings ? buildSettings.navMeshBMin : meshBoundsMin; }
	[[nodiscard]] const float* getNavMeshBoundsMax() const { return hasBuildSettings ? buildSettings.navMeshBMax : meshBoundsMax; }
	[[nodiscard]] const BuildSettings* getBuildSettings() const { return hasBuildSettings ? &buildSettings : nullptr; }
	bool raycastMesh(float* src, float* dst, float& tmin);

	/// @name Off-Mesh connections.
	///@{
	void addOffMeshConnection(const float* startPos, const float* endPos, float rad, unsigned char bidir, unsigned char area, unsigned short flags);
	void deleteOffMeshConnection(int i);
	void drawOffMeshConnections(duDebugDraw* dd, bool highlight = false);
	///@}

	/// @name Box Volumes.
	///@{
	void addConvexVolume(const float* verts, int nverts, float minh, float maxh, unsigned char area);
	void deleteConvexVolume(int i);
	void drawConvexVolumes(duDebugDraw* dd, bool highlight = false);
	///@}

private:
	bool loadMesh(rcContext* ctx, const std::string& filepath);
	bool loadGeomSet(rcContext* ctx, const std::string& filepath);
	bool loadGeomSet(rcContext* ctx, char* buffer, size_t bufferLen);
};

