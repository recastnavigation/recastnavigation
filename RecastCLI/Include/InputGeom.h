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

#ifndef INPUTGEOM_H
#define INPUTGEOM_H

#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"
#include "Recast.h"

static constexpr int MAX_CONVEXVOL_PTS = 12;

struct ConvexVolume
{
    float verts[MAX_CONVEXVOL_PTS * 3];
    float hmin, hmax;
    int nverts;
    int area;
};

struct BuildSettings
{
    // Cell size in world units
    float cellSize;
    // Cell height in world units
    float cellHeight;
    // Agent height in world units
    float agentHeight;
    // Agent radius in world units
    float agentRadius;
    // Agent max climb in world units
    float agentMaxClimb;
    // Agent max slope in degrees
    float agentMaxSlope;
    // Region minimum size in voxels.
    // regionMinSize = sqrt(regionMinArea)
    float regionMinSize;
    // Region merge size in voxels.
    // regionMergeSize = sqrt(regionMergeArea)
    float regionMergeSize;
    // Edge max length in world units
    float edgeMaxLen;
    // Edge max error in voxels
    float edgeMaxError;
    float vertsPerPoly;
    // Detail sample distance in voxels
    float detailSampleDist;
    // Detail sample max error in voxel heights.
    float detailSampleMaxError;
    // Partition type, see SamplePartitionType
    int partitionType;
    // Bounds of the area to mesh
    float navMeshBMin[3];
    float navMeshBMax[3];
    // Size of the tiles in voxels
    float tileSize;
};

class InputGeom
{
    rcChunkyTriMesh* m_chunkyMesh;
    rcMeshLoaderObj* m_mesh;
    float m_meshBMin[3], m_meshBMax[3];
    BuildSettings m_buildSettings;
    bool m_hasBuildSettings;

    /// @name Off-Mesh connections.
    ///@{
    static constexpr int MAX_OFFMESH_CONNECTIONS = 256;
    float m_offMeshConVerts[MAX_OFFMESH_CONNECTIONS * 3 * 2];
    float m_offMeshConRads[MAX_OFFMESH_CONNECTIONS];
    unsigned char m_offMeshConDirs[MAX_OFFMESH_CONNECTIONS];
    unsigned char m_offMeshConAreas[MAX_OFFMESH_CONNECTIONS];
    unsigned short m_offMeshConFlags[MAX_OFFMESH_CONNECTIONS];
    unsigned int m_offMeshConId[MAX_OFFMESH_CONNECTIONS];
    int m_offMeshConCount;
    ///@}

    /// @name Convex Volumes.
    ///@{
    static constexpr int MAX_VOLUMES = 256;
    ConvexVolume m_volumes[MAX_VOLUMES];
    int m_volumeCount;
    ///@}

    bool loadMesh(rcContext* ctx, const std::string& filepath);
    bool loadGeomSet(rcContext* ctx, const std::string& filepath);

public:
    InputGeom();
    ~InputGeom();


    bool load(rcContext* ctx, const std::string& filepath);
    bool saveGeomSet(const BuildSettings* settings) const;

    /// Method to return static mesh data.
    [[nodiscard]] const rcMeshLoaderObj* getMesh() const { return m_mesh; }
    [[nodiscard]] const float* getMeshBoundsMin() const { return m_meshBMin; }
    [[nodiscard]] const float* getMeshBoundsMax() const { return m_meshBMax; }

    [[nodiscard]] const float* getNavMeshBoundsMin() const
    {
        return m_hasBuildSettings ? m_buildSettings.navMeshBMin : m_meshBMin;
    }

    [[nodiscard]] const float* getNavMeshBoundsMax() const
    {
        return m_hasBuildSettings ? m_buildSettings.navMeshBMax : m_meshBMax;
    }

    [[nodiscard]] const rcChunkyTriMesh* getChunkyMesh() const { return m_chunkyMesh; }
    [[nodiscard]] const BuildSettings* getBuildSettings() const { return m_hasBuildSettings ? &m_buildSettings : nullptr; }
    bool raycastMesh(const float* src, const float* dst, float& tmin) const;

    /// @name Off-Mesh connections.
    ///@{
    [[nodiscard]] int getOffMeshConnectionCount() const { return m_offMeshConCount; }
    [[nodiscard]] const float* getOffMeshConnectionVerts() const { return m_offMeshConVerts; }
    [[nodiscard]] const float* getOffMeshConnectionRads() const { return m_offMeshConRads; }
    [[nodiscard]] const unsigned char* getOffMeshConnectionDirs() const { return m_offMeshConDirs; }
    [[nodiscard]] const unsigned char* getOffMeshConnectionAreas() const { return m_offMeshConAreas; }
    [[nodiscard]] const unsigned short* getOffMeshConnectionFlags() const { return m_offMeshConFlags; }
    [[nodiscard]] const unsigned int* getOffMeshConnectionId() const { return m_offMeshConId; }
    void addOffMeshConnection(const float* spos, const float* epos, float rad,
                                            unsigned char bidir, unsigned char area, unsigned short flags);
    void deleteOffMeshConnection(int i);
    void drawOffMeshConnections(struct duDebugDraw* dd, bool hilight = false) const;
    ///@}

    /// @name Box Volumes.
    ///@{
    [[nodiscard]] int getConvexVolumeCount() const { return m_volumeCount; }
    [[nodiscard]] const ConvexVolume* getConvexVolumes() const { return m_volumes; }
    void addConvexVolume(const float* verts, int nverts,
                         float minh, float maxh, unsigned char area);
    void deleteConvexVolume(int i);
    void drawConvexVolumes(duDebugDraw* dd, bool hilight = false) const;
    ///@}

private:
    // Explicitly disabled copy constructor and copy assignment operator.
    InputGeom(const InputGeom&);
    InputGeom& operator=(const InputGeom&);
};

#endif // INPUTGEOM_H
