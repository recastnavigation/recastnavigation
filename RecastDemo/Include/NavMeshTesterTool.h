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

#ifndef NAVMESHTESTERTOOL_H
#define NAVMESHTESTERTOOL_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

class NavMeshTesterTool : public SampleTool
{
protected:
    Sample* m_sample;

    dtNavMesh* m_navMesh;
    dtNavMeshQuery* m_navQuery;

    dtQueryFilter m_filter;

    dtStatus m_pathFindStatus;

    enum ToolMode
    {
        TOOLMODE_PATHFIND_FOLLOW,
        TOOLMODE_PATHFIND_STRAIGHT,
        TOOLMODE_PATHFIND_SLICED,
        TOOLMODE_RAYCAST,
        TOOLMODE_DISTANCE_TO_WALL,
        TOOLMODE_FIND_POLYS_IN_CIRCLE,
        TOOLMODE_FIND_POLYS_IN_SHAPE,
        TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD
    };

    ToolMode m_toolMode;

    int m_straightPathOptions;

    static constexpr int MAX_POLYS = 256;
    static constexpr int MAX_SMOOTH = 2048;

    dtPolyRef m_startRef;
    dtPolyRef m_endRef;
    dtPolyRef m_polys[MAX_POLYS];
    dtPolyRef m_parent[MAX_POLYS];
    int m_npolys;
    float m_straightPath[MAX_POLYS * 3];
    unsigned char m_straightPathFlags[MAX_POLYS];
    dtPolyRef m_straightPathPolys[MAX_POLYS];
    int m_nstraightPath;
    float m_polyPickExt[3]{};
    float m_smoothPath[MAX_SMOOTH * 3];
    int m_nsmoothPath;
    float m_queryPoly[4 * 3];

    static constexpr int MAX_RAND_POINTS = 64;
    float m_randPoints[MAX_RAND_POINTS * 3];
    int m_nrandPoints;
    bool m_randPointsInCircle;

    float m_spos[3];
    float m_epos[3];
    float m_hitPos[3];
    float m_hitNormal[3];
    bool m_hitResult;
    float m_distanceToWall;
    float m_neighbourhoodRadius;
    float m_randomRadius;
    bool m_sposSet;
    bool m_eposSet;

    int m_pathIterNum;
    dtPolyRef m_pathIterPolys[MAX_POLYS];
    int m_pathIterPolyCount;
    float m_prevIterPos[3], m_iterPos[3], m_steerPos[3], m_targetPos[3];

    static constexpr int MAX_STEER_POINTS = 10;
    float m_steerPoints[MAX_STEER_POINTS * 3];
    int m_steerPointCount;

public:
    NavMeshTesterTool();

    int type() override { return TOOL_NAVMESH_TESTER; }
    void init(Sample* sample) override;
    void reset() override;
    void handleMenu() override;
    void handleClick(const float* s, const float* p, bool shift) override;
    void handleToggle() override;
    void handleStep() override;
    void handleUpdate(float dt) override;
    void handleRender() override;
    void handleRenderOverlay(double* proj, double* model, int* view) override;

    virtual void recalc();

    void drawAgent(const float* pos, float r, float h, float c, unsigned int col) const;
};

class UnitedSizeNavMeshTesterTool final : public NavMeshTesterTool
{
    float agentSize = 0;

public:
    void handleMenu() override;
    void handleToggle() override;
    void recalc() override;
};
#endif // NAVMESHTESTERTOOL_H
