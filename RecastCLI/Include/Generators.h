//
// Created by joran on 15/12/2023.
//

#pragma once

struct rcPolyMeshDetail;
struct rcPolyMesh;
struct rcConfig;
class InputGeom;
class rcContext;

bool GenerateTheses(rcContext* pCtx, const InputGeom* pGeom, rcConfig& config, bool filterLowHangingObstacles,
                    bool filterLedgeSpans, bool filterWalkableLowHeightSpans, float& totalBuildTimeMs,
                    rcPolyMesh*& m_pmesh, rcPolyMeshDetail*& m_dmesh);

bool GenerateSingleMeshWaterShed(rcContext* pCtx, const InputGeom* pGeom, rcConfig& config,
                                 bool filterLowHangingObstacles, bool filterLedgeSpans,
                                 bool filterWalkableLowHeightSpans, float& totalBuildTimeMs,
                                 rcPolyMesh*& m_pmesh, rcPolyMeshDetail*& m_dmesh);
