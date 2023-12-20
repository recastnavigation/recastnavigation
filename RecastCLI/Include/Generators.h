//
// Created by joran on 15/12/2023.
//

#pragma once

struct rcConfig;
class InputGeom;
class rcContext;

bool GenerateTheses(rcContext* pCtx, const InputGeom* pGeom, rcConfig& config, bool filterLowHangingObstacles,
                    bool filterLedgeSpans, bool filterWalkableLowHeightSpans, float& totalBuildTimeMs);

bool GenerateSingleMeshWaterShed(rcContext* pCtx, const InputGeom* pGeom, rcConfig& config,
                                 bool filterLowHangingObstacles, bool filterLedgeSpans,
                                 bool filterWalkableLowHeightSpans, float& totalBuildTimeMs);