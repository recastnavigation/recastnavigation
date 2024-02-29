//
// Created by joran on 15/12/2023.
//

#pragma once

struct rcPolyMeshDetail;
struct rcPolyMesh;
struct rcConfig;
class InputGeom;
class rcContext;

bool generateTheses(rcContext& context,const InputGeom& pGeom, rcConfig &config, bool filterLowHangingObstacles,bool filterLedgeSpans, bool filterWalkableLowHeightSpans, rcPolyMesh *&pMesh, rcPolyMeshDetail *&pDetailedMesh, int *&bounderies, int &bounderyElementCount);

bool generateSingle(rcContext& context, const InputGeom& pGeom, rcConfig& config, bool filterLowHangingObstacles, bool filterLedgeSpans, bool filterWalkableLowHeightSpans, rcPolyMesh*& pMesh, rcPolyMeshDetail*& pDetailedMesh);
