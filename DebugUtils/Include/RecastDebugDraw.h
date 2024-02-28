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

void duDebugDrawTriMesh(duDebugDraw* dd, const float* verts, int nverts, const int* tris, const float* normals, int ntris, const unsigned char* flags, float texScale);
void duDebugDrawTriMeshSlope(duDebugDraw* dd, const float* verts, int nverts, const int* tris, const float* normals, int ntris, float walkableSlopeAngle, float texScale);

void duDebugDrawHeightfieldSolid(duDebugDraw* dd, const struct rcHeightfield& hf);
void duDebugDrawHeightfieldWalkable(duDebugDraw* dd, const rcHeightfield& hf);

void duDebugDrawCompactHeightfieldSolid(duDebugDraw* dd, const struct rcCompactHeightfield& chf);
void duDebugDrawCompactHeightfieldRegions(duDebugDraw* dd, const rcCompactHeightfield& chf);
void duDebugDrawCompactHeightfieldDistance(duDebugDraw* dd, const rcCompactHeightfield& chf);

void duDebugDrawHeightfieldLayer(duDebugDraw* dd, const struct rcHeightfieldLayer& layer, int idx);
void duDebugDrawHeightfieldLayers(duDebugDraw* dd, const struct rcHeightfieldLayerSet& lset);
void duDebugDrawHeightfieldLayersRegions(duDebugDraw* dd, const rcHeightfieldLayerSet& lset);

void duDebugDrawRegionConnections(duDebugDraw* dd, const struct rcContourSet& cset, float alpha = 1.0f);
void duDebugDrawRawContours(duDebugDraw* dd, const rcContourSet& cset, float alpha = 1.0f);
void duDebugDrawContours(duDebugDraw* dd, const rcContourSet& cset, float alpha = 1.0f);
void duDebugDrawPolyMesh(duDebugDraw* dd, const struct rcPolyMesh& mesh);
void duDebugDrawPolyMeshDetail(duDebugDraw* dd, const struct rcPolyMeshDetail& dmesh);
