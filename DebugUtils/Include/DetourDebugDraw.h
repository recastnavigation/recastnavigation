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
#include <DetourNavMesh.h>
#include <cstdint>

struct dtTileCachePolyMesh;
struct dtTileCacheContourSet;
struct dtTileCacheLayer;
class dtNavMeshQuery;
class dtNavMesh;
struct duDebugDraw;
enum DrawNavMeshFlags
{
	DU_DRAWNAVMESH_OFFMESHCONS = 0x01,
	DU_DRAWNAVMESH_CLOSEDLIST = 0x02,
	DU_DRAWNAVMESH_COLOR_TILES = 0x04
};

void duDebugDrawNavMesh(duDebugDraw* dd, const dtNavMesh& mesh, uint8_t flags);
void duDebugDrawNavMeshWithClosedList(duDebugDraw* dd, const dtNavMesh& mesh, const dtNavMeshQuery& query, uint8_t flags);
void duDebugDrawNavMeshNodes(duDebugDraw* dd, const dtNavMeshQuery& query);
void duDebugDrawNavMeshBVTree(duDebugDraw* dd, const dtNavMesh& mesh);
void duDebugDrawNavMeshPortals(duDebugDraw* dd, const dtNavMesh& mesh);
void duDebugDrawNavMeshPolysWithFlags(duDebugDraw* dd, const dtNavMesh& mesh, uint16_t polyFlags, uint32_t col);
void duDebugDrawNavMeshPoly(duDebugDraw* dd, const dtNavMesh& mesh, dtPolyRef ref, uint32_t col);

void duDebugDrawTileCacheLayerAreas(duDebugDraw* dd, const dtTileCacheLayer& layer, float cs, float ch);
void duDebugDrawTileCacheLayerRegions(duDebugDraw* dd, const dtTileCacheLayer& layer, float cs, float ch);
void duDebugDrawTileCacheContours(duDebugDraw* dd, const dtTileCacheContourSet& lcset, const float* orig, float cs, float ch);
void duDebugDrawTileCachePolyMesh(duDebugDraw* dd, const dtTileCachePolyMesh& lmesh, const float* orig, float cs, float ch);
