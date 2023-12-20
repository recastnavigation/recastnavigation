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

#ifndef DETOURDEBUGDRAW_H
#define DETOURDEBUGDRAW_H

#include "DetourNavMesh.h"
#include "DetourTileCacheBuilder.h"

enum DrawNavMeshFlags
{
	DU_DRAWNAVMESH_OFFMESHCONS = 0x01,
	DU_DRAWNAVMESH_CLOSEDLIST = 0x02,
	DU_DRAWNAVMESH_COLOR_TILES = 0x04
};

void duDebugDrawNavMesh(duDebugDraw* dd, const dtNavMesh& mesh, unsigned char flags);
void duDebugDrawNavMeshWithClosedList(duDebugDraw* dd, const dtNavMesh& mesh, const dtNavMeshQuery& query, unsigned char flags);
void duDebugDrawNavMeshNodes(duDebugDraw* dd, const dtNavMeshQuery& query);
void duDebugDrawNavMeshBVTree(duDebugDraw* dd, const dtNavMesh& mesh);
void duDebugDrawNavMeshPortals(duDebugDraw* dd, const dtNavMesh& mesh);
void duDebugDrawNavMeshPolysWithFlags(duDebugDraw* dd, const dtNavMesh& mesh, unsigned short polyFlags, unsigned int col);
void duDebugDrawNavMeshPoly(duDebugDraw* dd, const dtNavMesh& mesh, dtPolyRef ref, unsigned int col);

void duDebugDrawTileCacheLayerAreas(duDebugDraw* dd, const dtTileCacheLayer& layer, float cs, float ch);
void duDebugDrawTileCacheLayerRegions(duDebugDraw* dd, const dtTileCacheLayer& layer, float cs, float ch);
void duDebugDrawTileCacheContours(duDebugDraw* dd, const dtTileCacheContourSet& lcset,
								  const float* orig, float cs, float ch);
void duDebugDrawTileCachePolyMesh(duDebugDraw* dd, const dtTileCachePolyMesh& lmesh,
								  const float* orig, float cs, float ch);

#endif // DETOURDEBUGDRAW_H
