//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
// Dmitrii Shkarovskii scriper.proger@gmail.com
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

// Use in quick check passability from one polygon to another polygon
// This system works on compares polygon island indices
// To activate this system you need uncomment define NAVMESH_ISLAND_SYSTEM 

#ifndef DETOURISLANDSYSTEM_H
#define DETOURISLANDSYSTEM_H

#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#ifdef NAVMESH_ISLAND_SYSTEM

class dtIslandManager
{
public:
	dtIslandManager(dtNavMesh* navMesh);

	void splitMeshToIslands(const dtQueryFilter* filter);
	void splitTileToIslands(const int x, const int y, const dtQueryFilter* filter);

private:
	void fillPolysWithIslandIdx(unsigned int fromIdx, unsigned int toIdx, bool force);
	void splitTileToIslands(dtNavMeshQuery* query, dtPolyRef* nearestPolys, int tileNum, const dtMeshTile* tile, const dtQueryFilter* filter);
	void resetIndexes();

	dtNavMeshQuery* getQuery() const;
	void freeQuery(dtNavMeshQuery* query) const;

private:
	unsigned int m_freeIslandIdx;
	const dtNavMesh* m_navMesh;
};
#endif

#endif // DETOURISLANDSYSTEM_H