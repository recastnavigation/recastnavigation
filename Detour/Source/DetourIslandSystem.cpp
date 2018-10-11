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

#include "DetourIslandSystem.h"
#include "DetourCommon.h"

#ifdef NAVMESH_ISLAND_SYSTEM

#define ISLAND_FIND_POLYS_COUNT_MAX 300
#define ISLAND_CHECK_RADIUS 100.0f
#define ISLAND_QUERY_POLYS_MAX 65500

dtIslandManager::dtIslandManager(dtNavMesh* navMesh) :
	m_freeIslandIdx(0),
	m_navMesh(navMesh)
{
	resetIndexes();
}

void dtIslandManager::resetIndexes()
{
	m_freeIslandIdx = 1;
}

dtNavMeshQuery* dtIslandManager::getQuery() const
{
	dtNavMeshQuery* query = dtAllocNavMeshQuery(); 
	query->init(m_navMesh, ISLAND_QUERY_POLYS_MAX);
	return query;
}

void dtIslandManager::freeQuery(dtNavMeshQuery* query) const
{
	dtFreeNavMeshQuery(query);
}

void dtIslandManager::splitMeshToIslands(const dtQueryFilter* filter)
{
	resetIndexes();

	fillPolysWithIslandIdx(0, 0, true);
	dtNavMeshQuery* query = getQuery();
	if (query)
	{
		dtPolyRef nearestPolys[ISLAND_QUERY_POLYS_MAX];
		for (int tileNum = 0; tileNum < m_navMesh->getMaxTiles(); tileNum++)
		{
			const dtMeshTile* tile = m_navMesh->getTile(tileNum);
			if (!tile || !tile->header)
				continue;

			splitTileToIslands(query, nearestPolys, tileNum, tile, filter);
		}
		freeQuery(query);
	}
}

void dtIslandManager::splitTileToIslands(const int x, const int y, const dtQueryFilter* filter)
{
	const dtTileRef tileRef = m_navMesh->getTileRefAt(x, y, 0);
	const int tileNum = m_navMesh->decodePolyIdTile(tileRef);
	const dtMeshTile* tile = m_navMesh->getTile(tileNum);
	if (tile)
	{
		if (tile->header)
		{
			dtNavMeshQuery* query = getQuery();
			if (query)
			{
				dtPolyRef nearestPolys[ISLAND_QUERY_POLYS_MAX];
				splitTileToIslands(query, nearestPolys, tileNum, tile, filter);
				freeQuery(query);
			}
		}
	}
}

void dtIslandManager::fillPolysWithIslandIdx(unsigned int fromIdx, unsigned int toIdx, bool force)
{
	for (int tileNum = 0; tileNum < m_navMesh->getMaxTiles(); tileNum++)
	{
		const dtMeshTile* tile = m_navMesh->getTile(tileNum);
		if (!tile || !tile->header)
			continue;

		for (int polyNum = 0; polyNum < tile->header->polyCount; polyNum++)
		{
			dtPoly* poly = &(tile->polys)[polyNum];
			if (!poly)
				continue;
			if (force || poly->getIslandIdx() == fromIdx)
				poly->setIslandIdx(toIdx);
		}
	}
}

void dtIslandManager::splitTileToIslands(dtNavMeshQuery* query, dtPolyRef* nearestPolys, int tileNum, const dtMeshTile* tile, const dtQueryFilter* filter)
{
	for (int polyNum = 0; polyNum < tile->header->polyCount; polyNum++)
	{
		dtPoly* poly = &(tile->polys)[polyNum];
		if (!poly)
			continue;

		if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) // Skip off-mesh links.
			continue;

		if (poly->getIslandIdx() == 0) // If poly have not island index
		{
			dtPolyRef polyRef = m_navMesh->encodePolyId(tile->salt, tileNum, polyNum);
			float c[3];

			const dtMeshTile* tile = 0;
			const dtPoly* poly = 0;
			dtStatus status = m_navMesh->getTileAndPolyByRef(polyRef, &tile, &poly);
			if (!dtStatusFailed(status))
				dtCalcPolyCenter(c, poly->verts, (int)poly->vertCount, tile->verts);

			int findCount = 0;
			query->findPolysAroundCircle(polyRef, c, ISLAND_CHECK_RADIUS, filter, nearestPolys, 0, 0, &findCount, ISLAND_FIND_POLYS_COUNT_MAX); // Find nearest

			int isladIndex = 0;
			for (int i = 0; i < findCount; i++) // Find poly with island index
			{
				unsigned int islandCheckIndex = 0;
				m_navMesh->getPolyIslandIdx(nearestPolys[i], &islandCheckIndex);
				if (islandCheckIndex != 0)
				{
					isladIndex = islandCheckIndex;
					break;
				}
			}

			if (isladIndex == 0) // If not find then generate new index
			{
				isladIndex = m_freeIslandIdx;
				m_freeIslandIdx++;
				if (m_freeIslandIdx <= 0)
					m_freeIslandIdx = 1;
			}

			((dtPoly*)poly)->setIslandIdx(isladIndex); // Set island index

			for (int i = 0; i < findCount; i++)
			{
				const dtPolyRef nearestPoly = nearestPolys[i];
				unsigned int islandCheckIndex = 0;
				m_navMesh->getPolyIslandIdx(nearestPoly, &islandCheckIndex);
				if (islandCheckIndex != 0 && islandCheckIndex != isladIndex) 
					fillPolysWithIslandIdx(islandCheckIndex, isladIndex, false); // Merge islands
				else
					((dtNavMesh*)m_navMesh)->setPolyIslandIdx(nearestPoly, isladIndex);
			}
		}
	}
}
#endif
