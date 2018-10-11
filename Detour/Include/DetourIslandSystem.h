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

#include <cstring>
#include "DetourAssert.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

#ifdef NAVMESH_ISLAND_SYSTEM

class NavmeshIslands
{
	struct TileIslands
	{
		inline void purge() { dtFree(islands); }
		unsigned int* islands;
		int nislands;
		dtPolyRef base;
	};

	const dtNavMesh* m_nav;
	TileIslands* m_tiles;
	int m_ntiles;

public:
	NavmeshIslands() :
		m_nav(0), m_tiles(0), m_ntiles(0)
	{
	}

	~NavmeshIslands()
	{
		for (int i = 0; i < m_ntiles; ++i)
			m_tiles[i].purge();
		dtFree(m_tiles);
	}

	bool init(const dtNavMesh* nav)
	{
		m_nav = nav;

		m_ntiles = nav->getMaxTiles();
		if (!m_ntiles)
			return false;
		m_tiles = (TileIslands*)dtAlloc(sizeof(TileIslands)*m_ntiles, DT_ALLOC_TEMP);
		if (!m_tiles)
			return false;
		memset(m_tiles, 0, sizeof(TileIslands)*m_ntiles);

		// Alloc flags for each tile.
		for (int i = 0; i < nav->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = nav->getTile(i);
			if (!tile->header) continue;
			TileIslands* ti = &m_tiles[i];
			ti->nislands = tile->header->polyCount;
			ti->base = nav->getPolyRefBase(tile);
			if (ti->nislands)
			{
				ti->islands = (unsigned int*)dtAlloc(ti->nislands, DT_ALLOC_TEMP);
				if (!ti->islands)
					return false;
				memset(ti->islands, 0, ti->nislands);
			}
		}

		return true;
	}

	inline void clearAllIslands()
	{
		for (int i = 0; i < m_ntiles; ++i)
		{
			TileIslands* tf = &m_tiles[i];
			if (tf->nislands)
				memset(tf->islands, 0, tf->nislands);
		}
	}

	inline unsigned int getIsland(dtPolyRef ref)
	{
		dtAssert(m_nav);
		dtAssert(m_ntiles);
		// Assume the ref is valid, no bounds checks.
		unsigned int salt, it, ip;
		m_nav->decodePolyId(ref, salt, it, ip);
		return m_tiles[it].islands[ip];
	}

	inline void setFlags(dtPolyRef ref, unsigned int island)
	{
		dtAssert(m_nav);
		dtAssert(m_ntiles);
		// Assume the ref is valid, no bounds checks.
		unsigned int salt, it, ip;
		m_nav->decodePolyId(ref, salt, it, ip);
		m_tiles[it].islands[ip] = island;
	}

};

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