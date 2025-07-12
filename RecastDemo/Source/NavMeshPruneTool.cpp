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

#include "NavMeshPruneTool.h"

#include "DetourAssert.h"
#include "DetourCommon.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "InputGeom.h"
#include "Sample.h"
#include "imguiHelpers.h"

#include <imgui.h>

#include <cstring>
#include <vector>

#ifdef WIN32
#	define snprintf _snprintf
#endif

class NavmeshFlags
{
	struct TileFlags
	{
		inline void purge() { dtFree(flags); }
		unsigned char* flags;
		int nflags;
		dtPolyRef base;
	};

	const dtNavMesh* nav = nullptr;
	TileFlags* tiles = nullptr;
	int numTiles = 0;

public:
	~NavmeshFlags()
	{
		for (int i = 0; i < numTiles; ++i)
		{
			tiles[i].purge();
		}
		dtFree(tiles);
	}

	bool init(const dtNavMesh* navmesh)
	{
		numTiles = navmesh->getMaxTiles();
		if (!numTiles)
		{
			return true;
		}

		tiles = (TileFlags*)dtAlloc(sizeof(TileFlags) * numTiles, DT_ALLOC_TEMP);
		if (!tiles)
		{
			return false;
		}
		memset(tiles, 0, sizeof(TileFlags) * numTiles);

		// Alloc flags for each tile.
		for (int i = 0; i < navmesh->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = navmesh->getTile(i);
			if (!tile->header)
			{
				continue;
			}
			TileFlags* tileFlags = &tiles[i];
			tileFlags->nflags = tile->header->polyCount;
			tileFlags->base = navmesh->getPolyRefBase(tile);
			if (tileFlags->nflags)
			{
				tileFlags->flags = (unsigned char*)dtAlloc(tileFlags->nflags, DT_ALLOC_TEMP);
				if (!tileFlags->flags)
				{
					return false;
				}
				memset(tileFlags->flags, 0, tileFlags->nflags);
			}
		}

		nav = navmesh;

		return false;
	}

	inline void clearAllFlags()
	{
		for (int i = 0; i < numTiles; ++i)
		{
			TileFlags* tileFlags = &tiles[i];
			if (tileFlags->nflags)
			{
				memset(tileFlags->flags, 0, tileFlags->nflags);
			}
		}
	}

	inline unsigned char getFlags(dtPolyRef ref)
	{
		dtAssert(nav);
		dtAssert(numTiles);
		// Assume the ref is valid, no bounds checks.
		unsigned int salt;
		unsigned int it;
		unsigned int ip;
		nav->decodePolyId(ref, salt, it, ip);
		return tiles[it].flags[ip];
	}

	inline void setFlags(dtPolyRef ref, unsigned char flags)
	{
		dtAssert(nav);
		dtAssert(numTiles);
		// Assume the ref is valid, no bounds checks.
		unsigned int salt;
		unsigned int it;
		unsigned int ip;
		nav->decodePolyId(ref, salt, it, ip);
		tiles[it].flags[ip] = flags;
	}
};

static void floodNavmesh(dtNavMesh* nav, NavmeshFlags* flags, dtPolyRef start, unsigned char flag)
{
	// If already visited, skip.
	if (flags->getFlags(start))
	{
		return;
	}

	flags->setFlags(start, flag);

	std::vector<dtPolyRef> openList;
	openList.push_back(start);

	while (openList.size())
	{
		const dtPolyRef ref = openList.back();
		openList.pop_back();

		// Get current poly and tile.
		// The API input has been checked already, skip checking internal data.
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		nav->getTileAndPolyByRefUnsafe(ref, &tile, &poly);

		// Visit linked polygons.
		for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
		{
			const dtPolyRef neiRef = tile->links[i].ref;
			// Skip invalid and already visited.
			if (!neiRef || flags->getFlags(neiRef))
			{
				continue;
			}

			// Mark as visited
			flags->setFlags(neiRef, flag);
			// Visit neighbours
			openList.push_back(neiRef);
		}
	}
}

static void disableUnvisitedPolys(dtNavMesh* nav, NavmeshFlags* flags)
{
	for (int i = 0; i < nav->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = ((const dtNavMesh*)nav)->getTile(i);
		if (!tile->header)
		{
			continue;
		}
		const dtPolyRef base = nav->getPolyRefBase(tile);
		for (int j = 0; j < tile->header->polyCount; ++j)
		{
			const dtPolyRef ref = base | (unsigned int)j;
			if (!flags->getFlags(ref))
			{
				unsigned short f = 0;
				nav->getPolyFlags(ref, &f);
				nav->setPolyFlags(ref, f | SAMPLE_POLYFLAGS_DISABLED);
			}
		}
	}
}

NavMeshPruneTool::~NavMeshPruneTool()
{
	delete flags;
}

void NavMeshPruneTool::reset()
{
	hitPosSet = false;
	delete flags;
	flags = nullptr;
}

void NavMeshPruneTool::handleMenu()
{
	if (!flags)
	{
		return;
	}

	dtNavMesh* nav = sample->getNavMesh();
	if (!nav)
	{
		return;
	}

	if (ImGui::Button("Clear Selection"))
	{
		flags->clearAllFlags();
	}

	if (ImGui::Button("Prune Unselected"))
	{
		disableUnvisitedPolys(nav, flags);
		delete flags;
		flags = nullptr;
	}
}

void NavMeshPruneTool::handleClick(const float* s, const float* p, bool shift)
{
	rcIgnoreUnused(s);
	rcIgnoreUnused(shift);

	if (!sample)
	{
		return;
	}
	InputGeom* geom = sample->getInputGeom();
	if (!geom)
	{
		return;
	}
	dtNavMesh* nav = sample->getNavMesh();
	if (!nav)
	{
		return;
	}
	dtNavMeshQuery* query = sample->getNavMeshQuery();
	if (!query)
	{
		return;
	}

	dtVcopy(hitPos, p);
	hitPosSet = true;

	if (!flags)
	{
		flags = new NavmeshFlags;
		flags->init(nav);
	}

	const float halfExtents[3] = {2, 4, 2};
	dtQueryFilter filter;
	dtPolyRef ref = 0;
	query->findNearestPoly(p, halfExtents, &filter, &ref, 0);

	floodNavmesh(nav, flags, ref, 1);
}

void NavMeshPruneTool::handleRender()
{
	duDebugDraw& debugDraw = sample->getDebugDraw();

	if (hitPosSet)
	{
		const float s = sample->getAgentRadius();
		const unsigned int col = duRGBA(255, 255, 255, 255);
		debugDraw.begin(DU_DRAW_LINES);
		debugDraw.vertex(hitPos[0] - s, hitPos[1], hitPos[2], col);
		debugDraw.vertex(hitPos[0] + s, hitPos[1], hitPos[2], col);
		debugDraw.vertex(hitPos[0], hitPos[1] - s, hitPos[2], col);
		debugDraw.vertex(hitPos[0], hitPos[1] + s, hitPos[2], col);
		debugDraw.vertex(hitPos[0], hitPos[1], hitPos[2] - s, col);
		debugDraw.vertex(hitPos[0], hitPos[1], hitPos[2] + s, col);
		debugDraw.end();
	}

	const dtNavMesh* nav = sample->getNavMesh();
	if (flags && nav)
	{
		for (int i = 0; i < nav->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = nav->getTile(i);
			if (!tile->header)
			{
				continue;
			}
			const dtPolyRef base = nav->getPolyRefBase(tile);
			for (int j = 0; j < tile->header->polyCount; ++j)
			{
				const dtPolyRef ref = base | (unsigned int)j;
				if (flags->getFlags(ref))
				{
					duDebugDrawNavMeshPoly(&debugDraw, *nav, ref, duRGBA(255, 255, 255, 128));
				}
			}
		}
	}
}

void NavMeshPruneTool::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* view)
{
	DrawScreenspaceText(280.0f, static_cast<float>(view[3]) - 40.0f, IM_COL32(255, 255, 255, 192), "LMB: Click fill area.");
}
