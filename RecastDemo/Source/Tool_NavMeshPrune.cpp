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

#include "Tool_NavMeshPrune.h"

#include "DetourCommon.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "imguiHelpers.h"

#include <imgui.h>

namespace
{
void floodNavmesh(const dtNavMesh* navmesh, NavmeshFlags* flags, const dtPolyRef start, const unsigned char flag)
{
	// If already visited, skip.
	if (flags->getFlags(start))
	{
		return;
	}

	flags->setFlags(start, flag);

	std::vector<dtPolyRef> openList;
	openList.push_back(start);

	while (!openList.empty())
	{
		const dtPolyRef ref = openList.back();
		openList.pop_back();

		// Get current poly and tile.
		// The API input has been checked already, skip checking internal data.
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		navmesh->getTileAndPolyByRefUnsafe(ref, &tile, &poly);

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
			// Visit neighbors
			openList.push_back(neiRef);
		}
	}
}

void disableUnvisitedPolys(dtNavMesh* nav, NavmeshFlags* flags)
{
	for (int i = 0; i < nav->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = static_cast<const dtNavMesh*>(nav)->getTile(i);
		if (!tile->header)
		{
			continue;
		}
		const dtPolyRef base = nav->getPolyRefBase(tile);
		for (int j = 0; j < tile->header->polyCount; ++j)
		{
			const dtPolyRef ref = base | static_cast<unsigned int>(j);
			if (!flags->getFlags(ref))
			{
				unsigned short f = 0;
				nav->getPolyFlags(ref, &f);
				nav->setPolyFlags(ref, f | SAMPLE_POLYFLAGS_DISABLED);
			}
		}
	}
}
}

void NavmeshFlags::init(const dtNavMesh* newNavmesh)
{
	navmesh = newNavmesh;
	tileFlags.clear();

	const int numTiles = navmesh->getMaxTiles();
	if (numTiles == 0)
	{
		return;
	}
	tileFlags.resize(numTiles);

	// Alloc flags for each poly in each tile.
	for (int i = 0; i < numTiles; ++i)
	{
		const dtMeshTile* tile = navmesh->getTile(i);
		if (tile->header == nullptr)
		{
			continue;
		}

		tileFlags[i].resize(tile->header->polyCount, 0);
	}
}

void NavmeshFlags::clearAllFlags()
{
	for (auto& tile : tileFlags)
	{
		for (auto& polyFlags : tile)
		{
			polyFlags = 0;
		}
	}
}

[[nodiscard]] unsigned char NavmeshFlags::getFlags(const dtPolyRef ref) const
{
	dtAssert(navmesh != nullptr);
	dtAssert(!tileFlags.empty());
	// Assume the ref is valid, no bounds checks.
	unsigned int salt;
	unsigned int it;
	unsigned int ip;
	navmesh->decodePolyId(ref, salt, it, ip);
	return tileFlags[it][ip];
}

void NavmeshFlags::setFlags(const dtPolyRef ref, const unsigned char flags)
{
	dtAssert(navmesh != nullptr);
	dtAssert(!tileFlags.empty());
	// Assume the ref is valid, no bounds checks.
	unsigned int salt;
	unsigned int it;
	unsigned int ip;
	navmesh->decodePolyId(ref, salt, it, ip);
	tileFlags[it][ip] = flags;
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

void NavMeshPruneTool::drawMenuUI()
{
	if (!flags)
	{
		return;
	}

	dtNavMesh* nav = sample->navMesh;
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

void NavMeshPruneTool::onClick(const float* s, const float* p, bool shift)
{
	rcIgnoreUnused(s);
	rcIgnoreUnused(shift);

	if (!sample)
	{
		return;
	}

	const InputGeom* geom = sample->inputGeometry;
	if (!geom)
	{
		return;
	}

	const dtNavMesh* nav = sample->navMesh;
	if (!nav)
	{
		return;
	}

	const dtNavMeshQuery* query = sample->navQuery;
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

	constexpr float halfExtents[3] = {2, 4, 2};
	const dtQueryFilter filter;
	dtPolyRef ref = 0;
	query->findNearestPoly(p, halfExtents, &filter, &ref, 0);

	floodNavmesh(nav, flags, ref, 1);
}

void NavMeshPruneTool::render()
{
	duDebugDraw& debugDraw = sample->debugDraw;

	if (hitPosSet)
	{
		const float radius = sample->agentRadius;
		const unsigned int color = duRGBA(255, 255, 255, 255);
		debugDraw.begin(DU_DRAW_LINES);
		debugDraw.vertex(hitPos[0] - radius, hitPos[1], hitPos[2], color);
		debugDraw.vertex(hitPos[0] + radius, hitPos[1], hitPos[2], color);
		debugDraw.vertex(hitPos[0], hitPos[1] - radius, hitPos[2], color);
		debugDraw.vertex(hitPos[0], hitPos[1] + radius, hitPos[2], color);
		debugDraw.vertex(hitPos[0], hitPos[1], hitPos[2] - radius, color);
		debugDraw.vertex(hitPos[0], hitPos[1], hitPos[2] + radius, color);
		debugDraw.end();
	}

	const dtNavMesh* navmesh = sample->navMesh;
	if (flags && navmesh)
	{
		for (int i = 0; i < navmesh->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = navmesh->getTile(i);
			if (!tile->header)
			{
				continue;
			}
			const dtPolyRef base = navmesh->getPolyRefBase(tile);
			for (int j = 0; j < tile->header->polyCount; ++j)
			{
				const dtPolyRef ref = base | static_cast<unsigned int>(j);
				if (flags->getFlags(ref))
				{
					duDebugDrawNavMeshPoly(&debugDraw, *navmesh, ref, duRGBA(255, 255, 255, 128));
				}
			}
		}
	}
}

void NavMeshPruneTool::drawOverlayUI()
{
	DrawScreenspaceText(280.0f, 40.0f, IM_COL32(255, 255, 255, 192), "LMB: Click fill area.");
}
