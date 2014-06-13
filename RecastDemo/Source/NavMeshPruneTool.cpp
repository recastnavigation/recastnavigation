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

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "NavMeshPruneTool.h"
#include "InputGeom.h"
#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourCommon.h"
#include "DetourAssert.h"
#include "DetourDebugDraw.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif



// Copy/paste from Recast int array
class PolyRefArray
{
	dtPolyRef* m_data;
	int m_size, m_cap;
	inline PolyRefArray(const PolyRefArray&);
	inline PolyRefArray& operator=(const PolyRefArray&);
public:
	
	inline PolyRefArray() : m_data(0), m_size(0), m_cap(0) {}
	inline PolyRefArray(int n) : m_data(0), m_size(0), m_cap(0) { resize(n); }
	inline ~PolyRefArray() { dtFree(m_data); }
	void resize(int n)
	{
		if (n > m_cap)
		{
			if (!m_cap) m_cap = n;
			while (m_cap < n) m_cap *= 2;
			dtPolyRef* newData = (dtPolyRef*)dtAlloc(m_cap*sizeof(dtPolyRef), DT_ALLOC_TEMP);
			if (m_size && newData) memcpy(newData, m_data, m_size*sizeof(dtPolyRef));
			dtFree(m_data);
			m_data = newData;
		}
		m_size = n;
	}
	inline void push(dtPolyRef item) { resize(m_size+1); m_data[m_size-1] = item; }
	inline dtPolyRef pop() { if (m_size > 0) m_size--; return m_data[m_size]; }
	inline const dtPolyRef& operator[](int i) const { return m_data[i]; }
	inline dtPolyRef& operator[](int i) { return m_data[i]; }
	inline int size() const { return m_size; }
};




class NavmeshFlags
{
	struct TileFlags
	{
		inline void purge() { dtFree(flags); }
		unsigned char* flags;
		int nflags;
		dtPolyRef base;
	};
	
	const dtNavMesh* m_nav;
	TileFlags* m_tiles;
	int m_ntiles;

public:
	NavmeshFlags() :
		m_nav(0), m_tiles(0), m_ntiles(0)
	{
	}
	
	~NavmeshFlags()
	{
		for (int i = 0; i < m_ntiles; ++i)
			m_tiles[i].purge();
		dtFree(m_tiles);
	}
	
	bool init(const dtNavMesh* nav)
	{
		m_ntiles = nav->getMaxTiles();
		if (!m_ntiles)
			return true;
		m_tiles = (TileFlags*)dtAlloc(sizeof(TileFlags)*m_ntiles, DT_ALLOC_TEMP);
		if (!m_tiles)
		{
			return false;
		}
		memset(m_tiles, 0, sizeof(TileFlags)*m_ntiles);
		
		// Alloc flags for each tile.
		for (int i = 0; i < nav->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = nav->getTile(i);
			if (!tile->header) continue;
			TileFlags* tf = &m_tiles[i];
			tf->nflags = tile->header->polyCount;
			tf->base = nav->getPolyRefBase(tile);
			if (tf->nflags)
			{
				tf->flags = (unsigned char*)dtAlloc(tf->nflags, DT_ALLOC_TEMP);
				if (!tf->flags)
					return false;
				memset(tf->flags, 0, tf->nflags);
			}
		}
		
		m_nav = nav;
		
		return false;
	}
	
	inline void clearAllFlags()
	{
		for (int i = 0; i < m_ntiles; ++i)
		{
			TileFlags* tf = &m_tiles[i];
			if (tf->nflags)
				memset(tf->flags, 0, tf->nflags);
		}
	}
	
	inline unsigned char getFlags(dtPolyRef ref)
	{
		dtAssert(m_nav);
		dtAssert(m_ntiles);
		// Assume the ref is valid, no bounds checks.
		unsigned int salt, it, ip;
		m_nav->decodePolyId(ref, salt, it, ip);
		return m_tiles[it].flags[ip];
	}

	inline void setFlags(dtPolyRef ref, unsigned char flags)
	{
		dtAssert(m_nav);
		dtAssert(m_ntiles);
		// Assume the ref is valid, no bounds checks.
		unsigned int salt, it, ip;
		m_nav->decodePolyId(ref, salt, it, ip);
		m_tiles[it].flags[ip] = flags;
	}
	
};

static void floodNavmesh(dtNavMesh* nav, NavmeshFlags* flags, dtPolyRef start, unsigned char flag)
{
	// If already visited, skip.
	if (flags->getFlags(start))
		return;
		
	PolyRefArray openList;
	openList.push(start);

	while (openList.size())
	{
		const dtPolyRef ref = openList.pop();
		// Get current poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		nav->getTileAndPolyByRefUnsafe(ref, &tile, &poly);

		// Visit linked polygons.
		for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
		{
			const dtPolyRef neiRef = tile->links[i].ref;
			// Skip invalid and already visited.
			if (!neiRef || flags->getFlags(neiRef))
				continue;
			// Mark as visited
			flags->setFlags(neiRef, flag);
			// Visit neighbours
			openList.push(neiRef);
		}
	}
}

static void disableUnvisitedPolys(dtNavMesh* nav, NavmeshFlags* flags)
{
	for (int i = 0; i < nav->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = ((const dtNavMesh*)nav)->getTile(i);
		if (!tile->header) continue;
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

NavMeshPruneTool::NavMeshPruneTool() :
	m_flags(0),
	m_hitPosSet(false)
{
}

NavMeshPruneTool::~NavMeshPruneTool()
{
	delete m_flags;
}

void NavMeshPruneTool::init(Sample* sample)
{
	m_sample = sample;
}

void NavMeshPruneTool::reset()
{
	m_hitPosSet = false;
	delete m_flags;
	m_flags = 0;
}

void NavMeshPruneTool::handleMenu()
{
	dtNavMesh* nav = m_sample->getNavMesh();
	if (!nav) return;
	if (!m_flags) return;

	if (imguiButton("Clear Selection"))
	{
		m_flags->clearAllFlags();
	}
	
	if (imguiButton("Prune Unselected"))
	{
		disableUnvisitedPolys(nav, m_flags);
		delete m_flags;
		m_flags = 0;
	}
}

void NavMeshPruneTool::handleClick(const float* s, const float* p, bool shift)
{
	rcIgnoreUnused(s);
	rcIgnoreUnused(shift);

	if (!m_sample) return;
	InputGeom* geom = m_sample->getInputGeom();
	if (!geom) return;
	dtNavMesh* nav = m_sample->getNavMesh();
	if (!nav) return;
	dtNavMeshQuery* query = m_sample->getNavMeshQuery();
	if (!query) return;
	
	dtVcopy(m_hitPos, p);
	m_hitPosSet = true;
	
	if (!m_flags)
	{
		m_flags = new NavmeshFlags;
		m_flags->init(nav);
	}
	
	const float ext[3] = {2,4,2};
	dtQueryFilter filter;
	dtPolyRef ref = 0;
	query->findNearestPoly(p, ext, &filter, &ref, 0);

	floodNavmesh(nav, m_flags, ref, 1);
}

void NavMeshPruneTool::handleToggle()
{
}

void NavMeshPruneTool::handleStep()
{
}

void NavMeshPruneTool::handleUpdate(const float /*dt*/)
{
}

void NavMeshPruneTool::handleRender()
{
	DebugDrawGL dd;

	if (m_hitPosSet)
	{
		const float s = m_sample->getAgentRadius();
		const unsigned int col = duRGBA(255,255,255,255);
		dd.begin(DU_DRAW_LINES);
		dd.vertex(m_hitPos[0]-s,m_hitPos[1],m_hitPos[2], col);
		dd.vertex(m_hitPos[0]+s,m_hitPos[1],m_hitPos[2], col);
		dd.vertex(m_hitPos[0],m_hitPos[1]-s,m_hitPos[2], col);
		dd.vertex(m_hitPos[0],m_hitPos[1]+s,m_hitPos[2], col);
		dd.vertex(m_hitPos[0],m_hitPos[1],m_hitPos[2]-s, col);
		dd.vertex(m_hitPos[0],m_hitPos[1],m_hitPos[2]+s, col);
		dd.end();
	}

	const dtNavMesh* nav = m_sample->getNavMesh();
	if (m_flags && nav)
	{
		for (int i = 0; i < nav->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = nav->getTile(i);
			if (!tile->header) continue;
			const dtPolyRef base = nav->getPolyRefBase(tile);
			for (int j = 0; j < tile->header->polyCount; ++j)
			{
				const dtPolyRef ref = base | (unsigned int)j;
				if (m_flags->getFlags(ref))
				{
					duDebugDrawNavMeshPoly(&dd, *nav, ref, duRGBA(255,255,255,128));
				}
			}
		}
	}

}

void NavMeshPruneTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	rcIgnoreUnused(model);
	rcIgnoreUnused(proj);

	// Tool help
	const int h = view[3];

	imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB: Click fill area.", imguiRGBA(255,255,255,192));
}
