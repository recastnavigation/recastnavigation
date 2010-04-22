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

#include <math.h>
#include <float.h>
#include <string.h>
#include <stdio.h>
#include "DetourNavMesh.h"
#include "DetourNode.h"
#include "DetourCommon.h"


inline int opposite(int side) { return (side+4) & 0x7; }

inline bool overlapBoxes(const float* amin, const float* amax,
						 const float* bmin, const float* bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

inline bool overlapRects(const float* amin, const float* amax,
						 const float* bmin, const float* bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	return overlap;
}

inline bool overlapSlabs(const float* amin, const float* amax,
						 const float* bmin, const float* bmax,
						 const float px, const float py)
{
	// Check for horizontal overlap.
	const float minx = dtMax(amin[0]-px,bmin[0]-px);
	const float maxx = dtMin(amax[0]+px,bmax[0]+px);
	if (minx > maxx)
		return false;
	
	// Check vertical overlap.
	const float ad = (amax[1]-amin[1]) / (amax[0]-amin[0]);
	const float ak = amin[1] - ad*amin[0];
	const float bd = (bmax[1]-bmin[1]) / (bmax[0]-bmin[0]);
	const float bk = bmin[1] - bd*bmin[0];
	const float aminy = ad*minx + ak;
	const float amaxy = ad*maxx + ak;
	const float bminy = bd*minx + bk;
	const float bmaxy = bd*maxx + bk;
	const float dmin = bminy - aminy;
	const float dmax = bmaxy - amaxy;
		
	// Crossing segments always overlap.
	if (dmin*dmax < 0)
		return true;
		
	// Check for overlap at endpoints.
	const float thr = dtSqr(py*2);
	if (dmin*dmin <= thr || dmax*dmax <= thr)
		return true;
		
	return false;
}

static void calcSlabEndPoints(const float* va, const float* vb, float* bmin, float* bmax, const int side)
{
	if (side == 0 || side == 4)
	{
		if (va[2] < vb[2])
		{
			bmin[0] = va[2];
			bmin[1] = va[1];
			bmax[0] = vb[2];
			bmax[1] = vb[1];
		}
		else
		{
			bmin[0] = vb[2];
			bmin[1] = vb[1];
			bmax[0] = va[2];
			bmax[1] = va[1];
		}
	}
	else if (side == 2 || side == 6)
	{
		if (va[0] < vb[0])
		{
			bmin[0] = va[0];
			bmin[1] = va[1];
			bmax[0] = vb[0];
			bmax[1] = vb[1];
		}
		else
		{
			bmin[0] = vb[0];
			bmin[1] = vb[1];
			bmax[0] = va[0];
			bmax[1] = va[1];
		}
	}
}

inline int computeTileHash(int x, int y, const int mask)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	unsigned int n = h1 * x + h2 * y;
	return (int)(n & mask);
}

inline unsigned int allocLink(dtMeshTile* tile)
{
	if (tile->linksFreeList == DT_NULL_LINK)
		return DT_NULL_LINK;
	unsigned int link = tile->linksFreeList;
	tile->linksFreeList = tile->links[link].next;
	return link;
}

inline void freeLink(dtMeshTile* tile, unsigned int link)
{
	tile->links[link].next = tile->linksFreeList;
	tile->linksFreeList = link;
}


inline bool passFilter(const dtQueryFilter* filter, unsigned short flags)
{
	return (flags & filter->includeFlags) != 0 && (flags & filter->excludeFlags) == 0;
}



//////////////////////////////////////////////////////////////////////////////////////////
dtNavMesh::dtNavMesh() :
	m_tileWidth(0),
	m_tileHeight(0),
	m_maxTiles(0),
	m_tileLutSize(0),
	m_tileLutMask(0),
	m_posLookup(0),
	m_nextFree(0),
	m_tiles(0),
	m_saltBits(0),
	m_tileBits(0),
	m_polyBits(0),
	m_nodePool(0),
	m_openList(0)
{
	m_orig[0] = 0;
	m_orig[1] = 0;
	m_orig[2] = 0;
	
	for (int i = 0; i < DT_MAX_AREAS; ++i)
		m_areaCost[i] = 1.0f;
}

dtNavMesh::~dtNavMesh()
{
	for (int i = 0; i < m_maxTiles; ++i)
	{
		if (m_tiles[i].flags & DT_TILE_FREE_DATA)
		{
			delete [] m_tiles[i].data;
			m_tiles[i].data = 0;
			m_tiles[i].dataSize = 0;
		}
	}
	delete m_nodePool;
	delete m_openList;
	delete [] m_posLookup;
	delete [] m_tiles;
}
		
bool dtNavMesh::init(const dtNavMeshParams* params)
{
	memcpy(&m_params, params, sizeof(dtNavMeshParams));
	dtVcopy(m_orig, params->orig);
	m_tileWidth = params->tileWidth;
	m_tileHeight = params->tileHeight;
	
	// Init tiles
	m_maxTiles = params->maxTiles;
	m_tileLutSize = dtNextPow2(params->maxTiles/4);
	if (!m_tileLutSize) m_tileLutSize = 1;
	m_tileLutMask = m_tileLutSize-1;
	
	m_tiles = new dtMeshTile[m_maxTiles];
	if (!m_tiles)
		return false;
	m_posLookup = new dtMeshTile*[m_tileLutSize];
	if (!m_posLookup)
		return false;
	memset(m_tiles, 0, sizeof(dtMeshTile)*m_maxTiles);
	memset(m_posLookup, 0, sizeof(dtMeshTile*)*m_tileLutSize);
	m_nextFree = 0;
	for (int i = m_maxTiles-1; i >= 0; --i)
	{
		m_tiles[i].next = m_nextFree;
		m_nextFree = &m_tiles[i];
	}

	if (!m_nodePool)
	{
		m_nodePool = new dtNodePool(params->maxNodes, dtNextPow2(params->maxNodes/4));
		if (!m_nodePool)
			return false;
	}
	
	if (!m_openList)
	{
		m_openList = new dtNodeQueue(params->maxNodes);
		if (!m_openList)
			return false;
	}
	
	// Init ID generator values.
	m_tileBits = dtMax((unsigned int)1, dtIlog2(dtNextPow2((unsigned int)params->maxTiles)));
	m_polyBits = dtMax((unsigned int)1, dtIlog2(dtNextPow2((unsigned int)params->maxPolys)));
	m_saltBits = 32 - m_tileBits - m_polyBits;
	if (m_saltBits < 10)
		return false;
	
	return true;
}

bool dtNavMesh::init(unsigned char* data, int dataSize, int flags, int maxNodes)
{
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return false;
	if (header->version != DT_NAVMESH_VERSION)
		return false;

	dtNavMeshParams params;
	dtVcopy(params.orig, header->bmin);
	params.tileWidth = header->bmax[0] - header->bmin[0];
	params.tileHeight = header->bmax[2] - header->bmin[2];
	params.maxTiles = 1;
	params.maxPolys = header->polyCount;
	params.maxNodes = maxNodes;
	if (!init(&params))
		return false;

	return addTile(data, dataSize, flags) != 0;
}

const dtNavMeshParams* dtNavMesh::getParams() const
{
	return &m_params;
}

//////////////////////////////////////////////////////////////////////////////////////////
int dtNavMesh::findConnectingPolys(const float* va, const float* vb,
								   const dtMeshTile* tile, int side,
								   dtPolyRef* con, float* conarea, int maxcon) const
{
	if (!tile) return 0;
	
	float amin[2], amax[2];
	calcSlabEndPoints(va,vb, amin,amax, side);

	// Remove links pointing to 'side' and compact the links array. 
	float bmin[2], bmax[2];
	unsigned short m = DT_EXT_LINK | (unsigned short)side;
	int n = 0;
	
	dtPolyRef base = getTilePolyRefBase(tile);
	
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];
		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip edges which do not point to the right side.
			if (poly->neis[j] != m) continue;
			// Check if the segments touch.
			const float* vc = &tile->verts[poly->verts[j]*3];
			const float* vd = &tile->verts[poly->verts[(j+1) % nv]*3];
			calcSlabEndPoints(vc,vd, bmin,bmax, side);

			if (!overlapSlabs(amin,amax, bmin,bmax, 0.01f, tile->header->walkableClimb)) continue;
			
			// Add return value.
			if (n < maxcon)
			{
				conarea[n*2+0] = dtMax(amin[0], bmin[0]);
				conarea[n*2+1] = dtMin(amax[0], bmax[0]);
				con[n] = base | (unsigned int)i;
				n++;
			}
			break;
		}
	}
	return n;
}

void dtNavMesh::unconnectExtLinks(dtMeshTile* tile, int side)
{
	if (!tile) return;

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];
		unsigned int j = poly->firstLink;
		unsigned int pj = DT_NULL_LINK;
		while (j != DT_NULL_LINK)
		{
			if (tile->links[j].side == side)
			{
				// Revove link.
				unsigned int nj = tile->links[j].next;
				if (pj == DT_NULL_LINK)
					poly->firstLink = nj;
				else
					tile->links[pj].next = nj;
				freeLink(tile, j);
				j = nj;
			}
			else
			{
				// Advance
				pj = j;
				j = tile->links[j].next;
			}
		}
	}
}

void dtNavMesh::connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side)
{
	if (!tile) return;
	
	// Connect border links.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];

		// Create new links.
		unsigned short m = DT_EXT_LINK | (unsigned short)side;
		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip edges which do not point to the right side.
			if (poly->neis[j] != m) continue;
			
			// Create new links
			const float* va = &tile->verts[poly->verts[j]*3];
			const float* vb = &tile->verts[poly->verts[(j+1) % nv]*3];
			dtPolyRef nei[4];
			float neia[4*2];
			int nnei = findConnectingPolys(va,vb, target, opposite(side), nei,neia,4);
			for (int k = 0; k < nnei; ++k)
			{
				unsigned int idx = allocLink(tile);
				if (idx != DT_NULL_LINK)
				{
					dtLink* link = &tile->links[idx];
					link->ref = nei[k];
					link->edge = (unsigned char)j;
					link->side = (unsigned char)side;
					
					link->next = poly->firstLink;
					poly->firstLink = idx;

					// Compress portal limits to a byte value.
					if (side == 0 || side == 4)
					{
						const float lmin = dtMin(va[2], vb[2]);
						const float lmax = dtMax(va[2], vb[2]);
						link->bmin = (unsigned char)(dtClamp((neia[k*2+0]-lmin)/(lmax-lmin), 0.0f, 1.0f)*255.0f);
						link->bmax = (unsigned char)(dtClamp((neia[k*2+1]-lmin)/(lmax-lmin), 0.0f, 1.0f)*255.0f);
					}
					else if (side == 2 || side == 6)
					{
						const float lmin = dtMin(va[0], vb[0]);
						const float lmax = dtMax(va[0], vb[0]);
						link->bmin = (unsigned char)(dtClamp((neia[k*2+0]-lmin)/(lmax-lmin), 0.0f, 1.0f)*255.0f);
						link->bmax = (unsigned char)(dtClamp((neia[k*2+1]-lmin)/(lmax-lmin), 0.0f, 1.0f)*255.0f);
					}
				}
			}
		}
	}
}

void dtNavMesh::connectExtOffMeshLinks(dtMeshTile* tile, dtMeshTile* target, int side)
{
	if (!tile) return;
	
	// Connect off-mesh links.
	// We are interested on links which land from target tile to this tile.
	const unsigned char oppositeSide = (unsigned char)opposite(side);
	dtQueryFilter defaultFilter;
	
	for (int i = 0; i < target->header->offMeshConCount; ++i)
	{
		dtOffMeshConnection* targetCon = &target->offMeshCons[i];
		if (targetCon->side != oppositeSide)
			continue;
		
		dtPoly* targetPoly = &target->polys[targetCon->poly];
		
		const float ext[3] = { targetCon->rad, target->header->walkableClimb, targetCon->rad };
		
		// Find polygon to connect to.
		const float* p = &targetCon->pos[3];
		float nearestPt[3];
		dtPolyRef ref = findNearestPolyInTile(tile, p, ext, &defaultFilter, nearestPt);
		if (!ref) continue;
		// findNearestPoly may return too optimistic results, further check to make sure. 
		if (dtSqr(nearestPt[0]-p[0])+dtSqr(nearestPt[2]-p[2]) > dtSqr(targetCon->rad))
			continue;
		// Make sure the location is on current mesh.
		float* v = &target->verts[targetPoly->verts[1]*3];
		dtVcopy(v, nearestPt);
				
		// Link off-mesh connection to target poly.
		unsigned int idx = allocLink(target);
		if (idx != DT_NULL_LINK)
		{
			dtLink* link = &target->links[idx];
			link->ref = ref;
			link->edge = (unsigned char)1;
			link->side = oppositeSide;
			link->bmin = link->bmax = 0;
			// Add to linked list.
			link->next = targetPoly->firstLink;
			targetPoly->firstLink = idx;
		}
		
		// Link target poly to off-mesh connection.
		if (targetCon->flags & DT_OFFMESH_CON_BIDIR)
		{
			unsigned int idx = allocLink(tile);
			if (idx != DT_NULL_LINK)
			{
				const unsigned short landPolyIdx = (unsigned short)decodePolyIdPoly(ref);
				dtPoly* landPoly = &tile->polys[landPolyIdx];
				dtLink* link = &tile->links[idx];
				link->ref = getTilePolyRefBase(target) | (unsigned int)(targetCon->poly);
				link->edge = 0xff;
				link->side = (unsigned char)side;
				link->bmin = link->bmax = 0;
				// Add to linked list.
				link->next = landPoly->firstLink;
				landPoly->firstLink = idx;
			}
		}
	}

}

void dtNavMesh::connectIntLinks(dtMeshTile* tile)
{
	if (!tile) return;

	dtPolyRef base = getTilePolyRefBase(tile);

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];
		poly->firstLink = DT_NULL_LINK;

		if (poly->type == DT_POLYTYPE_OFFMESH_CONNECTION)
			continue;
			
		// Build edge links backwards so that the links will be
		// in the linked list from lowest index to highest.
		for (int j = poly->vertCount-1; j >= 0; --j)
		{
			// Skip hard and non-internal edges.
			if (poly->neis[j] == 0 || (poly->neis[j] & DT_EXT_LINK)) continue;

			unsigned int idx = allocLink(tile);
			if (idx != DT_NULL_LINK)
			{
				dtLink* link = &tile->links[idx];
				link->ref = base | (unsigned int)(poly->neis[j]-1);
				link->edge = (unsigned char)j;
				link->side = 0xff;
				link->bmin = link->bmax = 0;
				// Add to linked list.
				link->next = poly->firstLink;
				poly->firstLink = idx;
			}
		}			
	}
}

void dtNavMesh::connectIntOffMeshLinks(dtMeshTile* tile)
{
	if (!tile) return;
	
	dtPolyRef base = getTilePolyRefBase(tile);
	
	// Find Off-mesh connection end points.
	for (int i = 0; i < tile->header->offMeshConCount; ++i)
	{
		dtOffMeshConnection* con = &tile->offMeshCons[i];
		dtPoly* poly = &tile->polys[con->poly];
		dtQueryFilter defaultFilter;
	
		const float ext[3] = { con->rad, tile->header->walkableClimb, con->rad };
		
		for (int j = 0; j < 2; ++j)
		{
			unsigned char side = j == 0 ? 0xff : con->side;

			if (side == 0xff)
			{
				// Find polygon to connect to.
				const float* p = &con->pos[j*3];
				float nearestPt[3];
				dtPolyRef ref = findNearestPolyInTile(tile, p, ext, &defaultFilter, nearestPt);
				if (!ref) continue;
				// findNearestPoly may return too optimistic results, further check to make sure. 
				if (dtSqr(nearestPt[0]-p[0])+dtSqr(nearestPt[2]-p[2]) > dtSqr(con->rad))
					continue;
				// Make sure the location is on current mesh.
				float* v = &tile->verts[poly->verts[j]*3];
				dtVcopy(v, nearestPt);

				// Link off-mesh connection to target poly.
				unsigned int idx = allocLink(tile);
				if (idx != DT_NULL_LINK)
				{
					dtLink* link = &tile->links[idx];
					link->ref = ref;
					link->edge = (unsigned char)j;
					link->side = 0xff;
					link->bmin = link->bmax = 0;
					// Add to linked list.
					link->next = poly->firstLink;
					poly->firstLink = idx;
				}

				// Start end-point is always connect back to off-mesh connection,
				// Destination end-point only if it is bidirectional link. 
				if (j == 0 || (j == 1 && (con->flags & DT_OFFMESH_CON_BIDIR)))
				{
					// Link target poly to off-mesh connection.
					unsigned int idx = allocLink(tile);
					if (idx != DT_NULL_LINK)
					{
						const unsigned short landPolyIdx = (unsigned short)decodePolyIdPoly(ref);
						dtPoly* landPoly = &tile->polys[landPolyIdx];
						dtLink* link = &tile->links[idx];
						link->ref = base | (unsigned int)(con->poly);
						link->edge = 0xff;
						link->side = 0xff;
						link->bmin = link->bmax = 0;
						// Add to linked list.
						link->next = landPoly->firstLink;
						landPoly->firstLink = idx;
					}
				}
				
			}
		}
	}
}

dtTileRef dtNavMesh::addTile(unsigned char* data, int dataSize, int flags, dtTileRef lastRef)
{
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return 0;
	if (header->version != DT_NAVMESH_VERSION)
		return 0;
		
	// Make sure the location is free.
	if (getTileAt(header->x, header->y))
		return 0;
		
	// Allocate a tile.
	dtMeshTile* tile = 0;
	if (!lastRef)
	{
		if (m_nextFree)
		{
			tile = m_nextFree;
			m_nextFree = tile->next;
			tile->next = 0;
		}
	}
	else
	{
		// TODO: Better error reporting!
		
		// Try to relocate the tile to specific index with same salt.
		int tileIndex = (int)decodePolyIdTile((dtPolyRef)lastRef);
		if (tileIndex >= m_maxTiles)
			return 0;
		// Try to find the specific tile id from the free list.
		dtMeshTile* target = &m_tiles[tileIndex];
		dtMeshTile* prev = 0;
		tile = m_nextFree;
		while (tile && tile != target)
		{
			prev = tile;
			tile = tile->next;
		}
		// Could not find the correct location.
		if (tile != target)
			return 0;
		// Remove from freelist
		if (!prev)
			m_nextFree = tile->next;
		else
			prev->next = tile->next;

		// Restore salt.
		tile->salt = decodePolyIdSalt((dtPolyRef)lastRef);
	}

	// Make sure we could allocate a tile.
	if (!tile)
		return 0;
	
	// Insert tile into the position lut.
	int h = computeTileHash(header->x, header->y, m_tileLutMask);
	tile->next = m_posLookup[h];
	m_posLookup[h] = tile;
	
	// Patch header pointers.
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float)*3*header->vertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly)*header->polyCount);
	const int linksSize = dtAlign4(sizeof(dtLink)*(header->maxLinkCount));
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail)*header->detailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float)*3*header->detailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char)*4*header->detailTriCount);
	const int bvtreeSize = dtAlign4(sizeof(dtBVNode)*header->bvNodeCount);
	const int offMeshLinksSize = dtAlign4(sizeof(dtOffMeshConnection)*header->offMeshConCount);
	
	unsigned char* d = data + headerSize;
	tile->verts = (float*)d; d += vertsSize;
	tile->polys = (dtPoly*)d; d += polysSize;
	tile->links = (dtLink*)d; d += linksSize;
	tile->detailMeshes = (dtPolyDetail*)d; d += detailMeshesSize;
	tile->detailVerts = (float*)d; d += detailVertsSize;
	tile->detailTris = (unsigned char*)d; d += detailTrisSize;
	tile->bvTree = (dtBVNode*)d; d += bvtreeSize;
	tile->offMeshCons = (dtOffMeshConnection*)d; d += offMeshLinksSize;

	// Build links freelist
	tile->linksFreeList = 0;
	tile->links[header->maxLinkCount-1].next = DT_NULL_LINK;
	for (int i = 0; i < header->maxLinkCount-1; ++i)
		tile->links[i].next = i+1;

	// Init tile.
	tile->header = header;
	tile->data = data;
	tile->dataSize = dataSize;
	tile->flags = flags;

	connectIntLinks(tile);
	connectIntOffMeshLinks(tile);

	// Create connections connections.
	for (int i = 0; i < 8; ++i)
	{
		dtMeshTile* nei = getNeighbourTileAt(header->x, header->y, i);
		if (nei)
		{
			connectExtLinks(tile, nei, i);
			connectExtLinks(nei, tile, opposite(i));
			connectExtOffMeshLinks(tile, nei, i);
			connectExtOffMeshLinks(nei, tile, opposite(i));
		}
	}
	
	return getTileRef(tile);
}

dtMeshTile* dtNavMesh::getTileAt(int x, int y) const
{
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header && tile->header->x == x && tile->header->y == y)
			return tile;
		tile = tile->next;
	}
	return 0;
}

dtTileRef dtNavMesh::getTileRefAt(int x, int y) const
{
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header && tile->header->x == x && tile->header->y == y)
			return getTileRef(tile);
		tile = tile->next;
	}
	return 0;
}

int dtNavMesh::getMaxTiles() const
{
	return m_maxTiles;
}

dtMeshTile* dtNavMesh::getTile(int i)
{
	return &m_tiles[i];
}

const dtMeshTile* dtNavMesh::getTile(int i) const
{
	return &m_tiles[i];
}

const dtMeshTile* dtNavMesh::getTileByPolyRef(dtPolyRef ref, int* polyIndex) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return 0;
	if (polyIndex) *polyIndex = (int)ip;
	return &m_tiles[it];
}

dtMeshTile* dtNavMesh::getNeighbourTileAt(int x, int y, int side) const
{
	switch (side)
	{
	case 0: x++; break;
	case 1: x++; y++; break;
	case 2: y++; break;
	case 3: x--; y++; break;
	case 4: x--; break;
	case 5: x--; y--; break;
	case 6: y--; break;
	case 7: x++; y--; break;
	};
	return getTileAt(x,y);
}

bool dtNavMesh::removeTile(dtTileRef ref, unsigned char** data, int* dataSize)
{
	if (!ref)
		return false;
	unsigned int tileIndex = decodePolyIdTile((dtPolyRef)ref);
	unsigned int tileSalt = decodePolyIdSalt((dtPolyRef)ref);
	if ((int)tileIndex >= m_maxTiles)
		return false;
	dtMeshTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return false;
	
	// Remove tile from hash lookup.
	int h = computeTileHash(tile->header->x,tile->header->y,m_tileLutMask);
	dtMeshTile* prev = 0;
	dtMeshTile* cur = m_posLookup[h];
	while (cur)
	{
		if (cur == tile)
		{
			if (prev)
				prev->next = cur->next;
			else
				m_posLookup[h] = cur->next;
			break;
		}
		prev = cur;
		cur = cur->next;
	}
	
	// Remove connections to neighbour tiles.
	for (int i = 0; i < 8; ++i)
	{
		dtMeshTile* nei = getNeighbourTileAt(tile->header->x,tile->header->y,i);
		if (!nei) continue;
		unconnectExtLinks(nei, opposite(i));
	}
	
	
	// Reset tile.
	if (tile->flags & DT_TILE_FREE_DATA)
	{
		// Owns data
		delete [] tile->data;
		tile->data = 0;
		tile->dataSize = 0;
		if (data) *data = 0;
		if (dataSize) *dataSize = 0;
	}
	else
	{
		if (data) *data = tile->data;
		if (dataSize) *dataSize = tile->dataSize;
	}

	tile->header = 0;
	tile->flags = 0;
	tile->linksFreeList = 0;
	tile->polys = 0;
	tile->verts = 0;
	tile->links = 0;
	tile->detailMeshes = 0;
	tile->detailVerts = 0;
	tile->detailTris = 0;
	tile->bvTree = 0;
	tile->offMeshCons = 0;
		
	tile->salt++;

	// Add to free list.
	tile->next = m_nextFree;
	m_nextFree = tile;

	return true;
}

dtTileRef dtNavMesh::getTileRef(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = tile - m_tiles;
	return (dtTileRef)encodePolyId(tile->salt, it, 0);
}

const dtMeshTile* dtNavMesh::getTileByRef(dtTileRef ref) const
{
	if (!ref) return 0;
	unsigned int tileIndex = decodePolyIdTile((dtPolyRef)ref);
	unsigned int tileSalt = decodePolyIdSalt((dtPolyRef)ref);
	if ((int)tileIndex > m_maxTiles)
		return 0;
	const dtMeshTile* tile = &m_tiles[m_maxTiles];
	if (tile->salt != tileSalt)
		return 0;
	return tile;
}

dtPolyRef dtNavMesh::getTilePolyRefBase(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = tile - m_tiles;
	return encodePolyId(tile->salt, it, 0);
}

struct dtTileState
{
	int magic;								// Magic number, used to identify the data.
	int version;							// Data version number.
	dtTileRef ref;							// Tile ref at the time of storing the data.
};

struct dtPolyState
{
	unsigned short flags;						// Flags (see dtPolyFlags).
	unsigned char area;							// Area ID of the polygon.
};

int dtNavMesh::getTileStateSize(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const int headerSize = dtAlign4(sizeof(dtTileState));
	const int polyStateSize = dtAlign4(sizeof(dtPolyState) * tile->header->polyCount);
	return headerSize + polyStateSize;
}

bool dtNavMesh::storeTileState(const dtMeshTile* tile, unsigned char* data, const int maxDataSize) const
{
	// Make sure there is enough space to store the state.
	const int sizeReq = getTileStateSize(tile);
	if (maxDataSize < sizeReq)
		return false;
		
	dtTileState* tileState = (dtTileState*)data; data += dtAlign4(sizeof(dtTileState));
	dtPolyState* polyStates = (dtPolyState*)data; data += dtAlign4(sizeof(dtPolyState) * tile->header->polyCount);
	
	// Store tile state.
	tileState->magic = DT_NAVMESH_STATE_MAGIC;
	tileState->version = DT_NAVMESH_STATE_VERSION;
	tileState->ref = getTileRef(tile);
	
	// Store per poly state.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		dtPolyState* s = &polyStates[i];
		s->flags = p->flags;
		s->area = p->area;
	}
	
	return true;
}

bool dtNavMesh::restoreTileState(dtMeshTile* tile, const unsigned char* data, const int maxDataSize)
{
	// Make sure there is enough space to store the state.
	const int sizeReq = getTileStateSize(tile);
	if (maxDataSize < sizeReq)
		return false;
	
	const dtTileState* tileState = (const dtTileState*)data; data += dtAlign4(sizeof(dtTileState));
	const dtPolyState* polyStates = (const dtPolyState*)data; data += dtAlign4(sizeof(dtPolyState) * tile->header->polyCount);
	
	// Check that the restore is possible.
	if (tileState->magic != DT_NAVMESH_STATE_MAGIC)
		return false;
	if (tileState->version != DT_NAVMESH_STATE_VERSION)
		return false;
	if (tileState->ref != getTileRef(tile))
		return false;
	
	// Restore per poly state.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* p = &tile->polys[i];
		const dtPolyState* s = &polyStates[i];
		p->flags = s->flags;
		p->area = s->area;
	}
	
	return true;
}


//////////////////////////////////////////////////////////////////////////////////////////
bool dtNavMesh::closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	const dtMeshHeader* header = m_tiles[it].header;
	if (ip >= (unsigned int)header->polyCount) return false;
	
	return closestPointOnPolyInTile(&m_tiles[it], ip, pos, closest);
}

bool dtNavMesh::closestPointOnPolyInTile(const dtMeshTile* tile, unsigned int ip, const float* pos, float* closest) const
{
	const dtPoly* poly = &tile->polys[ip];
	
	float closestDistSqr = FLT_MAX;
	const dtPolyDetail* pd = &tile->detailMeshes[ip];
	
	for (int j = 0; j < pd->triCount; ++j)
	{
		const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
		const float* v[3];
		for (int k = 0; k < 3; ++k)
		{
			if (t[k] < poly->vertCount)
				v[k] = &tile->verts[poly->verts[t[k]]*3];
			else
				v[k] = &tile->detailVerts[(pd->vertBase+(t[k]-poly->vertCount))*3];
		}
		float pt[3];
		dtClosestPtPointTriangle(pt, pos, v[0], v[1], v[2]);
		float d = dtVdistSqr(pos, pt);
		if (d < closestDistSqr)
		{
			dtVcopy(closest, pt);
			closestDistSqr = d;
		}
	}
	
	return true;
}

bool dtNavMesh::closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, float* closest) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	const dtMeshTile* tile = &m_tiles[it];
	
	if (ip >= (unsigned int)tile->header->polyCount) return false;
	const dtPoly* poly = &tile->polys[ip];

	// Collect vertices.
	float verts[DT_VERTS_PER_POLYGON*3];	
	float edged[DT_VERTS_PER_POLYGON];
	float edget[DT_VERTS_PER_POLYGON];
	int nv = 0;
	for (int i = 0; i < (int)poly->vertCount; ++i)
	{
		dtVcopy(&verts[nv*3], &tile->verts[poly->verts[i]*3]);
		nv++;
	}		
	
	bool inside = dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget);
	if (inside)
	{
		// Point is inside the polygon, return the point.
		dtVcopy(closest, pos);
	}
	else
	{
		// Point is outside the polygon, dtClamp to nearest edge.
		float dmin = FLT_MAX;
		int imin = -1;
		for (int i = 0; i < nv; ++i)
		{
			if (edged[i] < dmin)
			{
				dmin = edged[i];
				imin = i;
			}
		}
		const float* va = &verts[imin*3];
		const float* vb = &verts[((imin+1)%nv)*3];
		dtVlerp(closest, va, vb, edget[imin]);
	}

	return true;
}

// Returns start and end location of an off-mesh link polygon.
bool dtNavMesh::getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const
{
	unsigned int salt, it, ip;

	// Get current polygon
	decodePolyId(polyRef, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	const dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return false;
	const dtPoly* poly = &tile->polys[ip];

	// Make sure that the current poly is indeed off-mesh link.
	if (poly->type != DT_POLYTYPE_OFFMESH_CONNECTION)
		return false;

	// Figure out which way to hand out the vertices.
	int idx0 = 0, idx1 = 1;
	
	// Find link that points to first vertex.
	for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
	{
		if (tile->links[i].edge == 0)
		{
			if (tile->links[i].ref != prevRef)
			{
				idx0 = 1;
				idx1 = 0;
			}
			break;
		}
	}
	
	dtVcopy(startPos, &tile->verts[poly->verts[idx0]*3]);
	dtVcopy(endPos, &tile->verts[poly->verts[idx1]*3]);

	return true;
}


bool dtNavMesh::getPolyHeight(dtPolyRef ref, const float* pos, float* height) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	const dtMeshTile* tile = &m_tiles[it];
	
	if (ip >= (unsigned int)tile->header->polyCount) return false;
	const dtPoly* poly = &tile->polys[ip];
	
	if (poly->type == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const float* v0 = &tile->verts[poly->verts[0]*3];
		const float* v1 = &tile->verts[poly->verts[1]*3];
		const float d0 = dtVdist(pos, v0);
		const float d1 = dtVdist(pos, v1);
		const float u = d0 / (d0+d1);
		if (height)
			*height = v0[1] + (v1[1] - v0[1]) * u;
		return true;
	}
	else
	{
		const dtPolyDetail* pd = &tile->detailMeshes[ip];
		for (int j = 0; j < pd->triCount; ++j)
		{
			const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
			const float* v[3];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < poly->vertCount)
					v[k] = &tile->verts[poly->verts[t[k]]*3];
				else
					v[k] = &tile->detailVerts[(pd->vertBase+(t[k]-poly->vertCount))*3];
			}
			float h;
			if (dtClosestHeightPointTriangle(pos, v[0], v[1], v[2], h))
			{
				if (height)
					*height = h;
				return true;
			}
		}
	}
	
	return false;
}

void dtNavMesh::setAreaCost(const int area, float cost)
{
	if (area >= 0 && area < DT_MAX_AREAS)
		m_areaCost[area] = cost;
}

float dtNavMesh::getAreaCost(const int area) const
{
	if (area >= 0 && area < DT_MAX_AREAS)
		return m_areaCost[area];
	return -1;
}

dtPolyRef dtNavMesh::findNearestPoly(const float* center, const float* extents,
									 const dtQueryFilter* filter, float* nearestPt) const
{
	// Get nearby polygons from proximity grid.
	dtPolyRef polys[128];
	int polyCount = queryPolygons(center, extents, filter, polys, 128);
	
	// Find nearest polygon amongst the nearby polygons.
	dtPolyRef nearest = 0;
	float nearestDistanceSqr = FLT_MAX;
	for (int i = 0; i < polyCount; ++i)
	{
		dtPolyRef ref = polys[i];
		float closestPtPoly[3];
		if (!closestPointOnPoly(ref, center, closestPtPoly))
			continue;
		float d = dtVdistSqr(center, closestPtPoly);
		if (d < nearestDistanceSqr)
		{
			if (nearestPt)
				dtVcopy(nearestPt, closestPtPoly);
			nearestDistanceSqr = d;
			nearest = ref;
		}
	}
	
	return nearest;
}

dtPolyRef dtNavMesh::findNearestPolyInTile(const dtMeshTile* tile, const float* center, const float* extents,
										   const dtQueryFilter* filter, float* nearestPt) const
{
	float bmin[3], bmax[3];
	dtVsub(bmin, center, extents);
	dtVadd(bmax, center, extents);
	
	// Get nearby polygons from proximity grid.
	dtPolyRef polys[128];
	int polyCount = queryPolygonsInTile(tile, bmin, bmax, filter, polys, 128);
	
	// Find nearest polygon amongst the nearby polygons.
	dtPolyRef nearest = 0;
	float nearestDistanceSqr = FLT_MAX;
	for (int i = 0; i < polyCount; ++i)
	{
		dtPolyRef ref = polys[i];
		float closestPtPoly[3];
		if (!closestPointOnPolyInTile(tile, decodePolyIdPoly(ref), center, closestPtPoly))
			continue;
		float d = dtVdistSqr(center, closestPtPoly);
		if (d < nearestDistanceSqr)
		{
			if (nearestPt)
				dtVcopy(nearestPt, closestPtPoly);
			nearestDistanceSqr = d;
			nearest = ref;
		}
	}
	
	return nearest;
}

int dtNavMesh::queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
								   const dtQueryFilter* filter,
								   dtPolyRef* polys, const int maxPolys) const
{
	if (tile->bvTree)
	{
		const dtBVNode* node = &tile->bvTree[0];
		const dtBVNode* end = &tile->bvTree[tile->header->bvNodeCount];
		const float* tbmin = tile->header->bmin;
		const float* tbmax = tile->header->bmax;
		const float qfac = tile->header->bvQuantFactor;
		
		// Calculate quantized box
		unsigned short bmin[3], bmax[3];
		// dtClamp query box to world box.
		float minx = dtClamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
		float miny = dtClamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
		float minz = dtClamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
		float maxx = dtClamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
		float maxy = dtClamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
		float maxz = dtClamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
		// Quantize
		bmin[0] = (unsigned short)(qfac * minx) & 0xfffe;
		bmin[1] = (unsigned short)(qfac * miny) & 0xfffe;
		bmin[2] = (unsigned short)(qfac * minz) & 0xfffe;
		bmax[0] = (unsigned short)(qfac * maxx + 1) | 1;
		bmax[1] = (unsigned short)(qfac * maxy + 1) | 1;
		bmax[2] = (unsigned short)(qfac * maxz + 1) | 1;
			
		// Traverse tree
		dtPolyRef base = getTilePolyRefBase(tile);
		int n = 0;
		while (node < end)
		{
			const bool overlap = dtCheckOverlapBox(bmin, bmax, node->bmin, node->bmax);
			const bool isLeafNode = node->i >= 0;
			
			if (isLeafNode && overlap)
			{
				if (passFilter(filter, tile->polys[node->i].flags))
				{
					if (n < maxPolys)
						polys[n++] = base | (dtPolyRef)node->i;
				}
			}
			
			if (overlap || isLeafNode)
				node++;
			else
			{
				const int escapeIndex = -node->i;
				node += escapeIndex;
			}
		}
	
		return n;
	}
	else
	{
		float bmin[3], bmax[3];
		int n = 0;
		dtPolyRef base = getTilePolyRefBase(tile);
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			// Calc polygon bounds.
			dtPoly* p = &tile->polys[i];
			const float* v = &tile->verts[p->verts[0]*3];
			dtVcopy(bmin, v);
			dtVcopy(bmax, v);
			for (int j = 1; j < p->vertCount; ++j)
			{
				v = &tile->verts[p->verts[j]*3];
				dtVmin(bmin, v);
				dtVmax(bmax, v);
			}
			if (overlapBoxes(qmin,qmax, bmin,bmax))
			{
				if (passFilter(filter, p->flags))
				{
					if (n < maxPolys)
						polys[n++] = base | (dtPolyRef)i;
				}
			}
		}
		return n;
	}
}

int dtNavMesh::queryPolygons(const float* center, const float* extents, const dtQueryFilter* filter,
							 dtPolyRef* polys, const int maxPolys) const
{
	float bmin[3], bmax[3];
	dtVsub(bmin, center, extents);
	dtVadd(bmax, center, extents);
	
	// Find tiles the query touches.
	const int minx = (int)floorf((bmin[0]-m_orig[0]) / m_tileWidth);
	const int maxx = (int)floorf((bmax[0]-m_orig[0]) / m_tileWidth);
	const int miny = (int)floorf((bmin[2]-m_orig[2]) / m_tileHeight);
	const int maxy = (int)floorf((bmax[2]-m_orig[2]) / m_tileHeight);

	int n = 0;
	for (int y = miny; y <= maxy; ++y)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const dtMeshTile* tile = getTileAt(x,y);
			if (!tile) continue;
			n += queryPolygonsInTile(tile, bmin, bmax, filter, polys+n, maxPolys-n);
			if (n >= maxPolys) return n;
		}
	}

	return n;
}

int dtNavMesh::findPath(dtPolyRef startRef, dtPolyRef endRef,
						const float* startPos, const float* endPos,
						const dtQueryFilter* filter,
						dtPolyRef* path, const int maxPathSize) const
{
	if (!startRef || !endRef)
		return 0;
	
	if (!maxPathSize)
		return 0;
	
	if (!getPolyByRef(startRef) || !getPolyByRef(endRef))
		return 0;
	
	if (startRef == endRef)
	{
		path[0] = startRef;
		return 1;
	}
	
	if (!m_nodePool || !m_openList)
		return 0;
		
	m_nodePool->clear();
	m_openList->clear();
	
	static const float H_SCALE = 0.999f;	// Heuristic scale.
	
	dtNode* startNode = m_nodePool->getNode(startRef);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = dtVdist(startPos, endPos) * H_SCALE;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	dtNode* lastBestNode = startNode;
	float lastBestNodeCost = startNode->total;

	unsigned int it, ip;
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		// Remove node from open list and put it in closed list.
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;

		// Reached the goal, stop searching.
		if (bestNode->id == endRef)
		{
			lastBestNode = bestNode;
			break;
		}

		float previousEdgeMidPoint[3];

		// Get current poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		it = decodePolyIdTile(bestRef);
		ip = decodePolyIdPoly(bestRef);
		const dtMeshTile* bestTile = &m_tiles[it];
		const dtPoly* bestPoly = &bestTile->polys[ip];

		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
		{
			it = decodePolyIdTile(parentRef);
			ip = decodePolyIdPoly(parentRef);
			parentTile = &m_tiles[it];
			parentPoly = &parentTile->polys[ip];

			getEdgeMidPoint(parentRef, parentPoly, parentTile,
							bestRef, bestPoly, bestTile, previousEdgeMidPoint);
		}
		else
		{
			dtVcopy(previousEdgeMidPoint, startPos);
		}
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			dtPolyRef neighbourRef = bestTile->links[i].ref;
			
			// Skip invalid ids and do not expand back to where we came from.
			if (!neighbourRef || neighbourRef == bestRef)
				continue;

			// Get neighbour poly and tile.
			// The API input has been cheked already, skip checking internal data.
			it = decodePolyIdTile(neighbourRef);
			ip = decodePolyIdPoly(neighbourRef);
			const dtMeshTile* neighbourTile = &m_tiles[it];
			const dtPoly* neighbourPoly = &neighbourTile->polys[ip];

			if (!passFilter(filter, neighbourPoly->flags))
				continue;

			dtNode newNode;
			newNode.pidx = m_nodePool->getNodeIdx(bestNode);
			newNode.id = neighbourRef;

			// Calculate cost.
			float edgeMidPoint[3];
			
			getEdgeMidPoint(bestRef, bestPoly, bestTile,
							neighbourRef, neighbourPoly, neighbourTile, edgeMidPoint);
			
			// Special case for last node.
			float h = 0;
			if (neighbourRef == endRef)
			{
				// Cost
				newNode.cost = bestNode->cost +
								dtVdist(previousEdgeMidPoint,edgeMidPoint) * m_areaCost[bestPoly->area] +
								dtVdist(edgeMidPoint, endPos) * m_areaCost[neighbourPoly->area];
				// Heuristic
				h = 0;
			}
			else
			{
				// Cost
				newNode.cost = bestNode->cost +
								dtVdist(previousEdgeMidPoint,edgeMidPoint) * m_areaCost[bestPoly->area];
				// Heuristic
				h = dtVdist(edgeMidPoint,endPos)*H_SCALE;
			}
			newNode.total = newNode.cost + h;
			
			dtNode* actualNode = m_nodePool->getNode(newNode.id);
			if (!actualNode)
				continue;

			// The node is already in open list and the new result is worse, skip.
			if ((actualNode->flags & DT_NODE_OPEN) && newNode.total >= actualNode->total)
				continue;
			// The node is already visited and process, and the new result is worse, skip.
			if ((actualNode->flags & DT_NODE_CLOSED) && newNode.total >= actualNode->total)
				continue;

			// Add or update the node.
			actualNode->flags &= ~DT_NODE_CLOSED;
			actualNode->pidx = newNode.pidx;
			actualNode->cost = newNode.cost;
			actualNode->total = newNode.total;

			// Update nearest node to target so far.
			if (h < lastBestNodeCost)
			{
				lastBestNodeCost = h;
				lastBestNode = actualNode;
			}
				
			if (actualNode->flags & DT_NODE_OPEN)
			{
				// Already in open, update node location.
				m_openList->modify(actualNode);
			}
			else
			{
				// Put the node in open list.
				actualNode->flags |= DT_NODE_OPEN;
				m_openList->push(actualNode);
			}
		}
	}
	
	// Reverse the path.
	dtNode* prev = 0;
	dtNode* node = lastBestNode;
	do
	{
		dtNode* next = m_nodePool->getNodeAtIdx(node->pidx);
		node->pidx = m_nodePool->getNodeIdx(prev);
		prev = node;
		node = next;
	}
	while (node);
	
	// Store path
	node = prev;
	int n = 0;
	do
	{
		path[n++] = node->id;
		node = m_nodePool->getNodeAtIdx(node->pidx);
	}
	while (node && n < maxPathSize);
	
	return n;
}

int dtNavMesh::findStraightPath(const float* startPos, const float* endPos,
								const dtPolyRef* path, const int pathSize,
								float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
								const int maxStraightPathSize) const
{
	if (!maxStraightPathSize)
		return 0;
	
	if (!path[0])
		return 0;
	
	int straightPathSize = 0;
	
	// TODO: Should this be callers responsibility?
	float closestStartPos[3];
	if (!closestPointOnPolyBoundary(path[0], startPos, closestStartPos))
		return 0;
	
	// Add start point.
	dtVcopy(&straightPath[straightPathSize*3], closestStartPos);
	if (straightPathFlags)
		straightPathFlags[straightPathSize] = DT_STRAIGHTPATH_START;
	if (straightPathRefs)
		straightPathRefs[straightPathSize] = path[0];
	straightPathSize++;
	if (straightPathSize >= maxStraightPathSize)
		return straightPathSize;
	
	float closestEndPos[3];
	if (!closestPointOnPolyBoundary(path[pathSize-1], endPos, closestEndPos))
		return 0;
	
	if (pathSize > 1)
	{
		float portalApex[3], portalLeft[3], portalRight[3];
		dtVcopy(portalApex, closestStartPos);
		dtVcopy(portalLeft, portalApex);
		dtVcopy(portalRight, portalApex);
		int apexIndex = 0;
		int leftIndex = 0;
		int rightIndex = 0;

		unsigned char leftPolyType = 0;
		unsigned char rightPolyType = 0;

		dtPolyRef leftPolyRef = path[0];
		dtPolyRef rightPolyRef = path[0];

		for (int i = 0; i < pathSize; ++i)
		{
			float left[3], right[3];
			unsigned char fromType, toType;
			
			if (i+1 < pathSize)
			{
				// Next portal.
				if (!getPortalPoints(path[i], path[i+1], left, right, fromType, toType))
				{
					if (!closestPointOnPolyBoundary(path[i], endPos, closestEndPos))
						return 0;
					
					dtVcopy(&straightPath[straightPathSize*3], closestEndPos);
					if (straightPathFlags)
						straightPathFlags[straightPathSize] = 0;
					if (straightPathRefs)
						straightPathRefs[straightPathSize] = path[i];
					straightPathSize++;
					
					return straightPathSize;
				}
				
				// If starting really close the portal, advance.
				if (i == 0)
				{
					float t;
					if (dtDistancePtSegSqr2D(portalApex, left, right, t) < (0.001*0.001f))
						continue;
				}
			}
			else
			{
				// End of the path.
				dtVcopy(left, closestEndPos);
				dtVcopy(right, closestEndPos);

				fromType = toType = DT_POLYTYPE_GROUND;
			}
			
			// Right vertex.
			if (dtTriArea2D(portalApex, portalRight, right) <= 0.0f)
			{
				if (dtVequal(portalApex, portalRight) || dtTriArea2D(portalApex, portalLeft, right) > 0.0f)
				{
					dtVcopy(portalRight, right);
					rightPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
					rightPolyType = toType;
					rightIndex = i;
				}
				else
				{
					dtVcopy(portalApex, portalLeft);
					apexIndex = leftIndex;
					
					unsigned char flags = 0;
					if (!leftPolyRef)
						flags = DT_STRAIGHTPATH_END;
					else if (rightPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
						flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
					dtPolyRef ref = leftPolyRef;
					
					if (!dtVequal(&straightPath[(straightPathSize-1)*3], portalApex))
					{
						// Append new vertex.
						dtVcopy(&straightPath[straightPathSize*3], portalApex);
						if (straightPathFlags)
							straightPathFlags[straightPathSize] = flags;
						if (straightPathRefs)
							straightPathRefs[straightPathSize] = ref;
						straightPathSize++;
						// If reached end of path or there is no space to append more vertices, return.
						if (flags == DT_STRAIGHTPATH_END || straightPathSize >= maxStraightPathSize)
							return straightPathSize;
					}
					else
					{
						// The vertices are equal, update flags and poly.
						if (straightPathFlags)
							straightPathFlags[straightPathSize-1] = flags;
						if (straightPathRefs)
							straightPathRefs[straightPathSize-1] = ref;
					}
					
					dtVcopy(portalLeft, portalApex);
					dtVcopy(portalRight, portalApex);
					leftIndex = apexIndex;
					rightIndex = apexIndex;
					
					// Restart
					i = apexIndex;
					
					continue;
				}
			}
			
			// Left vertex.
			if (dtTriArea2D(portalApex, portalLeft, left) >= 0.0f)
			{
				if (dtVequal(portalApex, portalLeft) || dtTriArea2D(portalApex, portalRight, left) < 0.0f)
				{
					dtVcopy(portalLeft, left);
					leftPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
					leftPolyType = toType;
					leftIndex = i;
				}
				else
				{
					dtVcopy(portalApex, portalRight);
					apexIndex = rightIndex;

					unsigned char flags = 0;
					if (!rightPolyRef)
						flags = DT_STRAIGHTPATH_END;
					else if (rightPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
						flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
					dtPolyRef ref = rightPolyRef;
					
					if (!dtVequal(&straightPath[(straightPathSize-1)*3], portalApex))
					{
						// Append new vertex.
						dtVcopy(&straightPath[straightPathSize*3], portalApex);
						if (straightPathFlags)
							straightPathFlags[straightPathSize] = flags;
						if (straightPathRefs)
							straightPathRefs[straightPathSize] = ref;
						straightPathSize++;
						// If reached end of path or there is no space to append more vertices, return.
						if (flags == DT_STRAIGHTPATH_END || straightPathSize >= maxStraightPathSize)
							return straightPathSize;
					}
					else
					{
						// The vertices are equal, update flags and poly.
						if (straightPathFlags)
							straightPathFlags[straightPathSize-1] = flags;
						if (straightPathRefs)
							straightPathRefs[straightPathSize-1] = ref;
					}
					
					dtVcopy(portalLeft, portalApex);
					dtVcopy(portalRight, portalApex);
					leftIndex = apexIndex;
					rightIndex = apexIndex;
					
					// Restart
					i = apexIndex;
					
					continue;
				}
			}
		}
	}
	
	// If the point already exists, remove it and add reappend the actual end location.  
	if (straightPathSize && dtVequal(&straightPath[(straightPathSize-1)*3], closestEndPos))
		straightPathSize--;
		
	// Add end point.
	if (straightPathSize < maxStraightPathSize)
	{
		dtVcopy(&straightPath[straightPathSize*3], closestEndPos);
		if (straightPathFlags)
			straightPathFlags[straightPathSize] = DT_STRAIGHTPATH_END;
		if (straightPathRefs)
			straightPathRefs[straightPathSize] = 0;
		straightPathSize++;
	}
	
	return straightPathSize;
}

// Moves towards end position a long the path corridor.
// Returns: Index to the result path polygon.
int dtNavMesh::moveAlongPathCorridor(const float* startPos, const float* endPos, float* resultPos,
									 const dtPolyRef* path, const int pathSize) const
{
	if (!pathSize)
		return 0;
	
	float verts[DT_VERTS_PER_POLYGON*3];	
	float edged[DT_VERTS_PER_POLYGON];
	float edget[DT_VERTS_PER_POLYGON];
	int n = 0;
	
	static const float SLOP = 0.01f;

	dtVcopy(resultPos, startPos);
	
	while (n < pathSize)
	{
		// Get current polygon and poly vertices.
		unsigned int salt, it, ip;
		decodePolyId(path[n], salt, it, ip);
		if (it >= (unsigned int)m_maxTiles) return n;
		if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return n;
		if (ip >= (unsigned int)m_tiles[it].header->polyCount) return n;
		const dtMeshTile* tile = &m_tiles[it];
		const dtPoly* poly = &tile->polys[ip];
		
		// In case of Off-Mesh link, just snap to the end location and advance over it.
		if (poly->type == DT_POLYTYPE_OFFMESH_CONNECTION)
		{
			if (n+1 < pathSize)
			{
				float left[3], right[3];
				unsigned char fromType, toType;
				if (!getPortalPoints(path[n], path[n+1], left, right, fromType, toType))
					return n;
				dtVcopy(resultPos, endPos);
			}
			return n+1;
		}
		
		// Collect vertices.
		int nv = 0;
		for (int i = 0; i < (int)poly->vertCount; ++i)
		{
			dtVcopy(&verts[nv*3], &tile->verts[poly->verts[i]*3]);
			nv++;
		}

		const bool inside = dtDistancePtPolyEdgesSqr(endPos, verts, nv, edged, edget);
		if (inside)
		{
			// The end point is inside the current polygon.
			dtVcopy(resultPos, endPos);
			return n;
		}

		// Constraint the point on the polygon boundary.
		// This results sliding movement.
		float dmin = FLT_MAX;
		int imin = -1;
		for (int i = 0; i < nv; ++i)
		{
			if (edged[i] < dmin)
			{
				dmin = edged[i];
				imin = i;
			}
		}
		const float* va = &verts[imin*3];
		const float* vb = &verts[((imin+1)%nv)*3];
		dtVlerp(resultPos, va, vb, edget[imin]);
		
		// Check to see if the point is on the portal edge to the next polygon.
		if (n+1 >= pathSize)
			return n;
		// TODO: optimize
		float left[3], right[3];
		unsigned char fromType, toType;
		if (!getPortalPoints(path[n], path[n+1], left, right, fromType, toType))
			return n;
		// If the dtClamped point is close to the next portal edge, advance to next poly.
		float t;
		const float d = dtDistancePtSegSqr2D(resultPos, left, right, t);
		if (d > SLOP*SLOP)
			return n;
		// Advance to next polygon.
		n++;
	}
	
	return n;
}

bool dtNavMesh::getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right,
								unsigned char& fromType, unsigned char& toType) const
{
	unsigned int salt, it, ip;
	decodePolyId(from, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	const dtMeshTile* fromTile = &m_tiles[it];
	if (ip >= (unsigned int)fromTile->header->polyCount) return false;
	const dtPoly* fromPoly = &fromTile->polys[ip];
	fromType = fromPoly->type;

	decodePolyId(to, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	const dtMeshTile* toTile = &m_tiles[it];
	if (ip >= (unsigned int)toTile->header->polyCount) return false;
	const dtPoly* toPoly = &toTile->polys[ip];
	toType = toPoly->type;

	return getPortalPoints(from, fromPoly, fromTile,
						   to, toPoly, toTile,
						   left, right);
}

// Returns portal points between two polygons.
bool dtNavMesh::getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
								dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
								float* left, float* right) const
{
	// Find the link that points to the 'to' polygon.
	const dtLink* link = 0;
	for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
	{
		if (fromTile->links[i].ref == to)
		{
			link = &fromTile->links[i];
			break;
		}
	}
	if (!link)
		return false;
	
	// Handle off-mesh connections.
	if (fromPoly->type == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		// Find link that points to first vertex.
		for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
		{
			if (fromTile->links[i].ref == to)
			{
				const int v = fromTile->links[i].edge;
				dtVcopy(left, &fromTile->verts[fromPoly->verts[v]*3]);
				dtVcopy(right, &fromTile->verts[fromPoly->verts[v]*3]);
				return true;
			}
		}
		return false;
	}

	if (toPoly->type == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		for (unsigned int i = toPoly->firstLink; i != DT_NULL_LINK; i = toTile->links[i].next)
		{
			if (toTile->links[i].ref == from)
			{
				const int v = toTile->links[i].edge;
				dtVcopy(left, &toTile->verts[toPoly->verts[v]*3]);
				dtVcopy(right, &toTile->verts[toPoly->verts[v]*3]);
				return true;
			}
		}
		return false;
	}
		
	// Find portal vertices.
	const int v0 = fromPoly->verts[link->edge];
	const int v1 = fromPoly->verts[(link->edge+1) % (int)fromPoly->vertCount];
	dtVcopy(left, &fromTile->verts[v0*3]);
	dtVcopy(right, &fromTile->verts[v1*3]);
	
	// If the link is at tile boundary, dtClamp the vertices to
	// the link width.
	if (link->side == 0 || link->side == 4)
	{
		// Unpack portal limits.
		const float smin = dtMin(left[2],right[2]);
		const float smax = dtMax(left[2],right[2]);
		const float s = (smax-smin) / 255.0f;
		const float lmin = smin + link->bmin*s;
		const float lmax = smin + link->bmax*s;
		left[2] = dtMax(left[2],lmin);
		left[2] = dtMin(left[2],lmax);
		right[2] = dtMax(right[2],lmin);
		right[2] = dtMin(right[2],lmax);
	}
	else if (link->side == 2 || link->side == 6)
	{
		// Unpack portal limits.
		const float smin = dtMin(left[0],right[0]);
		const float smax = dtMax(left[0],right[0]);
		const float s = (smax-smin) / 255.0f;
		const float lmin = smin + link->bmin*s;
		const float lmax = smin + link->bmax*s;
		left[0] = dtMax(left[0],lmin);
		left[0] = dtMin(left[0],lmax);
		right[0] = dtMax(right[0],lmin);
		right[0] = dtMin(right[0],lmax);
	}
	
	return true;
}

// Returns edge mid point between two polygons.
bool dtNavMesh::getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const
{
	float left[3], right[3];
	unsigned char fromType, toType;
	if (!getPortalPoints(from, to, left,right, fromType, toType)) return false;
	mid[0] = (left[0]+right[0])*0.5f;
	mid[1] = (left[1]+right[1])*0.5f;
	mid[2] = (left[2]+right[2])*0.5f;
	return true;
}

bool dtNavMesh::getEdgeMidPoint(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
								dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
								float* mid) const
{
	float left[3], right[3];
	if (!getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right))
		return false;
	mid[0] = (left[0]+right[0])*0.5f;
	mid[1] = (left[1]+right[1])*0.5f;
	mid[2] = (left[2]+right[2])*0.5f;
	return true;
}

void dtNavMesh::setPolyFlags(dtPolyRef ref, unsigned short flags)
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return;
	dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return;
	dtPoly* poly = &tile->polys[ip];
	// Change flags.
	poly->flags = flags;
}

unsigned short dtNavMesh::getPolyFlags(dtPolyRef ref) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	const dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return 0;
	const dtPoly* poly = &tile->polys[ip];
	return poly->flags;
}

void dtNavMesh::setPolyArea(dtPolyRef ref, unsigned char area)
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return;
	dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return;
	dtPoly* poly = &tile->polys[ip];
	poly->area = area;
}

unsigned char dtNavMesh::getPolyArea(dtPolyRef ref) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	const dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return 0;
	const dtPoly* poly = &tile->polys[ip];
	return poly->area;
}

int dtNavMesh::raycast(dtPolyRef centerRef, const float* startPos, const float* endPos, const dtQueryFilter* filter,
					   float& t, float* hitNormal, dtPolyRef* path, const int pathSize) const
{
	t = 0;
	
	if (!centerRef || !getPolyByRef(centerRef))
		return 0;
	
	dtPolyRef curRef = centerRef;
	float verts[DT_VERTS_PER_POLYGON*3];	
	int n = 0;
	
	hitNormal[0] = 0;
	hitNormal[1] = 0;
	hitNormal[2] = 0;
	
	while (curRef)
	{
		// Cast ray against current polygon.
		
		// The API input has been cheked already, skip checking internal data.
		unsigned int it = decodePolyIdTile(curRef);
		unsigned int ip = decodePolyIdPoly(curRef);
		const dtMeshTile* tile = &m_tiles[it];
		const dtPoly* poly = &tile->polys[ip];

		// Collect vertices.
		int nv = 0;
		for (int i = 0; i < (int)poly->vertCount; ++i)
		{
			dtVcopy(&verts[nv*3], &tile->verts[poly->verts[i]*3]);
			nv++;
		}		
		
		float tmin, tmax;
		int segMin, segMax;
		if (!dtIntersectSegmentPoly2D(startPos, endPos, verts, nv, tmin, tmax, segMin, segMax))
		{
			// Could not hit the polygon, keep the old t and report hit.
			return n;
		}
		// Keep track of furthest t so far.
		if (tmax > t)
			t = tmax;

		// Store visited polygons.
		if (n < pathSize)
			path[n++] = curRef;

		// Ray end is completely inside the polygon.
		if (segMax == -1)
		{
			t = FLT_MAX;
			return n;
		}
		
		// Follow neighbours.
		dtPolyRef nextRef = 0;
		
		for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
		{
			const dtLink* link = &tile->links[i];
			
			// Find link which contains this edge.
			if ((int)link->edge != segMax)
				continue;
				
			// Get pointer to the next polygon.
			it = decodePolyIdTile(link->ref);
			ip = decodePolyIdPoly(link->ref);
			const dtMeshTile* nextTile = &m_tiles[it];
			const dtPoly* nextPoly = &nextTile->polys[ip];
			
			// Skip off-mesh connections.
			if (nextPoly->type == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
				
			// Skip links based on filter.
			if (!passFilter(filter, nextPoly->flags))
				continue;
		
			// If the link is internal, just return the ref.
			if (link->side == 0xff)
			{
				nextRef = link->ref;
				break;
			}
			
			// If the link is at tile boundary,
			const int v0 = poly->verts[link->edge];
			const int v1 = poly->verts[(link->edge+1) % poly->vertCount];
			const float* left = &tile->verts[v0*3];
			const float* right = &tile->verts[v1*3];
			
			// Check that the intersection lies inside the link portal.
			if (link->side == 0 || link->side == 4)
			{
				// Calculate link size.
				const float smin = dtMin(left[2],right[2]);
				const float smax = dtMax(left[2],right[2]);
				const float s = (smax-smin) / 255.0f;
				const float lmin = smin + link->bmin*s;
				const float lmax = smin + link->bmax*s;
				// Find Z intersection.
				float z = startPos[2] + (endPos[2]-startPos[2])*tmax;
				if (z >= lmin && z <= lmax)
				{
					nextRef = link->ref;
					break;
				}
			}
			else if (link->side == 2 || link->side == 6)
			{
				// Calculate link size.
				const float smin = dtMin(left[0],right[0]);
				const float smax = dtMax(left[0],right[0]);
				const float s = (smax-smin) / 255.0f;
				const float lmin = smin + link->bmin*s;
				const float lmax = smin + link->bmax*s;
				// Find X intersection.
				float x = startPos[0] + (endPos[0]-startPos[0])*tmax;
				if (x >= lmin && x <= lmax)
				{
					nextRef = link->ref;
					break;
				}
			}
		}
		
		if (!nextRef)
		{
			// No neighbour, we hit a wall.

			// Calculate hit normal.
			const int a = segMax;
			const int b = segMax+1 < nv ? segMax+1 : 0;
			const float* va = &verts[a*3];
			const float* vb = &verts[b*3];
			const float dx = vb[0] - va[0];
			const float dz = vb[2] - va[2];
			hitNormal[0] = dz;
			hitNormal[1] = 0;
			hitNormal[2] = -dx;
			dtVnormalize(hitNormal);
			
			return n;
		}
		
		// No hit, advance to neighbour polygon.
		curRef = nextRef;
	}
	
	return n;
}

int dtNavMesh::findPolysAround(dtPolyRef centerRef, const float* centerPos, float radius, const dtQueryFilter* filter,
									dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
									const int maxResult) const
{
	if (!centerRef) return 0;
	if (!getPolyByRef(centerRef)) return 0;
	if (!m_nodePool || !m_openList) return 0;
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtNode* startNode = m_nodePool->getNode(centerRef);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = centerRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	int n = 0;
	if (n < maxResult)
	{
		if (resultRef)
			resultRef[n] = startNode->id;
		if (resultParent)
			resultParent[n] = 0;
		if (resultCost)
			resultCost[n] = 0;
		++n;
	}
	
	const float radiusSqr = dtSqr(radius);

	unsigned int it, ip;
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();

		float previousEdgeMidPoint[3];

		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		it = decodePolyIdTile(bestRef);
		ip = decodePolyIdPoly(bestRef);
		const dtMeshTile* bestTile = &m_tiles[it];
		const dtPoly* bestPoly = &bestTile->polys[ip];

		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
		{
			it = decodePolyIdTile(parentRef);
			ip = decodePolyIdPoly(parentRef);
			parentTile = &m_tiles[it];
			parentPoly = &parentTile->polys[ip];
			
			getEdgeMidPoint(parentRef, parentPoly, parentTile,
							bestRef, bestPoly, bestTile, previousEdgeMidPoint);
		}
		else
		{
			dtVcopy(previousEdgeMidPoint, centerPos);
		}
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const dtLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;

			// Calc distance to the edge.
			const float* va = &bestTile->verts[bestPoly->verts[link->edge]*3];
			const float* vb = &bestTile->verts[bestPoly->verts[(link->edge+1) % bestPoly->vertCount]*3];
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			
			// If the circle is not touching the next polygon, skip it.
			if (distSqr > radiusSqr)
				continue;

			// Expand to neighbour
			it = decodePolyIdTile(neighbourRef);
			ip = decodePolyIdPoly(neighbourRef);
			const dtMeshTile* neighbourTile = &m_tiles[it];
			const dtPoly* neighbourPoly = &neighbourTile->polys[ip];
			
			if (!passFilter(filter, neighbourPoly->flags))
				continue;
			
			dtNode newNode;
			newNode.pidx = m_nodePool->getNodeIdx(bestNode);
			newNode.id = neighbourRef;

			// Cost
			float edgeMidPoint[3];
			getEdgeMidPoint(bestRef, bestPoly, bestTile,
							neighbourRef, neighbourPoly, neighbourTile, edgeMidPoint);
			
			newNode.total = bestNode->total + dtVdist(previousEdgeMidPoint, edgeMidPoint);
			
			dtNode* actualNode = m_nodePool->getNode(newNode.id);
			if (!actualNode)
				continue;
			
			if (!((actualNode->flags & DT_NODE_OPEN) && newNode.total > actualNode->total) &&
				!((actualNode->flags & DT_NODE_CLOSED) && newNode.total > actualNode->total))
			{
				actualNode->flags &= ~DT_NODE_CLOSED;
				actualNode->pidx = newNode.pidx;
				actualNode->total = newNode.total;
				
				if (actualNode->flags & DT_NODE_OPEN)
				{
					m_openList->modify(actualNode);
				}
				else
				{
					if (n < maxResult)
					{
						if (resultRef)
							resultRef[n] = actualNode->id;
						if (resultParent)
							resultParent[n] = m_nodePool->getNodeAtIdx(actualNode->pidx)->id;
						if (resultCost)
							resultCost[n] = actualNode->total;
						++n;
					}
					actualNode->flags = DT_NODE_OPEN;
					m_openList->push(actualNode);
				}
			}
		}
	}
	
	return n;
}

float dtNavMesh::findDistanceToWall(dtPolyRef centerRef, const float* centerPos, float maxRadius, const dtQueryFilter* filter,
									float* hitPos, float* hitNormal) const
{
	if (!centerRef) return 0;
	if (!getPolyByRef(centerRef)) return 0;
	if (!m_nodePool || !m_openList) return 0;
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtNode* startNode = m_nodePool->getNode(centerRef);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = centerRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	float radiusSqr = dtSqr(maxRadius);
	
	unsigned int it, ip;
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		
		float previousEdgeMidPoint[3];
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		it = decodePolyIdTile(bestRef);
		ip = decodePolyIdPoly(bestRef);
		const dtMeshTile* bestTile = &m_tiles[it];
		const dtPoly* bestPoly = &bestTile->polys[ip];
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
		{
			it = decodePolyIdTile(parentRef);
			ip = decodePolyIdPoly(parentRef);
			parentTile = &m_tiles[it];
			parentPoly = &parentTile->polys[ip];
			
			getEdgeMidPoint(parentRef, parentPoly, parentTile,
							bestRef, bestPoly, bestTile, previousEdgeMidPoint);
		}
		else
		{
			dtVcopy(previousEdgeMidPoint, centerPos);
		}
		
		// Hit test walls.
		for (int i = 0, j = (int)bestPoly->vertCount-1; i < (int)bestPoly->vertCount; j = i++)
		{
			// Skip non-solid edges.
			if (bestPoly->neis[j] & DT_EXT_LINK)
			{
				// Tile border.
				bool solid = true;
				for (unsigned int k = bestPoly->firstLink; k != DT_NULL_LINK; k = bestTile->links[k].next)
				{
					const dtLink* link = &bestTile->links[k];
					if (link->edge == j)
					{
						if (link->ref != 0 && passFilter(filter, getPolyFlags(link->ref)))
							solid = false;
						break;
					}
				}
				if (!solid) continue;
			}
			else if (bestPoly->neis[j] && passFilter(filter, bestTile->polys[bestPoly->neis[j]].flags))
			{
				// Internal edge
				continue;
			}
			
			// Calc distance to the edge.
			const float* vj = &bestTile->verts[bestPoly->verts[j]*3];
			const float* vi = &bestTile->verts[bestPoly->verts[i]*3];
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, vj, vi, tseg);
			
			// Edge is too far, skip.
			if (distSqr > radiusSqr)
				continue;
			
			// Hit wall, update radius.
			radiusSqr = distSqr;
			// Calculate hit pos.
			hitPos[0] = vj[0] + (vi[0] - vj[0])*tseg;
			hitPos[1] = vj[1] + (vi[1] - vj[1])*tseg;
			hitPos[2] = vj[2] + (vi[2] - vj[2])*tseg;
		}
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const dtLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Calc distance to the edge.
			const float* va = &bestTile->verts[bestPoly->verts[link->edge]*3];
			const float* vb = &bestTile->verts[bestPoly->verts[(link->edge+1) % bestPoly->vertCount]*3];
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			
			// If the circle is not touching the next polygon, skip it.
			if (distSqr > radiusSqr)
				continue;
			
			// Expand to neighbour.
			it = decodePolyIdTile(neighbourRef);
			ip = decodePolyIdPoly(neighbourRef);
			const dtMeshTile* neighbourTile = &m_tiles[it];
			const dtPoly* neighbourPoly = &neighbourTile->polys[ip];
			
			if (!passFilter(filter, neighbourPoly->flags))
				continue;
			
			dtNode newNode;
			newNode.pidx = m_nodePool->getNodeIdx(bestNode);
			newNode.id = neighbourRef;
			
			// Cost
			float edgeMidPoint[3];
			getEdgeMidPoint(bestRef, bestPoly, bestTile,
							neighbourRef, neighbourPoly, neighbourTile, edgeMidPoint);

			newNode.total = bestNode->total + dtVdist(previousEdgeMidPoint, edgeMidPoint);
			
			dtNode* actualNode = m_nodePool->getNode(newNode.id);
			if (!actualNode)
				continue;
			
			if (!((actualNode->flags & DT_NODE_OPEN) && newNode.total > actualNode->total) &&
				!((actualNode->flags & DT_NODE_CLOSED) && newNode.total > actualNode->total))
			{
				actualNode->flags &= ~DT_NODE_CLOSED;
				actualNode->pidx = newNode.pidx;
				actualNode->total = newNode.total;
				
				if (actualNode->flags & DT_NODE_OPEN)
				{
					m_openList->modify(actualNode);
				}
				else
				{
					actualNode->flags = DT_NODE_OPEN;
					m_openList->push(actualNode);
				}
			}
		}
	}
	
	// Calc hit normal.
	dtVsub(hitNormal, centerPos, hitPos);
	dtVnormalize(hitNormal);
	
	return sqrtf(radiusSqr);
}

const dtPoly* dtNavMesh::getPolyByRef(dtPolyRef ref) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return 0;
	return &m_tiles[it].polys[ip];
}

const float* dtNavMesh::getPolyVertsByRef(dtPolyRef ref) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return 0;
	return m_tiles[it].verts;
}

const dtLink* dtNavMesh::getPolyLinksByRef(dtPolyRef ref) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return 0;
	return m_tiles[it].links;
}

bool dtNavMesh::isInClosedList(dtPolyRef ref) const
{
	if (!m_nodePool) return false;
	const dtNode* node = m_nodePool->findNode(ref);
	return node && node->flags & DT_NODE_CLOSED;
}

