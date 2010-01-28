//
// Copyright (c) 2009 Mikko Mononen memon@inside.org
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

static void calcRect(const float* va, const float* vb,
					 float* bmin, float* bmax,
					 int side, float padx, float pady)
{
	if (side == 0 || side == 4)
	{
		bmin[0] = min(va[2],vb[2]) + padx;
		bmin[1] = min(va[1],vb[1]) - pady;
		bmax[0] = max(va[2],vb[2]) - padx;
		bmax[1] = max(va[1],vb[1]) + pady;
	}
	else if (side == 2 || side == 6)
	{
		bmin[0] = min(va[0],vb[0]) + padx;
		bmin[1] = min(va[1],vb[1]) - pady;
		bmax[0] = max(va[0],vb[0]) - padx;
		bmax[1] = max(va[1],vb[1]) + pady;
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
	if (tile->header->linksFreeList == DT_NULL_LINK)
		return DT_NULL_LINK;
	unsigned int link = tile->header->linksFreeList;
	tile->header->linksFreeList = tile->header->links[link].next;
	return link;
}

inline void freeLink(dtMeshTile* tile, unsigned int link)
{
	tile->header->links[link].next = tile->header->linksFreeList;
	tile->header->linksFreeList = link;
}


inline bool passFilter(dtQueryFilter* filter, unsigned short flags)
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
}

dtNavMesh::~dtNavMesh()
{
	for (int i = 0; i < m_maxTiles; ++i)
	{
		if (m_tiles[i].data && m_tiles[i].ownsData)
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
		
bool dtNavMesh::init(const float* orig, float tileWidth, float tileHeight,
					 int maxTiles, int maxPolys, int maxNodes)
{
	vcopy(m_orig, orig);
	m_tileWidth = tileWidth;
	m_tileHeight = tileHeight;
	
	// Init tiles
	m_maxTiles = maxTiles;
	m_tileLutSize = nextPow2(maxTiles/4);
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
		m_nodePool = new dtNodePool(maxNodes, nextPow2(maxNodes/4));
		if (!m_nodePool)
			return false;
	}
	
	if (!m_openList)
	{
		m_openList = new dtNodeQueue(maxNodes);
		if (!m_openList)
			return false;
	}
	
	// Init ID generator values.
	m_tileBits = max((unsigned int)1,ilog2(nextPow2((unsigned int)maxTiles)));
	m_polyBits = max((unsigned int)1,ilog2(nextPow2((unsigned int)maxPolys)));
	m_saltBits = 32 - m_tileBits - m_polyBits;
	if (m_saltBits < 10)
		return false;
	
	return true;
}

bool dtNavMesh::init(unsigned char* data, int dataSize, bool ownsData, int maxNodes)
{
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return false;
	if (header->version != DT_NAVMESH_VERSION)
		return false;

	const float w = header->bmax[0] - header->bmin[0];
	const float h = header->bmax[2] - header->bmin[2];
	if (!init(header->bmin, w, h, 1, header->polyCount, maxNodes))
		return false;

	return addTileAt(0,0, data, dataSize, ownsData);
}

//////////////////////////////////////////////////////////////////////////////////////////
int dtNavMesh::findConnectingPolys(const float* va, const float* vb,
								   dtMeshTile* tile, int side,
								   dtPolyRef* con, float* conarea, int maxcon)
{
	if (!tile) return 0;
	dtMeshHeader* h = tile->header;
	
	float amin[2], amax[2];
	calcRect(va,vb, amin,amax, side, 0.01f, h->walkableClimb);

	// Remove links pointing to 'side' and compact the links array. 
	float bmin[2], bmax[2];
	unsigned short m = DT_EXT_LINK | (unsigned short)side;
	int n = 0;
	
	dtPolyRef base = getTileId(tile);
	
	for (int i = 0; i < h->polyCount; ++i)
	{
		dtPoly* poly = &h->polys[i];
		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip edges which do not point to the right side.
			if (poly->neis[j] != m) continue;
			// Check if the segments touch.
			const float* vc = &h->verts[poly->verts[j]*3];
			const float* vd = &h->verts[poly->verts[(j+1) % nv]*3];
			calcRect(vc,vd, bmin,bmax, side, 0.01f, h->walkableClimb);
			if (!overlapRects(amin,amax, bmin,bmax)) continue;
			// Add return value.
			if (n < maxcon)
			{
				conarea[n*2+0] = max(amin[0], bmin[0]);
				conarea[n*2+1] = min(amax[0], bmax[0]);
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
	dtMeshHeader* header = tile->header;

	for (int i = 0; i < header->polyCount; ++i)
	{
		dtPoly* poly = &header->polys[i];
		unsigned int j = poly->firstLink;
		unsigned int pj = DT_NULL_LINK;
		while (j != DT_NULL_LINK)
		{
			if (header->links[j].side == side)
			{
				// Revove link.
				unsigned int nj = header->links[j].next;
				if (pj == DT_NULL_LINK)
					poly->firstLink = nj;
				else
					header->links[pj].next = nj;
				freeLink(tile, j);
				j = nj;
			}
			else
			{
				// Advance
				pj = j;
				j = header->links[j].next;
			}
		}
	}
}

void dtNavMesh::connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side)
{
	if (!tile) return;
	dtMeshHeader* header = tile->header;
	
	// Connect border links.
	for (int i = 0; i < header->polyCount; ++i)
	{
		dtPoly* poly = &header->polys[i];

		// Create new links.
		unsigned short m = DT_EXT_LINK | (unsigned short)side;
		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip edges which do not point to the right side.
			if (poly->neis[j] != m) continue;
			
			// Create new links
			const float* va = &header->verts[poly->verts[j]*3];
			const float* vb = &header->verts[poly->verts[(j+1) % nv]*3];
			dtPolyRef nei[4];
			float neia[4*2];
			int nnei = findConnectingPolys(va,vb, target, opposite(side), nei,neia,4);
			for (int k = 0; k < nnei; ++k)
			{
				unsigned int idx = allocLink(tile);
				if (idx != DT_NULL_LINK)
				{
					dtLink* link = &header->links[idx];
					link->ref = nei[k];
					link->edge = (unsigned char)j;
					link->side = (unsigned char)side;
					
					link->next = poly->firstLink;
					poly->firstLink = idx;

					// Compress portal limits to a byte value.
					if (side == 0 || side == 4)
					{
						const float lmin = min(va[2], vb[2]);
						const float lmax = max(va[2], vb[2]);
						link->bmin = (unsigned char)(clamp((neia[k*2+0]-lmin)/(lmax-lmin), 0.0f, 1.0f)*255.0f);
						link->bmax = (unsigned char)(clamp((neia[k*2+1]-lmin)/(lmax-lmin), 0.0f, 1.0f)*255.0f);
					}
					else if (side == 2 || side == 6)
					{
						const float lmin = min(va[0], vb[0]);
						const float lmax = max(va[0], vb[0]);
						link->bmin = (unsigned char)(clamp((neia[k*2+0]-lmin)/(lmax-lmin), 0.0f, 1.0f)*255.0f);
						link->bmax = (unsigned char)(clamp((neia[k*2+1]-lmin)/(lmax-lmin), 0.0f, 1.0f)*255.0f);
					}
				}
			}
		}
	}
}

void dtNavMesh::connectExtOffMeshLinks(dtMeshTile* tile, dtMeshTile* target, int side)
{
	if (!tile) return;
	dtMeshHeader* header = tile->header;
	
	// Connect off-mesh links.
	// We are interested on links which land from target tile to this tile.
	dtMeshHeader* targetHeader = target->header;
	const unsigned char oppositeSide = (unsigned char)opposite(side);
	dtQueryFilter defaultFilter;
	
	for (int i = 0; i < targetHeader->offMeshConCount; ++i)
	{
		dtOffMeshConnection* targetCon = &targetHeader->offMeshCons[i];
		if (targetCon->side != oppositeSide)
			continue;
		
		dtPoly* targetPoly = &targetHeader->polys[targetCon->poly];
		
		const float ext[3] = { targetCon->rad, targetHeader->walkableClimb, targetCon->rad };
		
		// Find polygon to connect to.
		const float* p = &targetCon->pos[3];
		float nearestPt[3];
		dtPolyRef ref = findNearestPolyInTile(tile, p, ext, &defaultFilter, nearestPt);
		if (!ref) continue;
		// findNearestPoly may return too optimistic results, further check to make sure. 
		if (sqr(nearestPt[0]-p[0])+sqr(nearestPt[2]-p[2]) > sqr(targetCon->rad))
			continue;
		// Make sure the location is on current mesh.
		float* v = &targetHeader->verts[targetPoly->verts[1]*3];
		vcopy(v, nearestPt);
				
		// Link off-mesh connection to target poly.
		unsigned int idx = allocLink(target);
		if (idx != DT_NULL_LINK)
		{
			dtLink* link = &targetHeader->links[idx];
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
				unsigned short landPolyIdx = decodePolyIdPoly(ref);
				dtPoly* landPoly = &header->polys[landPolyIdx];
				dtLink* link = &header->links[idx];
				link->ref = getTileId(target) | (unsigned int)(targetCon->poly);
				link->edge = 0;
				link->side = side;
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
	dtMeshHeader* header = tile->header;

	dtPolyRef base = getTileId(tile);

	for (int i = 0; i < header->polyCount; ++i)
	{
		dtPoly* poly = &header->polys[i];
		poly->firstLink = DT_NULL_LINK;

		if (poly->flags & DT_POLY_OFFMESH_CONNECTION)
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
				dtLink* link = &header->links[idx];
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
	dtMeshHeader* header = tile->header;
	
	dtPolyRef base = getTileId(tile);
	
	// Find Off-mesh connection end points.
	for (int i = 0; i < header->offMeshConCount; ++i)
	{
		dtOffMeshConnection* con = &header->offMeshCons[i];
		dtPoly* poly = &header->polys[con->poly];
		dtQueryFilter defaultFilter;
	
		const float ext[3] = { con->rad, header->walkableClimb, con->rad };
		
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
				if (sqr(nearestPt[0]-p[0])+sqr(nearestPt[2]-p[2]) > sqr(con->rad))
					continue;
				// Make sure the location is on current mesh.
				float* v = &header->verts[poly->verts[j]*3];
				vcopy(v, nearestPt);

				// Link off-mesh connection to target poly.
				unsigned int idx = allocLink(tile);
				if (idx != DT_NULL_LINK)
				{
					dtLink* link = &header->links[idx];
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
						unsigned short landPolyIdx = decodePolyIdPoly(ref);
						dtPoly* landPoly = &header->polys[landPolyIdx];
						dtLink* link = &header->links[idx];
						link->ref = base | (unsigned int)(con->poly);
						link->edge = 0;
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

bool dtNavMesh::addTileAt(int x, int y, unsigned char* data, int dataSize, bool ownsData)
{
	if (getTileAt(x,y))
		return false;
	// Make sure there is enough space for new tile.
	if (!m_nextFree)
		return false;
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return false;
	if (header->version != DT_NAVMESH_VERSION)
		return false;
		
	// Allocate a tile.
	dtMeshTile* tile = m_nextFree;
	m_nextFree = tile->next;
	tile->next = 0;

	// Insert tile into the position lut.
	int h = computeTileHash(x,y,m_tileLutMask);
	tile->next = m_posLookup[h];
	m_posLookup[h] = tile;
	
	// Patch header pointers.
	const int headerSize = align4(sizeof(dtMeshHeader));
	const int vertsSize = align4(sizeof(float)*3*header->vertCount);
	const int polysSize = align4(sizeof(dtPoly)*header->polyCount);
	const int linksSize = align4(sizeof(dtLink)*(header->maxLinkCount));
	const int detailMeshesSize = align4(sizeof(dtPolyDetail)*header->detailMeshCount);
	const int detailVertsSize = align4(sizeof(float)*3*header->detailVertCount);
	const int detailTrisSize = align4(sizeof(unsigned char)*4*header->detailTriCount);
	const int bvtreeSize = align4(sizeof(dtBVNode)*header->bvNodeCount);
	const int offMeshLinksSize = align4(sizeof(dtOffMeshConnection)*header->offMeshConCount);
	
	unsigned char* d = data + headerSize;
	header->verts = (float*)d; d += vertsSize;
	header->polys = (dtPoly*)d; d += polysSize;
	header->links = (dtLink*)d; d += linksSize;
	header->detailMeshes = (dtPolyDetail*)d; d += detailMeshesSize;
	header->detailVerts = (float*)d; d += detailVertsSize;
	header->detailTris = (unsigned char*)d; d += detailTrisSize;
	header->bvTree = (dtBVNode*)d; d += bvtreeSize;
	header->offMeshCons = (dtOffMeshConnection*)d; d += offMeshLinksSize;

	// Build links freelist
	header->linksFreeList = 0;
	header->links[header->maxLinkCount-1].next = DT_NULL_LINK;
	for (int i = 0; i < header->maxLinkCount-1; ++i)
		header->links[i].next = i+1;

	// Init tile.
	tile->header = header;
	tile->x = x;
	tile->y = y;
	tile->data = data;
	tile->dataSize = dataSize;
	tile->ownsData = ownsData;

	connectIntLinks(tile);
	connectIntOffMeshLinks(tile);

	// Create connections connections.
	for (int i = 0; i < 8; ++i)
	{
		dtMeshTile* nei = getNeighbourTileAt(x,y,i);
		if (nei)
		{
			connectExtLinks(tile, nei, i);
			connectExtLinks(nei, tile, opposite(i));
			connectExtOffMeshLinks(tile, nei, i);
			connectExtOffMeshLinks(nei, tile, opposite(i));
		}
	}
	
	return true;
}

dtMeshTile* dtNavMesh::getTileAt(int x, int y)
{
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->x == x && tile->y == y)
			return tile;
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

const dtMeshTile* dtNavMesh::getTileByRef(dtPolyRef ref, int* polyIndex) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return 0;
	if (polyIndex) *polyIndex = (int)ip;
	return &m_tiles[it];
}

dtMeshTile* dtNavMesh::getNeighbourTileAt(int x, int y, int side)
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

bool dtNavMesh::removeTileAt(int x, int y, unsigned char** data, int* dataSize)
{
	// Remove tile from hash lookup.
	int h = computeTileHash(x,y,m_tileLutMask);
	dtMeshTile* prev = 0;
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->x == x && tile->y == y)
		{
			if (prev)
				prev->next = tile->next;
			else
				m_posLookup[h] = tile->next;
			break;
		}
		prev = tile;
		tile = tile->next;
	}
	if (!tile)
		return false;
	
	// Remove connections to neighbour tiles.
	for (int i = 0; i < 8; ++i)
	{
		dtMeshTile* nei = getNeighbourTileAt(x,y,i);
		if (!nei) continue;
		unconnectExtLinks(nei, opposite(i));
	}
	
	
	// Reset tile.
	if (tile->ownsData)
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
	tile->x = tile->y = 0;
	tile->salt++;

	// Add to free list.
	tile->next = m_nextFree;
	m_nextFree = tile;

	return true;
}

dtPolyRef dtNavMesh::getTileId(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = tile - m_tiles;
	return encodePolyId(tile->salt, it, 0);
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
	const dtMeshHeader* header = tile->header;
	const dtPoly* poly = &header->polys[ip];
	
	float closestDistSqr = FLT_MAX;
	const dtPolyDetail* pd = &header->detailMeshes[ip];
	
	for (int j = 0; j < pd->triCount; ++j)
	{
		const unsigned char* t = &header->detailTris[(pd->triBase+j)*4];
		const float* v[3];
		for (int k = 0; k < 3; ++k)
		{
			if (t[k] < poly->vertCount)
				v[k] = &header->verts[poly->verts[t[k]]*3];
			else
				v[k] = &header->detailVerts[(pd->vertBase+(t[k]-poly->vertCount))*3];
		}
		float pt[3];
		closestPtPointTriangle(pt, pos, v[0], v[1], v[2]);
		float d = vdistSqr(pos, pt);
		if (d < closestDistSqr)
		{
			vcopy(closest, pt);
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
	const dtMeshHeader* header = m_tiles[it].header;
	
	if (ip >= (unsigned int)header->polyCount) return false;
	const dtPoly* poly = &header->polys[ip];

	// Collect vertices.
	float verts[DT_VERTS_PER_POLYGON*3];	
	float edged[DT_VERTS_PER_POLYGON];
	float edget[DT_VERTS_PER_POLYGON];
	int nv = 0;
	for (int i = 0; i < (int)poly->vertCount; ++i)
	{
		vcopy(&verts[nv*3], &header->verts[poly->verts[i]*3]);
		nv++;
	}		
	
	bool inside = distancePtPolyEdgesSqr(pos, verts, nv, edged, edget);
	if (inside)
	{
		// Point is inside the polygon, return the point.
		vcopy(closest, pos);
	}
	else
	{
		// Point is outside the polygon, clamp to nearest edge.
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
		vlerp(closest, va, vb, edget[imin]);
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
	const dtMeshHeader* header = m_tiles[it].header;
	if (ip >= (unsigned int)header->polyCount) return false;
	const dtPoly* poly = &header->polys[ip];

	// Make sure that the current poly is indeed off-mesh link.
	if ((poly->flags & DT_POLY_OFFMESH_CONNECTION) == 0)
		return false;

	// Figure out which way to hand out the vertices.
	int idx0 = 0, idx1 = 1;
	
	// Find link that points to first vertex.
	for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = header->links[i].next)
	{
		if (header->links[i].edge == 0)
		{
			if (header->links[i].ref != prevRef)
			{
				idx0 = 1;
				idx1 = 0;
			}
			break;
		}
	}
	
	vcopy(startPos, &header->verts[poly->verts[idx0]*3]);
	vcopy(endPos, &header->verts[poly->verts[idx1]*3]);

	return true;
}


bool dtNavMesh::getPolyHeight(dtPolyRef ref, const float* pos, float* height) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	const dtMeshHeader* header = m_tiles[it].header;
	
	if (ip >= (unsigned int)header->polyCount) return false;
	const dtPoly* poly = &header->polys[ip];
	
	if (poly->flags & DT_POLY_OFFMESH_CONNECTION)
	{
		const float* v0 = &header->verts[poly->verts[0]*3];
		const float* v1 = &header->verts[poly->verts[1]*3];
		const float d0 = vdist(pos, v0);
		const float d1 = vdist(pos, v1);
		const float u = d0 / (d0+d1);
		if (height)
			*height = v0[1] + (v1[1] - v0[1]) * u;
		return true;
	}
	else
	{
		const dtPolyDetail* pd = &header->detailMeshes[ip];
		for (int j = 0; j < pd->triCount; ++j)
		{
			const unsigned char* t = &header->detailTris[(pd->triBase+j)*4];
			const float* v[3];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < poly->vertCount)
					v[k] = &header->verts[poly->verts[t[k]]*3];
				else
					v[k] = &header->detailVerts[(pd->vertBase+(t[k]-poly->vertCount))*3];
			}
			float h;
			if (closestHeightPointTriangle(pos, v[0], v[1], v[2], h))
			{
				if (height)
					*height = h;
				return true;
			}
		}
	}
	
	return false;
}


dtPolyRef dtNavMesh::findNearestPoly(const float* center, const float* extents, dtQueryFilter* filter, float* nearestPt)
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
		float d = vdistSqr(center, closestPtPoly);
		if (d < nearestDistanceSqr)
		{
			if (nearestPt)
				vcopy(nearestPt, closestPtPoly);
			nearestDistanceSqr = d;
			nearest = ref;
		}
	}
	
	return nearest;
}

dtPolyRef dtNavMesh::findNearestPolyInTile(dtMeshTile* tile, const float* center, const float* extents,
										   dtQueryFilter* filter, float* nearestPt)
{
	float bmin[3], bmax[3];
	vsub(bmin, center, extents);
	vadd(bmax, center, extents);
	
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
		float d = vdistSqr(center, closestPtPoly);
		if (d < nearestDistanceSqr)
		{
			if (nearestPt)
				vcopy(nearestPt, closestPtPoly);
			nearestDistanceSqr = d;
			nearest = ref;
		}
	}
	
	return nearest;
}

int dtNavMesh::queryPolygonsInTile(dtMeshTile* tile, const float* qmin, const float* qmax,
								   dtQueryFilter* filter,
								   dtPolyRef* polys, const int maxPolys)
{
	const dtMeshHeader* header = tile->header;
	if (header->bvTree)
	{
		const dtBVNode* node = &header->bvTree[0];
		const dtBVNode* end = &header->bvTree[header->bvNodeCount];
			
		// Calculate quantized box
		unsigned short bmin[3], bmax[3];
		// Clamp query box to world box.
		float minx = clamp(qmin[0], header->bmin[0], header->bmax[0]) - header->bmin[0];
		float miny = clamp(qmin[1], header->bmin[1], header->bmax[1]) - header->bmin[1];
		float minz = clamp(qmin[2], header->bmin[2], header->bmax[2]) - header->bmin[2];
		float maxx = clamp(qmax[0], header->bmin[0], header->bmax[0]) - header->bmin[0];
		float maxy = clamp(qmax[1], header->bmin[1], header->bmax[1]) - header->bmin[1];
		float maxz = clamp(qmax[2], header->bmin[2], header->bmax[2]) - header->bmin[2];
		// Quantize
		bmin[0] = (unsigned short)(header->bvQuantFactor * minx) & 0xfffe;
		bmin[1] = (unsigned short)(header->bvQuantFactor * miny) & 0xfffe;
		bmin[2] = (unsigned short)(header->bvQuantFactor * minz) & 0xfffe;
		bmax[0] = (unsigned short)(header->bvQuantFactor * maxx + 1) | 1;
		bmax[1] = (unsigned short)(header->bvQuantFactor * maxy + 1) | 1;
		bmax[2] = (unsigned short)(header->bvQuantFactor * maxz + 1) | 1;
			
		// Traverse tree
		dtPolyRef base = getTileId(tile);
		int n = 0;
		while (node < end)
		{
			bool overlap = checkOverlapBox(bmin, bmax, node->bmin, node->bmax);
			bool isLeafNode = node->i >= 0;
			
			if (isLeafNode && overlap)
			{
				if (passFilter(filter, header->polys[node->i].flags))
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
		const dtMeshHeader* header = tile->header;
		int n = 0;
		dtPolyRef base = getTileId(tile);
		for (int i = 0; i < header->polyCount; ++i)
		{
			// Calc polygon bounds.
			dtPoly* p = &header->polys[i];
			const float* v = &header->verts[p->verts[0]*3];
			vcopy(bmin, v);
			vcopy(bmax, v);
			for (int j = 1; j < p->vertCount; ++j)
			{
				v = &header->verts[p->verts[j]*3];
				vmin(bmin, v);
				vmax(bmax, v);
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

int dtNavMesh::queryPolygons(const float* center, const float* extents, dtQueryFilter* filter,
							 dtPolyRef* polys, const int maxPolys)
{
	float bmin[3], bmax[3];
	vsub(bmin, center, extents);
	vadd(bmax, center, extents);
	
	// Find tiles the query touches.
	const int minx = (int)floorf((bmin[0]-m_orig[0]) / m_tileWidth);
	const int maxx = (int)ceilf((bmax[0]-m_orig[0]) / m_tileWidth);

	const int miny = (int)floorf((bmin[2]-m_orig[2]) / m_tileHeight);
	const int maxy = (int)ceilf((bmax[2]-m_orig[2]) / m_tileHeight);

	int n = 0;
	for (int y = miny; y < maxy; ++y)
	{
		for (int x = minx; x < maxx; ++x)
		{
			dtMeshTile* tile = getTileAt(x,y);
			if (!tile) continue;
			n += queryPolygonsInTile(tile, bmin, bmax, filter, polys+n, maxPolys-n);
			if (n >= maxPolys) return n;
		}
	}

	return n;
}

int dtNavMesh::findPath(dtPolyRef startRef, dtPolyRef endRef,
						const float* startPos, const float* endPos,
						dtQueryFilter* filter,
						dtPolyRef* path, const int maxPathSize)
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
	
	static const float H_SCALE = 1.1f;	// Heuristic scale.
	
	dtNode* startNode = m_nodePool->getNode(startRef);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = vdist(startPos, endPos) * H_SCALE;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	dtNode* lastBestNode = startNode;
	float lastBestNodeCost = startNode->total;
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		
		if (bestNode->id == endRef)
		{
			lastBestNode = bestNode;
			break;
		}

		// Get poly and tile.
		unsigned int salt, it, ip;
		decodePolyId(bestNode->id, salt, it, ip);
		// The API input has been cheked already, skip checking internal data.
		const dtMeshHeader* header = m_tiles[it].header;
		const dtPoly* poly = &header->polys[ip];
		
		for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = header->links[i].next)
		{
			dtPolyRef neighbour = header->links[i].ref;
			if (neighbour)
			{
				// Skip parent node.
				if (bestNode->pidx && m_nodePool->getNodeAtIdx(bestNode->pidx)->id == neighbour)
					continue;

				// TODO: Avoid digging the polygon (done in getEdgeMidPoint too).
				if (!passFilter(filter, getPolyFlags(neighbour)))
					continue;

				dtNode* parent = bestNode;
				dtNode newNode;
				newNode.pidx = m_nodePool->getNodeIdx(parent);
				newNode.id = neighbour;

				// Calculate cost.
				float p0[3], p1[3];
				if (!parent->pidx)
					vcopy(p0, startPos);
				else
					getEdgeMidPoint(m_nodePool->getNodeAtIdx(parent->pidx)->id, parent->id, p0);
				
				getEdgeMidPoint(parent->id, newNode.id, p1);
				
				newNode.cost = parent->cost + vdist(p0,p1);
				// Special case for last node.
				if (newNode.id == endRef)
					newNode.cost += vdist(p1, endPos);

				// Heuristic
				const float h = vdist(p1,endPos)*H_SCALE;
				newNode.total = newNode.cost + h;
				
				dtNode* actualNode = m_nodePool->getNode(newNode.id);
				if (!actualNode)
					continue;
				
				if (!((actualNode->flags & DT_NODE_OPEN) && newNode.total > actualNode->total) &&
					!((actualNode->flags & DT_NODE_CLOSED) && newNode.total > actualNode->total))
				{
					actualNode->flags &= ~DT_NODE_CLOSED;
					actualNode->pidx = newNode.pidx;
					actualNode->cost = newNode.cost;
					actualNode->total = newNode.total;
					
					if (h < lastBestNodeCost)
					{
						lastBestNodeCost = h;
						lastBestNode = actualNode;
					}
					
					if (actualNode->flags & DT_NODE_OPEN)
					{
						m_openList->modify(actualNode);
					}
					else
					{
						actualNode->flags |= DT_NODE_OPEN;
						m_openList->push(actualNode);
					}
				}
			}
		}
		bestNode->flags |= DT_NODE_CLOSED;
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
								const int maxStraightPathSize)
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
	vcopy(&straightPath[straightPathSize*3], closestStartPos);
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
		vcopy(portalApex, closestStartPos);
		vcopy(portalLeft, portalApex);
		vcopy(portalRight, portalApex);
		int apexIndex = 0;
		int leftIndex = 0;
		int rightIndex = 0;

		unsigned short leftPolyFlags = 0;
		unsigned short rightPolyFlags = 0;

		dtPolyRef leftPolyRef = path[0];
		dtPolyRef rightPolyRef = path[0];

		for (int i = 0; i < pathSize; ++i)
		{
			float left[3], right[3];
			unsigned short fromFlags, toFlags;
			
			if (i+1 < pathSize)
			{
				// Next portal.
				if (!getPortalPoints(path[i], path[i+1], left, right, fromFlags, toFlags))
				{
					if (!closestPointOnPolyBoundary(path[i], endPos, closestEndPos))
						return 0;
					
					vcopy(&straightPath[straightPathSize*3], closestEndPos);
					if (straightPathFlags)
						straightPathFlags[straightPathSize] = 0;
					if (straightPathRefs)
						straightPathRefs[straightPathSize] = path[i];
					straightPathSize++;
					
					return straightPathSize;
				}
			}
			else
			{
				// End of the path.
				vcopy(left, closestEndPos);
				vcopy(right, closestEndPos);

				fromFlags = toFlags = 0;
			}
			
			// Right vertex.
			if (vequal(portalApex, portalRight))
			{
				vcopy(portalRight, right);
				rightPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
				rightPolyFlags = toFlags;
				rightIndex = i;
			}
			else
			{
				if (triArea2D(portalApex, portalRight, right) <= 0.0f)
				{
					if (triArea2D(portalApex, portalLeft, right) > 0.0f)
					{
						vcopy(portalRight, right);
						rightPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
						rightPolyFlags = toFlags;
						rightIndex = i;
					}
					else
					{
						vcopy(portalApex, portalLeft);
						apexIndex = leftIndex;
						
						unsigned char flags = (leftPolyFlags & DT_POLY_OFFMESH_CONNECTION) ? DT_STRAIGHTPATH_OFFMESH_CONNECTION : 0;
						dtPolyRef ref = leftPolyRef;
						
						if (!vequal(&straightPath[(straightPathSize-1)*3], portalApex))
						{
							vcopy(&straightPath[straightPathSize*3], portalApex);
							if (straightPathFlags)
								straightPathFlags[straightPathSize] = flags;
							if (straightPathRefs)
								straightPathRefs[straightPathSize] = ref;
								
							straightPathSize++;
							if (straightPathSize >= maxStraightPathSize)
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
						
						vcopy(portalLeft, portalApex);
						vcopy(portalRight, portalApex);
						leftIndex = apexIndex;
						rightIndex = apexIndex;
						
						// Restart
						i = apexIndex;
						
						continue;
					}
				}
			}
			
			// Left vertex.
			if (vequal(portalApex, portalLeft))
			{
				vcopy(portalLeft, left);
				leftPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
				leftPolyFlags = toFlags;
				leftIndex = i;
			}
			else
			{
				if (triArea2D(portalApex, portalLeft, left) >= 0.0f)
				{
					if (triArea2D(portalApex, portalRight, left) < 0.0f)
					{
						vcopy(portalLeft, left);
						leftPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
						leftPolyFlags = toFlags;
						leftIndex = i;
					}
					else
					{
						vcopy(portalApex, portalRight);
						apexIndex = rightIndex;

						unsigned char flags = (rightPolyFlags & DT_POLY_OFFMESH_CONNECTION) ? DT_STRAIGHTPATH_OFFMESH_CONNECTION : 0;
						dtPolyRef ref = rightPolyRef;
						
						if (!vequal(&straightPath[(straightPathSize-1)*3], portalApex))
						{
							vcopy(&straightPath[straightPathSize*3], portalApex);
							if (straightPathFlags)
								straightPathFlags[straightPathSize] = flags;
							if (straightPathRefs)
								straightPathRefs[straightPathSize] = ref;
							
							straightPathSize++;
							if (straightPathSize >= maxStraightPathSize)
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
						
						vcopy(portalLeft, portalApex);
						vcopy(portalRight, portalApex);
						leftIndex = apexIndex;
						rightIndex = apexIndex;
						
						// Restart
						i = apexIndex;
						
						continue;
					}
				}
			}
		}
	}
	
	// Add end point.
	vcopy(&straightPath[straightPathSize*3], closestEndPos);
	if (straightPathFlags)
		straightPathFlags[straightPathSize] = DT_STRAIGHTPATH_END;
	if (straightPathRefs)
		straightPathRefs[straightPathSize] = 0;
	
	straightPathSize++;
	
	return straightPathSize;
}

// Moves towards end position a long the path corridor.
// Returns: Index to the result path polygon.
int dtNavMesh::moveAlongPathCorridor(const float* startPos, const float* endPos, float* resultPos,
									 const dtPolyRef* path, const int pathSize)
{
	if (!pathSize)
		return 0;
	
	float verts[DT_VERTS_PER_POLYGON*3];	
	float edged[DT_VERTS_PER_POLYGON];
	float edget[DT_VERTS_PER_POLYGON];
	int n = 0;
	
	static const float SLOP = 0.01f;

	vcopy(resultPos, startPos);
	
	while (n < pathSize)
	{
		// Get current polygon and poly vertices.
		unsigned int salt, it, ip;
		decodePolyId(path[n], salt, it, ip);
		if (it >= (unsigned int)m_maxTiles) return n;
		if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return n;
		if (ip >= (unsigned int)m_tiles[it].header->polyCount) return n;
		const dtMeshHeader* header = m_tiles[it].header;
		const dtPoly* poly = &header->polys[ip];
		
		// In case of Off-Mesh link, just snap to the end location and advance over it.
		if (poly->flags & DT_POLY_OFFMESH_CONNECTION)
		{
			if (n+1 < pathSize)
			{
				float left[3], right[3];
				unsigned short fromFlags, toFlags;
				if (!getPortalPoints(path[n], path[n+1], left, right, fromFlags, toFlags))
					return n;
				vcopy(resultPos, endPos);
			}
			return n+1;
		}
		
		// Collect vertices.
		int nv = 0;
		for (int i = 0; i < (int)poly->vertCount; ++i)
		{
			vcopy(&verts[nv*3], &header->verts[poly->verts[i]*3]);
			nv++;
		}

		bool inside = distancePtPolyEdgesSqr(endPos, verts, nv, edged, edget);
		if (inside)
		{
			// The end point is inside the current polygon.
			vcopy(resultPos, endPos);
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
		vlerp(resultPos, va, vb, edget[imin]);
		
		// Check to see if the point is on the portal edge to the next polygon.
		if (n+1 >= pathSize)
			return n;
		float left[3], right[3];
		unsigned short fromFlags, toFlags;
		if (!getPortalPoints(path[n], path[n+1], left, right, fromFlags, toFlags))
			return n;
		// If the clamped point is close to the next portal edge, advance to next poly.
		float t;
		float d = distancePtSegSqr2D(resultPos, left, right, t);
		if (d > SLOP*SLOP)
			return n;
		// Advance to next polygon.
		n++;
	}
	
	return n;
}

// Returns portal points between two polygons.
bool dtNavMesh::getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right,
								unsigned short& fromFlags, unsigned short& toFlags) const
{
	unsigned int salt, it, ip;
	decodePolyId(from, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return false;
	const dtMeshHeader* fromHeader = m_tiles[it].header;
	const dtPoly* fromPoly = &fromHeader->polys[ip];
	fromFlags = fromPoly->flags;

	for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromHeader->links[i].next)
	{
		const dtLink* link = &fromHeader->links[i];
		if (link->ref != to)
			continue;

		decodePolyId(to, salt, it, ip);
		if (it >= (unsigned int)m_maxTiles) return false;
		if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
		if (ip >= (unsigned int)m_tiles[it].header->polyCount) return false;
		const dtMeshHeader* toHeader = m_tiles[it].header;
		const dtPoly* toPoly = &toHeader->polys[ip];
		toFlags = toPoly->flags;
		
		if (fromPoly->flags & DT_POLY_OFFMESH_CONNECTION)
		{
			// Find link that points to first vertex.
			for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromHeader->links[i].next)
			{
				if (fromHeader->links[i].ref == to)
				{
					const int v = fromHeader->links[i].edge;
					vcopy(left, &fromHeader->verts[fromPoly->verts[v]*3]);
					vcopy(right, &fromHeader->verts[fromPoly->verts[v]*3]);
					return true;
				}
			}
			return false;
		}

		if (toPoly->flags & DT_POLY_OFFMESH_CONNECTION)
		{
			for (unsigned int i = toPoly->firstLink; i != DT_NULL_LINK; i = toHeader->links[i].next)
			{
				if (toHeader->links[i].ref == from)
				{
					const int v = toHeader->links[i].edge;
					vcopy(left, &toHeader->verts[toPoly->verts[v]*3]);
					vcopy(right, &toHeader->verts[toPoly->verts[v]*3]);
					return true;
				}
			}
			return false;
		}
		
		// Find portal vertices.
		const int v0 = fromPoly->verts[link->edge];
		const int v1 = fromPoly->verts[(link->edge+1) % (int)fromPoly->vertCount];
		vcopy(left, &fromHeader->verts[v0*3]);
		vcopy(right, &fromHeader->verts[v1*3]);
		// If the link is at tile boundary, clamp the vertices to
		// the link width.
		if (link->side == 0 || link->side == 4)
		{
			// Unpack portal limits.
			const float smin = min(left[2],right[2]);
			const float smax = max(left[2],right[2]);
			const float s = (smax-smin) / 255.0f;
			const float lmin = smin + link->bmin*s;
			const float lmax = smin + link->bmax*s;
			left[2] = max(left[2],lmin);
			left[2] = min(left[2],lmax);
			right[2] = max(right[2],lmin);
			right[2] = min(right[2],lmax);
		}
		else if (link->side == 2 || link->side == 6)
		{
			// Unpack portal limits.
			const float smin = min(left[0],right[0]);
			const float smax = max(left[0],right[0]);
			const float s = (smax-smin) / 255.0f;
			const float lmin = smin + link->bmin*s;
			const float lmax = smin + link->bmax*s;
			left[0] = max(left[0],lmin);
			left[0] = min(left[0],lmax);
			right[0] = max(right[0],lmin);
			right[0] = min(right[0],lmax);
		}
		return true;
	}
	return false;
}

// Returns edge mid point between two polygons.
bool dtNavMesh::getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const
{
	float left[3], right[3];
	unsigned short fromFlags, toFlags;
	if (!getPortalPoints(from, to, left,right, fromFlags, toFlags)) return false;
	mid[0] = (left[0]+right[0])*0.5f;
	mid[1] = (left[1]+right[1])*0.5f;
	mid[2] = (left[2]+right[2])*0.5f;
	return true;
}

unsigned short dtNavMesh::getPolyFlags(dtPolyRef ref)
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return 0;
	const dtMeshHeader* header = m_tiles[it].header;
	const dtPoly* poly = &header->polys[ip];
	return poly->flags;
}

int dtNavMesh::raycast(dtPolyRef centerRef, const float* startPos, const float* endPos, dtQueryFilter* filter,
					   float& t, float* hitNormal, dtPolyRef* path, const int pathSize)
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
		const dtMeshHeader* header = m_tiles[it].header;
		const dtPoly* poly = &header->polys[ip];

		// Collect vertices.
		int nv = 0;
		for (int i = 0; i < (int)poly->vertCount; ++i)
		{
			vcopy(&verts[nv*3], &header->verts[poly->verts[i]*3]);
			nv++;
		}		
		
		float tmin, tmax;
		int segMin, segMax;
		if (!intersectSegmentPoly2D(startPos, endPos, verts, nv, tmin, tmax, segMin, segMax))
		{
			// Could not hit the polygon, keep the old t and report hit.
			return n;
		}
		// Keep track of furthest t so far.
		if (tmax > t)
			t = tmax;

		if (n < pathSize)
			path[n++] = curRef;
		
		// Follow neighbours.
		dtPolyRef nextRef = 0;
		
		for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = header->links[i].next)
		{
			const dtLink* link = &header->links[i];
			if ((int)link->edge == segMax)
			{
				// If the link is internal, just return the ref.
				if (link->side == 0xff)
				{
					nextRef = link->ref;
					break;
				}
				
				// If the link is at tile boundary,
				const int v0 = poly->verts[link->edge];
				const int v1 = poly->verts[(link->edge+1) % poly->vertCount];
				const float* left = &header->verts[v0*3];
				const float* right = &header->verts[v1*3];
				
				// Check that the intersection lies inside the link portal.
				if (link->side == 0 || link->side == 4)
				{
					// Calculate link size.
					const float smin = min(left[2],right[2]);
					const float smax = max(left[2],right[2]);
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
					const float smin = min(left[0],right[0]);
					const float smax = max(left[0],right[0]);
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
		}
		
		if (!nextRef || !passFilter(filter, getPolyFlags(nextRef)))
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
			vnormalize(hitNormal);
			
			return n;
		}
		
		// No hit, advance to neighbour polygon.
		curRef = nextRef;
	}
	
	return n;
}

int dtNavMesh::findPolysAround(dtPolyRef centerRef, const float* centerPos, float radius, dtQueryFilter* filter,
									dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
									const int maxResult)
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
	
	const float radiusSqr = sqr(radius);
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();

		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		unsigned int it = decodePolyIdTile(bestNode->id);
		unsigned int ip = decodePolyIdPoly(bestNode->id);
		const dtMeshHeader* header = m_tiles[it].header;
		const dtPoly* poly = &header->polys[ip];
		
		for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = header->links[i].next)
		{
			const dtLink* link = &header->links[i];
			dtPolyRef neighbour = link->ref;
			if (neighbour)
			{
				// Skip parent node.
				if (bestNode->pidx && m_nodePool->getNodeAtIdx(bestNode->pidx)->id == neighbour)
					continue;
				
				// Calc distance to the edge.
				const float* va = &header->verts[poly->verts[link->edge]*3];
				const float* vb = &header->verts[poly->verts[(link->edge+1) % poly->vertCount]*3];
				float tseg;
				float distSqr = distancePtSegSqr2D(centerPos, va, vb, tseg);
				
				// If the circle is not touching the next polygon, skip it.
				if (distSqr > radiusSqr)
					continue;

				if (!passFilter(filter, getPolyFlags(neighbour)))
					continue;
				
				dtNode* parent = bestNode;
				dtNode newNode;
				newNode.pidx = m_nodePool->getNodeIdx(parent);
				newNode.id = neighbour;

				// Cost
				float p0[3], p1[3];
				if (!parent->pidx)
					vcopy(p0, centerPos);
				else
					getEdgeMidPoint(m_nodePool->getNodeAtIdx(parent->pidx)->id, parent->id, p0);
				getEdgeMidPoint(parent->id, newNode.id, p1);
				newNode.total = parent->total + vdist(p0,p1);
				
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
	}
	
	return n;
}

float dtNavMesh::findDistanceToWall(dtPolyRef centerRef, const float* centerPos, float maxRadius, dtQueryFilter* filter,
									float* hitPos, float* hitNormal)
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
	
	float radiusSqr = sqr(maxRadius);
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		unsigned int it = decodePolyIdTile(bestNode->id);
		unsigned int ip = decodePolyIdPoly(bestNode->id);
		const dtMeshHeader* header = m_tiles[it].header;
		const dtPoly* poly = &header->polys[ip];
		
		// Hit test walls.
		for (int i = 0, j = (int)poly->vertCount-1; i < (int)poly->vertCount; j = i++)
		{
			// Skip non-solid edges.
			if (poly->neis[j] & DT_EXT_LINK)
			{
				// Tile border.
				bool solid = true;
				for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = header->links[k].next)
				{
					const dtLink* link = &header->links[k];
					if (link->edge == j && link->ref != 0 && passFilter(filter, getPolyFlags(link->ref)))
					{
						solid = false;
						break;
					}
				}
				if (!solid) continue;
			}
			else if (poly->neis[j] && passFilter(filter, header->polys[poly->neis[j]].flags))
			{
				// Internal edge
				continue;
			}
			
			// Calc distance to the edge.
			const float* vj = &header->verts[poly->verts[j]*3];
			const float* vi = &header->verts[poly->verts[i]*3];
			float tseg;
			float distSqr = distancePtSegSqr2D(centerPos, vj, vi, tseg);
			
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
		
		for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = header->links[i].next)
		{
			const dtLink* link = &header->links[i];
			dtPolyRef neighbour = link->ref;
			if (neighbour)
			{
				// Skip parent node.
				if (bestNode->pidx && m_nodePool->getNodeAtIdx(bestNode->pidx)->id == neighbour)
					continue;
				
				// Calc distance to the edge.
				const float* va = &header->verts[poly->verts[link->edge]*3];
				const float* vb = &header->verts[poly->verts[(link->edge+1) % poly->vertCount]*3];
				float tseg;
				float distSqr = distancePtSegSqr2D(centerPos, va, vb, tseg);
				
				// If the circle is not touching the next polygon, skip it.
				if (distSqr > radiusSqr)
					continue;
				
				if (!passFilter(filter, getPolyFlags(neighbour)))
					continue;
				
				dtNode* parent = bestNode;
				dtNode newNode;
				newNode.pidx = m_nodePool->getNodeIdx(parent);
				newNode.id = neighbour;

				float p0[3], p1[3];
				if (!parent->pidx)
					vcopy(p0, centerPos);
				else
					getEdgeMidPoint(m_nodePool->getNodeAtIdx(parent->pidx)->id, parent->id, p0);
				getEdgeMidPoint(parent->id, newNode.id, p1);
				newNode.total = parent->total + vdist(p0,p1);
				
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
	}
	
	// Calc hit normal.
	vsub(hitNormal, centerPos, hitPos);
	vnormalize(hitNormal);
	
	return sqrtf(radiusSqr);
}

const dtPoly* dtNavMesh::getPolyByRef(dtPolyRef ref) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return 0;
	return &m_tiles[it].header->polys[ip];
}

const float* dtNavMesh::getPolyVertsByRef(dtPolyRef ref) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return 0;
	return m_tiles[it].header->verts;
}

const dtLink* dtNavMesh::getPolyLinksByRef(dtPolyRef ref) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return 0;
	return m_tiles[it].header->links;
}

bool dtNavMesh::isInClosedList(dtPolyRef ref) const
{
	if (!m_nodePool) return false;
	const dtNode* node = m_nodePool->findNode(ref);
	return node && node->flags & DT_NODE_CLOSED;
}

