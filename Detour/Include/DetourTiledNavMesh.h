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

#ifndef DETOURTILEDNAVMESH_H
#define DETOURTILEDNAVMESH_H

// Reference to navigation polygon.
typedef unsigned int dtTilePolyRef;

static const int DT_TILE_REF_SALT_BITS = 12;
static const int DT_TILE_REF_SALT_MASK = (1<<DT_TILE_REF_SALT_BITS)-1;
static const int DT_TILE_REF_TILE_BITS = 12;
static const int DT_TILE_REF_TILE_MASK = (1<<DT_TILE_REF_TILE_BITS)-1;
static const int DT_TILE_REF_POLY_BITS = 8;
static const int DT_TILE_REF_POLY_MASK = (1<<DT_TILE_REF_POLY_BITS)-1;

// Maximum number of vertices per navigation polygon.
static const int DT_TILE_VERTS_PER_POLYGON = 6;

static const int DT_MAX_TILES = 1 << DT_TILE_REF_TILE_BITS;
static const int DT_MAX_POLYGONS = 1 << DT_TILE_REF_POLY_BITS;

// Structure holding the navigation polygon data.
struct dtTilePoly
{
	unsigned short v[DT_TILE_VERTS_PER_POLYGON];	// Indices to vertices of the poly.
	unsigned short n[DT_TILE_VERTS_PER_POLYGON];	// Refs to neighbours of the poly.
	unsigned short links;
	unsigned char nlinks;
	unsigned char nv;								// Number of vertices.
	unsigned char flags;							// Flags (not used).
};

struct dtTileLink
{
	dtTilePolyRef ref;
	unsigned short p;
	unsigned char e;
	unsigned char side;
	float bmin, bmax;
};

static const int DT_TILE_NAVMESH_MAGIC = 'NAVT';
static const int DT_TILE_NAVMESH_VERSION = 1;

struct dtTileHeader
{
	int magic;
	int version;
	int npolys;
	int nverts;
	int nportals[4];
	int nlinks;
	int maxlinks;
	float cs;
	float bmin[3], bmax[3];
	dtTilePoly* polys;
	float* verts;
	dtTileLink* links;
	struct dtTile* nei[4];
};

struct dtTile
{
	int salt;
	int x,y;
	dtTileHeader* header;
	dtTile* next;
};

inline dtTilePolyRef encodeId(int salt, int it, int ip)
{
	return ((unsigned int)salt << (DT_TILE_REF_POLY_BITS+DT_TILE_REF_TILE_BITS)) | ((unsigned int)(it+1) << DT_TILE_REF_POLY_BITS) | (unsigned int)ip;
}

inline void decodeId(dtTilePolyRef ref, int& salt, int& it, int& ip)
{
	salt = (int)((ref >> (DT_TILE_REF_POLY_BITS+DT_TILE_REF_TILE_BITS)) & DT_TILE_REF_SALT_MASK);
	it = (int)(((ref >> DT_TILE_REF_POLY_BITS) & DT_TILE_REF_TILE_MASK) - 1);
	ip = (int)(ref & DT_TILE_REF_POLY_MASK);
}

static const int DT_TILE_LOOKUP_SIZE = DT_MAX_TILES/4;

class dtTiledNavMesh
{
public:
	dtTiledNavMesh();
	~dtTiledNavMesh();
	
	bool init(const float* orig, float tileSize, float portalHeight);
	bool addTile(int x, int y, unsigned char* data, int dataSize);
	bool removeTile(int x, int y);
	dtTile* getTile(int x, int y);
	const dtTile* getTile(int i) const { return &m_tiles[i]; }

	// Finds the nearest navigation polygon around the center location.
	// Params:
	//	center - (in) The center of the search box.
	//	extents - (in) The extents of the search box.
	// Returns: Reference identifier for the polygon, or 0 if no polygons found.
	dtTilePolyRef findNearestPoly(const float* center, const float* extents);
	
	// Returns polygons which touch the query box.
	// Params:
	//	center - (in) the center of the search box.
	//	extents - (in) the extents of the search box.
	//	polys - (out) array holding the search result.
	//	maxPolys - (in) The max number of polygons the polys array can hold.
	// Returns: Number of polygons in search result array.
	int queryPolygons(const float* center, const float* extents,
					  dtTilePolyRef* polys, const int maxPolys);
	
	bool closestPointToPoly(dtTilePolyRef ref, const float* pos, float* closest) const;

	int getPolyNeighbours(dtTilePolyRef ref, dtTilePolyRef* nei, int maxNei) const;

	int findPath(dtTilePolyRef startRef, dtTilePolyRef endRef,
				 dtTilePolyRef* path, const int maxPathSize);

	int findStraightPath(const float* startPos, const float* endPos,
						 const dtTilePolyRef* path, const int pathSize,
						 float* straightPath, const int maxStraightPathSize);
	
	int raycast(dtTilePolyRef startRef, const float* startPos, const float* endPos,
				float& t, dtTilePolyRef* path, const int pathSize);

	int	findPolysAround(dtTilePolyRef centerRef, const float* centerPos, float radius,
						dtTilePolyRef* resultRef, dtTilePolyRef* resultParent,
						float* resultCost, unsigned short* resultDepth,
						const int maxResult);
	
	float findDistanceToWall(dtTilePolyRef centerRef, const float* centerPos, float maxRadius,
							 float* hitPos, float* hitNormal);
	
	inline const dtTilePoly* getPolyByRef(dtTilePolyRef ref) const
	{
		int salt, it, ip;
		decodeId(ref, salt, it, ip);
		if (it >= DT_MAX_TILES) return 0;
		if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
		if (ip >= m_tiles[it].header->npolys) return 0;
		return &m_tiles[it].header->polys[ip];
	}

	inline const float* getPolyVertsByRef(dtTilePolyRef ref) const
	{
		int salt, it, ip;
		decodeId(ref, salt, it, ip);
		if (it >= DT_MAX_TILES) return 0;
		if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
		if (ip >= m_tiles[it].header->npolys) return 0;
		return m_tiles[it].header->verts;
	}
	
private:

	dtTilePolyRef getTileId(dtTile* tile);
	dtTile* getNeighbourTile(int x, int y, int side);
	
	void buildIntLinks(dtTile* tile);
	void buildExtLinks(dtTile* tile, dtTile* target, int side);
	void removeExtLinks(dtTile* tile, int side);
	
	void createConnections(dtTile* tilea, dtTile* tileb, int sidea); 
	void removeConnections(dtTile* tile, int side);
	int queryTilePolygons(dtTile* tile, const float* qmin, const float* qmax,
						  dtTilePolyRef* polys, const int maxPolys);
	int findConnectingPolys(const float* va, const float* vb,
							dtTile* tile, int side,
							dtTilePolyRef* con, float* conarea, int maxcon);

	float getCost(dtTilePolyRef prev, dtTilePolyRef from, dtTilePolyRef to) const;
	float getHeuristic(dtTilePolyRef from, dtTilePolyRef to) const;

	// Returns portal points between two polygons.
	bool getPortalPoints(dtTilePolyRef from, dtTilePolyRef to, float* left, float* right) const;

	float m_orig[3];
	float m_tileSize;
	float m_portalHeight;

	dtTile* m_posLookup[DT_TILE_LOOKUP_SIZE];
	dtTile* m_nextFree;
	dtTile m_tiles[DT_MAX_TILES];
	
	dtTileLink* m_tmpLinks;
	int m_ntmpLinks;

	class dtTileNodePool* m_nodePool;
	class dtTileNodeQueue* m_openList;
};

#endif // DETOURTILEDNAVMESH_H
