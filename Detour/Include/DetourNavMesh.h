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

#ifndef DETOURNAVMESH_H
#define DETOURNAVMESH_H

// Reference to navigation polygon.
typedef unsigned int dtPolyRef;

// The bits used in the poly ref.
/*static const int DT_REF_SALT_BITS = 12;
static const int DT_REF_TILE_BITS = 12;
static const int DT_REF_POLY_BITS = 8;
static const int DT_REF_SALT_MASK = (1<<DT_REF_SALT_BITS)-1;
static const int DT_REF_TILE_MASK = (1<<DT_REF_TILE_BITS)-1;
static const int DT_REF_POLY_MASK = (1<<DT_REF_POLY_BITS)-1;*/

// Maximum number of vertices per navigation polygon.
static const int DT_VERTS_PER_POLYGON = 6;

/*
static const int DT_MAX_TILES = 1 << DT_REF_TILE_BITS;
static const int DT_MAX_POLYGONS = 1 << DT_REF_POLY_BITS;
*/

static const int DT_NAVMESH_MAGIC = 'DNAV';
static const int DT_NAVMESH_VERSION = 1;

// Structure holding the navigation polygon data.
struct dtPoly
{
	unsigned short v[DT_VERTS_PER_POLYGON];	// Indices to vertices of the poly.
	unsigned short n[DT_VERTS_PER_POLYGON];	// Refs to neighbours of the poly.
	unsigned short links;							// Base index to header 'links' array. 
	unsigned char nlinks;							// Number of links for 
	unsigned char nv;								// Number of vertices.
	unsigned char flags;							// Flags (not used).
};

struct dtPolyDetail
{
	unsigned short vbase;	// Offset to detail vertex array.
	unsigned short nverts;	// Number of vertices in the detail mesh.
	unsigned short tbase;	// Offset to detail triangle array.
	unsigned short ntris;	// Number of triangles.
};

// Stucture holding a link to another polygon.
struct dtLink
{
	dtPolyRef ref;				// Neighbour reference.
	unsigned short p;			// Index to polygon which owns this link.
	unsigned char e;			// Index to polygon edge which owns this link. 
	unsigned char side;			// If boundary link, defines on which side the link is.
	unsigned char bmin, bmax;	// If boundary link, defines the sub edge area.
};

struct dtBVNode
{
	unsigned short bmin[3], bmax[3];
	int i;
};

struct dtMeshHeader
{
	int magic;					// Magic number, used to identify the data.
	int version;				// Data version number.
	int npolys;					// Number of polygons in the tile.
	int nverts;					// Number of vertices in the tile.
	int nlinks;					// Number of links in the tile (will be updated when tile is added).
	int maxlinks;				// Number of allocated links.
	int ndmeshes;
	int ndverts;
	int ndtris;
	int nbvtree;
	float bmin[3], bmax[3];		// Bounding box of the tile.
	float bvquant;
	dtPoly* polys;				// Pointer to the polygons (will be updated when tile is added).
	float* verts;				// Pointer to the vertices (will be updated when tile added).
	dtLink* links;				// Pointer to the links (will be updated when tile added).
	dtPolyDetail* dmeshes;
	float* dverts;
	unsigned char* dtris;
	dtBVNode* bvtree;
};

struct dtMeshTile
{
	int salt;				// Counter describing modifications to the tile.
	int x,y;				// Grid location of the tile.
	dtMeshHeader* header;	// Pointer to tile header.
	unsigned char* data;	// Pointer to tile data.
	int dataSize;			// Size of the tile data.
	bool ownsData;			// Flag indicating of the navmesh should release the data.
	dtMeshTile* next;		// Next free tile or, next tile in spatial grid.
};

class dtNavMesh
{
public:
	dtNavMesh();
	~dtNavMesh();

	// Initializes the nav mesh.
	// Params:
	//  orig - (in) origin of the nav mesh tile space.
	//  tileSiz - (in) size of a tile.
	//  portalheight - (in) height of the portal region between tiles.
	// Returns: True if succeed, else false.
	bool init(const float* orig, float tileSize, float portalHeight,
			  int maxTiles, int maxPolys, int maxNodes);

	// Adds new tile into the navmesh.
	// The add will fail if the data is in wrong format,
	// there is not enough tiles left, or if there is a tile already at the location.
	// Params:
	//  x,y - (in) Location of the new tile.
	//  data - (in) Data of the new tile mesh.
	//  dataSize - (in) Data size of the new tile mesh.
	//	ownsData - (in) Flag indicating if the navmesh should own and delete the data.
	// Returns: True if tile was added, else false. 
	bool addTileAt(int x, int y, unsigned char* data, int dataSize, bool ownsData);
	
	// Removes tile at specified location.
	// Params:
	//  x,y - (in) Location of the tile to remove.
	//  data - (out) Data associated with deleted tile.
	//  dataSize - (out) Size of the data associated with deleted tile. 
	// Returns: True if remove suceed, else false.
	bool removeTileAt(int x, int y, unsigned char** data, int* dataSize);

	// Returns pointer to tile at specified location.
	// Params:
	//  x,y - (in) Location of the tile to get.
	// Returns: pointer to tile if tile exists or 0 tile does not exists.
	dtMeshTile* getTileAt(int x, int y);

	// Returns max number of tiles.
	int getMaxTiles() const;
		
	// Returns pointer to tile in the tile array.
	// Params:
	//  i - (in) Index to the tile to retrieve, max index is getMaxTiles()-1.
	// Returns: Pointer to specified tile.
	dtMeshTile* getTile(int i);
	const dtMeshTile* getTile(int i) const;
	
	// Returns pointer to tile in the tile array.
	// Params:
	//  ref - (in) reference to a polygon inside the tile.
	//  plyIndex - (out,optional) pointer to value where polygon index within the tile is stored.
	// Returns: Pointer to specified tile.
	const dtMeshTile* getTileByRef(dtPolyRef ref, int* polyIndex) const;
	
	// Finds the nearest navigation polygon around the center location.
	// Params:
	//	center - (in) The center of the search box.
	//	extents - (in) The extents of the search box.
	// Returns: Reference identifier for the polygon, or 0 if no polygons found.
	dtPolyRef findNearestPoly(const float* center, const float* extents);
	
	// Returns polygons which touch the query box.
	// Params:
	//	center - (in) the center of the search box.
	//	extents - (in) the extents of the search box.
	//	polys - (out) array holding the search result.
	//	maxPolys - (in) The max number of polygons the polys array can hold.
	// Returns: Number of polygons in search result array.
	int queryPolygons(const float* center, const float* extents,
					  dtPolyRef* polys, const int maxPolys);
	
	// Finds path from start polygon to end polygon.
	// If target polygon canno be reached through the navigation graph,
	// the last node on the array is nearest node to the end polygon.
	// Params:
	//	startRef - (in) ref to path start polygon.
	//	endRef - (in) ref to path end polygon.
	//	path - (out) array holding the search result.
	//	maxPathSize - (in) The max number of polygons the path array can hold.
	// Returns: Number of polygons in search result array.
	int findPath(dtPolyRef startRef, dtPolyRef endRef,
				 const float* startPos, const float* endPos,
				 dtPolyRef* path, const int maxPathSize);

	// Finds a straight path from start to end locations within the corridor
	// described by the path polygons.
	// Start and end locations will be clamped on the corridor.
	// Params:
	//	startPos - (in) Path start location.
	//	endPos - (in) Path end location.
	//	path - (in) Array of connected polygons describing the corridor.
	//	pathSize - (in) Number of polygons in path array.
	//	straightPath - (out) Points describing the straight path.
	//	maxStraightPathSize - (in) The max number of points the straight path array can hold.
	// Returns: Number of points in the path.
	int findStraightPath(const float* startPos, const float* endPos,
						 const dtPolyRef* path, const int pathSize,
						 float* straightPath, const int maxStraightPathSize);

	// Finds intersection againts walls starting from start pos.
	// Params:
	//	startRef - (in) ref to the polygon where the start lies.
	//	startPos - (in) start position of the query.
	//	endPos - (in) end position of the query.
	//	t - (out) hit parameter along the segment, 0 if no hit.
	//	endRef - (out) ref to the last polygon which was processed.
	// Returns: Number of polygons in path or 0 if failed.
	int raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
				float& t, dtPolyRef* path, const int pathSize);

	// Returns distance to nearest wall from the specified location.
	// Params:
	//	centerRef - (in) ref to the polygon where the center lies.
	//	centerPos - (in) center if the query circle.
	//	maxRadius - (in) max search radius.
	//	hitPos - (out) location of the nearest hit.
	//	hitNormal - (out) normal of the nearest hit.
	// Returns: Distance to nearest wall from the test location.
	float findDistanceToWall(dtPolyRef centerRef, const float* centerPos, float maxRadius,
							 float* hitPos, float* hitNormal);

	// Finds polygons found along the navigation graph which touch the specified circle.
	// Params:
	//	centerRef - (in) ref to the polygon where the center lies.
	//	centerPos - (in) center if the query circle
	//	radius - (in) radius of the query circle
	//	resultRef - (out, opt) refs to the polygons touched by the circle.
	//	resultParent - (out, opt) parent of each result polygon.
	//	resultCost - (out, opt) search cost at each result polygon.
	//	maxResult - (int) maximum capacity of search results.
	// Returns: Number of results.
	int	findPolysAround(dtPolyRef centerRef, const float* centerPos, float radius,
						dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
						const int maxResult);
	
	// Returns closest point on navigation polygon.
	// Params:
	//	ref - (in) ref to the polygon.
	//	pos - (in) the point to check.
	//	closest - (out) closest point.
	// Returns: true if closest point found.
	bool closestPointToPoly(dtPolyRef ref, const float* pos, float* closest) const;

	// Returns height of the polygon at specified location.
	// Params:
	//	ref - (in) ref to the polygon.
	//	pos - (in) the point where to locate the height.
	//	height - (out) height at the location.
	// Returns: true if over polygon.
	bool getPolyHeight(dtPolyRef ref, const float* pos, float* height) const;
	
	// Returns pointer to a polygon based on ref.
	const dtPoly* getPolyByRef(dtPolyRef ref) const;

	// Returns pointer to a polygon vertices based on ref.
	const float* getPolyVertsByRef(dtPolyRef ref) const;

	// Returns pointer to a polygon link based on ref.
	const dtLink* getPolyLinksByRef(dtPolyRef ref) const;
	
private:

	// Encodes a tile id.
	inline dtPolyRef dtEncodePolyId(unsigned int salt, unsigned int it, unsigned int ip) const
	{
		return (salt << (m_polyBits+m_tileBits)) | ((it+1) << m_polyBits) | ip;
	}
	
	// Decodes a tile id.
	inline void dtDecodePolyId(dtPolyRef ref, unsigned int& salt, unsigned int& it, unsigned int& ip) const
	{
		salt = (ref >> (m_polyBits+m_tileBits)) & ((1<<m_saltBits)-1);
		it = ((ref >> m_polyBits) - 1) & ((1<<m_tileBits)-1);
		ip = ref & ((1<<m_polyBits)-1);
	}
	
	// Returns base id for the tile.
	dtPolyRef getTileId(dtMeshTile* tile);
	// Returns neighbour tile based on side. 
	dtMeshTile* getNeighbourTileAt(int x, int y, int side);
	// Returns all polygons in neighbour tile based on portal defined by the segment.
	int findConnectingPolys(const float* va, const float* vb,
							dtMeshTile* tile, int side,
							dtPolyRef* con, float* conarea, int maxcon);
	// Builds internal polygons links for a tile.
	void buildIntLinks(dtMeshTile* tile);
	// Builds external polygon links for a tile.
	void buildExtLinks(dtMeshTile* tile, dtMeshTile* target, int side);
	// Removes external links at specified side.
	void removeExtLinks(dtMeshTile* tile, int side);
	// Queries polygons within a tile.
	int queryTilePolygons(dtMeshTile* tile, const float* qmin, const float* qmax,
						  dtPolyRef* polys, const int maxPolys);
						  
	float getCost(dtPolyRef prev, dtPolyRef from, dtPolyRef to) const;
	float getFirstCost(const float* pos, dtPolyRef from, dtPolyRef to) const;
	float getLastCost(dtPolyRef from, dtPolyRef to, const float* pos) const;
	float getHeuristic(const float* from, const float* to) const;
	
	// Returns portal points between two polygons.
	bool getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right) const;
	// Returns edge mid point between two polygons.
	bool getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const;

	float m_orig[3];
	float m_tileSize;
	float m_portalHeight;
	int m_maxTiles;
	int m_tileLutSize;
	int m_tileLutMask;

	dtMeshTile** m_posLookup; //[DT_TILE_LOOKUP_SIZE];
	dtMeshTile* m_nextFree;
	dtMeshTile* m_tiles; //[DT_MAX_TILES];
	
	// TODO: dont grow!
	dtLink* m_tmpLinks;
	int m_ntmpLinks;
	
	unsigned int m_saltBits;
	unsigned int m_tileBits;
	unsigned int m_polyBits;

	class dtNodePool* m_nodePool;
	class dtNodeQueue* m_openList;
};

#endif // DETOURNAVMESH_H
