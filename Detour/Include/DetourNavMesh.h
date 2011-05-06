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

#ifndef DETOURNAVMESH_H
#define DETOURNAVMESH_H

#include "DetourAlloc.h"
#include "DetourStatus.h"


// Note: If you want to use 64-bit refs, change the types of both dtPolyRef & dtTileRef.
// It is also recommended to change dtHashRef() to proper 64-bit hash too.

/// Reference to navigation polygon.
typedef unsigned int dtPolyRef;

/// Reference to navigation mesh tile.
typedef unsigned int dtTileRef;

/// Maximum number of vertices per navigation polygon.
static const int DT_VERTS_PER_POLYGON = 6;

static const int DT_NAVMESH_MAGIC = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V'; ///< 'DNAV'
static const int DT_NAVMESH_VERSION = 7;

static const int DT_NAVMESH_STATE_MAGIC = 'D'<<24 | 'N'<<16 | 'M'<<8 | 'S'; ///< 'DNMS'
static const int DT_NAVMESH_STATE_VERSION = 1;

static const unsigned short DT_EXT_LINK = 0x8000;
static const unsigned int DT_NULL_LINK = 0xffffffff;
static const unsigned int DT_OFFMESH_CON_BIDIR = 1;

static const int DT_MAX_AREAS = 64;

/// Flags for addTile
enum dtTileFlags
{
	DT_TILE_FREE_DATA = 0x01,					///< Navmesh owns the tile memory and should free it.
};

/// Flags returned by findStraightPath().
enum dtStraightPathFlags
{
	DT_STRAIGHTPATH_START = 0x01,				///< The vertex is the start position.
	DT_STRAIGHTPATH_END = 0x02,					///< The vertex is the end position.
	DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04,	///< The vertex is start of an off-mesh link.
};

/// Flags describing polygon properties.
enum dtPolyTypes
{
	DT_POLYTYPE_GROUND = 0,						///< Regular ground polygons.
	DT_POLYTYPE_OFFMESH_CONNECTION = 1,			///< Off-mesh connections.
};


/// Structure describing the navigation polygon data.
struct dtPoly
{
	unsigned int firstLink;						///< Index to first link in linked list.
	unsigned short verts[DT_VERTS_PER_POLYGON];	///< Indices to vertices of the poly.
	unsigned short neis[DT_VERTS_PER_POLYGON];	///< Refs to neighbours of the poly.
	unsigned short flags;						///< Flags (see dtPolyFlags).
	unsigned char vertCount;					///< Number of vertices.
	unsigned char areaAndtype;					///< Bit packed: Area ID of the polygon, and Polygon type, see dtPolyTypes..
	inline void setArea(unsigned char a) { areaAndtype = (areaAndtype & 0xc0) | (a & 0x3f); }
	inline void setType(unsigned char t) { areaAndtype = (areaAndtype & 0x3f) | (t << 6); }
	inline unsigned char getArea() const { return areaAndtype & 0x3f; }
	inline unsigned char getType() const { return areaAndtype >> 6; }
};

/// Stucture describing polygon detail triangles.
struct dtPolyDetail
{
	unsigned int vertBase;						///< Offset to detail vertex array.
	unsigned int triBase;						///< Offset to detail triangle array.
	unsigned char vertCount;					///< Number of vertices in the detail mesh.
	unsigned char triCount;						///< Number of triangles.
};

/// Stucture describing a link to another polygon.
struct dtLink
{
	dtPolyRef ref;							///< Neighbour reference.
	unsigned int next;						///< Index to next link.
	unsigned char edge;						///< Index to polygon edge which owns this link.
	unsigned char side;						///< If boundary link, defines on which side the link is.
	unsigned char bmin, bmax;				///< If boundary link, defines the sub edge area.
};

struct dtBVNode
{
	unsigned short bmin[3], bmax[3];		///< BVnode bounds
	int i;									///< Index to item or if negative, escape index.
};

struct dtOffMeshConnection
{
	float pos[6];							///< Both end point locations.
	float rad;								///< Link connection radius.
	unsigned short poly;					///< Poly Id
	unsigned char flags;					///< Link flags
	unsigned char side;						///< End point side.
	unsigned int userId;					///< User ID to identify this connection.
};

struct dtMeshHeader
{
	int magic;								///< Magic number, used to identify the data.
	int version;							///< Data version number.
	int x, y, layer;						///< Location of the tile on the grid.
	unsigned int userId;					///< User ID of the tile.
	int polyCount;							///< Number of polygons in the tile.
	int vertCount;							///< Number of vertices in the tile.
	int maxLinkCount;						///< Number of allocated links.
	int detailMeshCount;					///< Number of detail meshes.
	int detailVertCount;					///< Number of detail vertices.
	int detailTriCount;						///< Number of detail triangles.
	int bvNodeCount;						///< Number of BVtree nodes.
	int offMeshConCount;					///< Number of Off-Mesh links.
	int offMeshBase;						///< Index to first polygon which is Off-Mesh link.
	float walkableHeight;					///< Height of the agent.
	float walkableRadius;					///< Radius of the agent
	float walkableClimb;					///< Max climb height of the agent.
	float bmin[3], bmax[3];					///< Bounding box of the tile.
	float bvQuantFactor;					///< BVtree quantization factor (world to bvnode coords)
};

struct dtMeshTile
{
	unsigned int salt;						///< Counter describing modifications to the tile.

	unsigned int linksFreeList;				///< Index to next free link.
	dtMeshHeader* header;					///< Pointer to tile header.
	dtPoly* polys;							///< Pointer to the polygons (will be updated when tile is added).
	float* verts;							///< Pointer to the vertices (will be updated when tile added).
	dtLink* links;							///< Pointer to the links (will be updated when tile added).
	dtPolyDetail* detailMeshes;				///< Pointer to detail meshes (will be updated when tile added).
	float* detailVerts;						///< Pointer to detail vertices (will be updated when tile added).
	unsigned char* detailTris;				///< Pointer to detail triangles (will be updated when tile added).
	dtBVNode* bvTree;						///< Pointer to BVtree nodes (will be updated when tile added).
	dtOffMeshConnection* offMeshCons;		///< Pointer to Off-Mesh links. (will be updated when tile added).
		
	unsigned char* data;					///< Pointer to tile data.
	int dataSize;							///< Size of the tile data.
	int flags;								///< Tile flags, see dtTileFlags.
	dtMeshTile* next;						///< Next free tile or, next tile in spatial grid.
};

struct dtNavMeshParams
{
	float orig[3];					///< Origin of the nav mesh tile space.
	float tileWidth, tileHeight;	///< Width and height of each tile.
	int maxTiles;					///< Maximum number of tiles the navmesh can contain.
	int maxPolys;					///< Maximum number of polygons each tile can contain.
};


class dtNavMesh
{
public:
	dtNavMesh();
	~dtNavMesh();

	/// Initializes the nav mesh for tiled use.
	///  @param params [in] navmesh initialization params, see dtNavMeshParams.
	/// @return True if succeed, else false.
	dtStatus init(const dtNavMeshParams* params);

	/// Initializes the nav mesh for single tile use.
	///  @param data - [in] Data of the new tile mesh.
	///  @param dataSize - [in] Data size of the new tile mesh.
	///  @param flags - [in] Tile flags, see dtTileFlags.
	/// @return True if succeed, else false.
	dtStatus init(unsigned char* data, const int dataSize, const int flags);
	
	/// Returns pointer to navmesh initialization params.
	const dtNavMeshParams* getParams() const;
	
	/// Adds new tile into the navmesh.
	/// The add will fail if the data is in wrong format,
	/// there is not enough tiles left, or if there is a tile already at the location.
	///  @param data [in] Data of the new tile mesh.
	///  @param dataSize [in] Data size of the new tile mesh.
	///	@param flags [in] Tile flags, see dtTileFlags.
	///  @param lastRef [in,optional] Last tile ref, the tile will be restored so that
	///            the reference (as well as poly references) will be the same. Default: 0.
	///  @param result [out,optional] tile ref if the tile was succesfully added.
	dtStatus addTile(unsigned char* data, int dataSize, int flags, dtTileRef lastRef, dtTileRef* result);
	
	/// Removes specified tile.
	///  @param ref [in] Reference to the tile to remove.
	///  @param data [out] Data associated with deleted tile.
	///  @param dataSize [out] Size of the data associated with deleted tile.
	dtStatus removeTile(dtTileRef ref, unsigned char** data, int* dataSize);

	/// Calculates tile location based in input world position.
	///  @param pos [in] world position of the query.
	///  @param tx [out] tile x location.
	///  @param ty [out] tile y location.
	void calcTileLoc(const float* pos, int* tx, int* ty) const;

	/// Returns pointer to tile at specified location.
	///  @param x,y [in] Location of the tile to get.
	///  @returns pointer to tile if tile exists or 0 tile does not exists.
	const dtMeshTile* getTileAt(const int x, const int y, const int layer) const;

	int getTilesAt(const int x, const int y,
				   dtMeshTile const** tiles, const int maxTiles) const;
	
	/// Returns reference to tile at specified location.
	///  @param x,y [in] Location of the tile to get.
	///  @returns reference to tile if tile exists or 0 tile does not exists.
	dtTileRef getTileRefAt(int x, int y, int layer) const;

	/// Returns tile references of a tile based on tile pointer.
	dtTileRef getTileRef(const dtMeshTile* tile) const;

	/// Returns tile based on references.
	const dtMeshTile* getTileByRef(dtTileRef ref) const;
	
	/// Returns max number of tiles.
	int getMaxTiles() const;
	
	/// Returns pointer to tile in the tile array.
	///  @param i [in] Index to the tile to retrieve, max index is getMaxTiles()-1.
	///  @returns Pointer to specified tile.
	const dtMeshTile* getTile(int i) const;

	/// Returns pointer to tile and polygon pointed by the polygon reference.
	///  @param ref [in] reference to a polygon.
	///  @param tile [out] pointer to the tile containing the polygon.
	///  @param poly [out] pointer to the polygon.
	dtStatus getTileAndPolyByRef(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;
	
	/// Returns pointer to tile and polygon pointed by the polygon reference.
	/// Note: this function does not check if 'ref' s valid, and is thus faster. Use only with valid refs!
	///
	///  @param ref [in] reference to a polygon.
	///  @param tile [out] pointer to the tile containing the polygon.
	///  @param poly [out] pointer to the polygon.
	void getTileAndPolyByRefUnsafe(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;

	/// Returns true if polygon reference points to valid data.
	bool isValidPolyRef(dtPolyRef ref) const;
	
	/// Returns base poly id for specified tile, polygon refs can be deducted from this.
	dtPolyRef getPolyRefBase(const dtMeshTile* tile) const;
	
	/// Returns start and end location of an off-mesh link polygon.
	///  @param prevRef [in] ref to the polygon before the link (used to select direction).
	///  @param polyRef [in] ref to the off-mesh link polygon.
	///  @param startPos[3] [out] start point of the link.
	///  @param endPos[3] [out] end point of the link.
	///  @returns true if link is found.
	dtStatus getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const;

	/// Returns pointer to off-mesh connection based on polyref, or null if ref not valid.
	const dtOffMeshConnection* getOffMeshConnectionByRef(dtPolyRef ref) const;
	
	/// Sets polygon flags.
	dtStatus setPolyFlags(dtPolyRef ref, unsigned short flags);

	/// Return polygon flags.
	dtStatus getPolyFlags(dtPolyRef ref, unsigned short* resultFlags) const;

	/// Set polygon type.
	dtStatus setPolyArea(dtPolyRef ref, unsigned char area);

	/// Return polygon area type.
	dtStatus getPolyArea(dtPolyRef ref, unsigned char* resultArea) const;


	/// Returns number of bytes required to store tile state.
	int getTileStateSize(const dtMeshTile* tile) const;
	
	/// Stores tile state to buffer.
	dtStatus storeTileState(const dtMeshTile* tile, unsigned char* data, const int maxDataSize) const;
	
	/// Restores tile state.
	dtStatus restoreTileState(dtMeshTile* tile, const unsigned char* data, const int maxDataSize);
	

	/// Encodes a tile id.
	inline dtPolyRef encodePolyId(unsigned int salt, unsigned int it, unsigned int ip) const
	{
		return ((dtPolyRef)salt << (m_polyBits+m_tileBits)) | ((dtPolyRef)it << m_polyBits) | (dtPolyRef)ip;
	}
	
	/// Decodes a tile id.
	inline void decodePolyId(dtPolyRef ref, unsigned int& salt, unsigned int& it, unsigned int& ip) const
	{
		const dtPolyRef saltMask = ((dtPolyRef)1<<m_saltBits)-1;
		const dtPolyRef tileMask = ((dtPolyRef)1<<m_tileBits)-1;
		const dtPolyRef polyMask = ((dtPolyRef)1<<m_polyBits)-1;
		salt = (unsigned int)((ref >> (m_polyBits+m_tileBits)) & saltMask);
		it = (unsigned int)((ref >> m_polyBits) & tileMask);
		ip = (unsigned int)(ref & polyMask);
	}

	/// Decodes a tile salt.
	inline unsigned int decodePolyIdSalt(dtPolyRef ref) const
	{
		const dtPolyRef saltMask = ((dtPolyRef)1<<m_saltBits)-1;
		return (unsigned int)((ref >> (m_polyBits+m_tileBits)) & saltMask);
	}
	
	/// Decodes a tile id.
	inline unsigned int decodePolyIdTile(dtPolyRef ref) const
	{
		const dtPolyRef tileMask = ((dtPolyRef)1<<m_tileBits)-1;
		return (unsigned int)((ref >> m_polyBits) & tileMask);
	}
	
	/// Decodes a poly id.
	inline unsigned int decodePolyIdPoly(dtPolyRef ref) const
	{
		const dtPolyRef polyMask = ((dtPolyRef)1<<m_polyBits)-1;
		return (unsigned int)(ref & polyMask);
	}
	
private:

	/// Returns pointer to tile in the tile array.
	dtMeshTile* getTile(int i);

	/// Returns neighbour tile based on side.
	int getTilesAt(const int x, const int y,
				   dtMeshTile** tiles, const int maxTiles) const;

	/// Returns neighbour tile based on side.
	int getNeighbourTilesAt(const int x, const int y, const int side,
							dtMeshTile** tiles, const int maxTiles) const;
	
	/// Returns all polygons in neighbour tile based on portal defined by the segment.
	int findConnectingPolys(const float* va, const float* vb,
							const dtMeshTile* tile, int side,
							dtPolyRef* con, float* conarea, int maxcon) const;
	
	/// Builds internal polygons links for a tile.
	void connectIntLinks(dtMeshTile* tile);
	/// Builds internal polygons links for a tile.
	void connectIntOffMeshLinks(dtMeshTile* tile);

	/// Builds external polygon links for a tile.
	void connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side);
	/// Builds external polygon links for a tile.
	void connectExtOffMeshLinks(dtMeshTile* tile, dtMeshTile* target, int side);
	
	/// Removes external links at specified side.
	void unconnectExtLinks(dtMeshTile* tile, dtMeshTile* target);
	

	// TODO: These methods are duplicates from dtNavMeshQuery, but are needed for off-mesh connection finding.
	
	/// Queries polygons within a tile.
	int queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
							dtPolyRef* polys, const int maxPolys) const;
	/// Find nearest polygon within a tile.
	dtPolyRef findNearestPolyInTile(const dtMeshTile* tile, const float* center,
									const float* extents, float* nearestPt) const;
	/// Returns closest point on polygon.
	void closestPointOnPolyInTile(const dtMeshTile* tile, unsigned int ip,
								  const float* pos, float* closest) const;
	
	dtNavMeshParams m_params;			///< Current initialization params. TODO: do not store this info twice.
	float m_orig[3];					///< Origin of the tile (0,0)
	float m_tileWidth, m_tileHeight;	///< Dimensions of each tile.
	int m_maxTiles;						///< Max number of tiles.
	int m_tileLutSize;					///< Tile hash lookup size (must be pot).
	int m_tileLutMask;					///< Tile hash lookup mask.

	dtMeshTile** m_posLookup;			///< Tile hash lookup.
	dtMeshTile* m_nextFree;				///< Freelist of tiles.
	dtMeshTile* m_tiles;				///< List of tiles.
		
	unsigned int m_saltBits;			///< Number of salt bits in the tile ID.
	unsigned int m_tileBits;			///< Number of tile bits in the tile ID.
	unsigned int m_polyBits;			///< Number of poly bits in the tile ID.
};

/// Helper function to allocate navmesh class using Detour allocator.
dtNavMesh* dtAllocNavMesh();
void dtFreeNavMesh(dtNavMesh* navmesh);

#endif // DETOURNAVMESH_H
