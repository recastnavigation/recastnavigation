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

#ifndef DETOURSTATNAVMESH_H
#define DETOURSTATNAVMESH_H

// Reference to navigation polygon.
typedef unsigned short dtPolyRef;

// Maximum number of vertices per navigation polygon.
static const int DT_VERTS_PER_POLYGON = 6;

// Structure holding the navigation polygon data.
struct dtPoly
{
	unsigned short v[DT_VERTS_PER_POLYGON];	// Indices to vertices of the poly.
	dtPolyRef n[DT_VERTS_PER_POLYGON];		// Refs to neighbours of the poly.
	unsigned char nv;							// Number of vertices.
	unsigned char flags;						// Flags (not used).
	unsigned char pad[2];
};

const int DT_NAVMESH_MAGIC = 'NAVM';
const int DT_NAVMESH_VERSION = 2;

struct dtBVNode
{
	unsigned short bmin[3], bmax[3];
	int i;
};

struct dtStatNavMeshHeader
{
	int magic;
	int version;
	int npolys;
	int nverts;
	int nnodes;
	float cs;
	float bmin[3], bmax[3];
};

class dtStatNavMesh
{
public:
	
	dtStatNavMesh();
	~dtStatNavMesh();

	// Initializes the path finder with path data.
	// Params:
	//	data - (in) Pointer to path data.
	//	dataSize - (in) size of the path data.
	//	ownsData - (in) Flag indicating if the pathfinder should delete the data.
	bool init(unsigned char* data, int dataSize, bool ownsData);

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
					  unsigned short* polys, const int maxPolys);
	
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
	//	t - (out) hit parameter along the segment, valid only if hit.
	//	endRef - (out) ref to the last polygon which was processed.
	// Returns: True if hit wall.
	// TODO: Return the whole corridor!!
	bool raycast(dtPolyRef startRef, const float* startPos, const float* endPos, float& t, dtPolyRef& endRef);
	
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
	//	resultDepth - (out, opt) search depth at each result polygon.
	//	maxResult - (int) maximum capacity of search results.
	// Returns: Number of results.
	int	findPolysAround(dtPolyRef centerRef, const float* centerPos, float radius,
						dtPolyRef* resultRef, dtPolyRef* resultParent,
						unsigned short* resultCost, unsigned short* resultDepth,
						const int maxResult);

	// Returns closest point on navigation polygon.
	// Params:
	//	ref - (in) ref to the polygon.
	//	pos - (in) the point to check.
	//	closest - (out) closest point.
	// Returns: true if closest point found.
	bool closestPointToPoly(dtPolyRef ref, const float* pos, float* closest) const;

	// Returns cost between two polygons.
	unsigned short getCost(dtPolyRef from, dtPolyRef to) const;

	// Returns pointer to a polygon based on ref.
	const dtPoly* getPolyByRef(dtPolyRef ref) const;
	// Returns number of navigation polygons.
	inline int getPolyCount() const { return m_header ? m_header->npolys : 0; }
	// Rerturns pointer to specified navigation polygon.
	inline const dtPoly* getPoly(int i) const { return &m_polys[i]; }
	// Returns number of vertices.
	inline int getVertexCount() const { return m_header ? m_header->nverts : 0; }
	// Returns pointer to specified vertex.
	inline const float* getVertex(int i) const { return &m_verts[i*3]; }

	bool isInOpenList(dtPolyRef ref) const;
	
	int getMemUsed() const;

	inline const dtStatNavMeshHeader* getHeader() const { return m_header; }
	
	inline const dtBVNode* getBvTreeNodes() const { return m_bvtree; }
	inline int getBvTreeNodeCount() const { return m_header->nnodes; }
	
private:

	// Copies the locations of vertices of a polygon to an array.
	int getPolyVerts(dtPolyRef ref, float* verts);
	// Returns portal points between two polygons.
	bool getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right);

	unsigned char* m_data;
	int m_dataSize;
	
	dtStatNavMeshHeader* m_header;
	dtPoly* m_polys;
	float* m_verts;
	dtBVNode* m_bvtree;

	class dtNodePool* m_nodePool;
	class dtNodeQueue* m_openList;
};

#endif // DETOURSTATNAVMESH_H
