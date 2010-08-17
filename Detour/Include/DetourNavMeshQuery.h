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

#ifndef DETOURNAVMESHQUERY_H
#define DETOURNAVMESHQUERY_H

#include "DetourAlloc.h"
#include "DetourNavMesh.h"

struct dtQueryFilter
{
	dtQueryFilter() : includeFlags(0xffff), excludeFlags(0) {}
	unsigned short includeFlags;				// If any of the flags are set, the poly is included.
	unsigned short excludeFlags;				// If any of the flags are set, the poly is excluded.
};

enum dtQueryState
{
	DT_QUERY_FAILED = 0,						// Path find failed.
	DT_QUERY_RUNNING,							// Path find running.
	DT_QUERY_READY,								// Path find results ready.
};

class dtNavMeshQuery
{
public:
	dtNavMeshQuery();
	~dtNavMeshQuery();
	
	// Initializes the nav mesh query.
	// Params:
	//  nav - (in) pointer to navigation mesh data.
	//  maxNodes - (in) Maximum number of search nodes to use (max 65536).
	// Returns: True if succeed, else false.
	bool init(dtNavMesh* nav, const int maxNodes);
	
	// Sets the pathfinding cost of the specified area.
	// Params:
	//  area - (in) area ID (0-63).
	//  cost - (int) travel cost of the area.
	void setAreaCost(const int area, float cost);
	
	// Returns the pathfinding cost of the specified area.
	// Params:
	//  area - (in) area ID (0-63).
	float getAreaCost(const int area) const;
	
	// Finds the nearest navigation polygon around the center location.
	// Params:
	//	center[3] - (in) The center of the search box.
	//	extents[3] - (in) The extents of the search box.
	//  filter - (in) path polygon filter.
	//  nearestPt[3] - (out, opt) The nearest point on found polygon, null if not needed.
	// Returns: Reference identifier for the polygon, or 0 if no polygons found.
	dtPolyRef findNearestPoly(const float* center, const float* extents,
							  const dtQueryFilter* filter, float* nearestPt) const;
	
	// Returns polygons which overlap the query box.
	// Params:
	//	center[3] - (in) the center of the search box.
	//	extents[3] - (in) the extents of the search box.
	//  filter - (in) path polygon filter.
	//	polys - (out) array holding the search result.
	//	maxPolys - (in) The max number of polygons the polys array can hold.
	// Returns: Number of polygons in search result array.
	int queryPolygons(const float* center, const float* extents, const dtQueryFilter* filter,
					  dtPolyRef* polys, const int maxPolys) const;
	
	// Finds path from start polygon to end polygon.
	// If target polygon canno be reached through the navigation graph,
	// the last node on the array is nearest node to the end polygon.
	// Start end end positions are needed to calculate more accurate
	// traversal cost at start end end polygons.
	// Params:
	//	startRef - (in) ref to path start polygon.
	//	endRef - (in) ref to path end polygon.
	//	startPos[3] - (in) Path start location.
	//	endPos[3] - (in) Path end location.
	//  filter - (in) path polygon filter.
	//	path - (out) array holding the search result.
	//	maxPathSize - (in) The max number of polygons the path array can hold.
	// Returns: Number of polygons in search result array.
	int findPath(dtPolyRef startRef, dtPolyRef endRef,
				 const float* startPos, const float* endPos,
				 const dtQueryFilter* filter,
				 dtPolyRef* path, const int maxPathSize) const;
	
	// Intializes sliced path find query.
	// Note: calling any other dtNavMeshQuery method before calling findPathEnd()
	// may results in corrupted data!
	// Params:
	//	startRef - (in) ref to path start polygon.
	//	endRef - (in) ref to path end polygon.
	//	startPos[3] - (in) Path start location.
	//	endPos[3] - (in) Path end location.
	//  filter - (in) path polygon filter.
	// Returns: Path query state.
	dtQueryState initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
									const float* startPos, const float* endPos,
									const dtQueryFilter* filter);

	// Updates sliced path find query.
	// Params:
	//  maxIter - (in) max number of iterations to update.
	// Returns: Path query state.
	dtQueryState updateSlicedFindPath(const int maxIter);

	// Finalizes sliced path find  query.
	//	path - (out) array holding the search result.
	//	maxPathSize - (in) The max number of polygons the path array can hold.
	// Returns: Number of polygons in search result array.
	int finalizeSlicedFindPath(dtPolyRef* path, const int maxPathSize);
	
	// Finds a straight path from start to end locations within the corridor
	// described by the path polygons.
	// Start and end locations will be clamped on the corridor.
	// The returned polygon references are point to polygon which was entered when
	// a path point was added. For the end point, zero will be returned. This allows
	// to match for example off-mesh link points to their representative polygons.
	// Params:
	//	startPos[3] - (in) Path start location.
	//	endPo[3] - (in) Path end location.
	//	path - (in) Array of connected polygons describing the corridor.
	//	pathSize - (in) Number of polygons in path array.
	//	straightPath - (out) Points describing the straight path.
	//  straightPathFlags - (out, opt) Flags describing each point type, see dtStraightPathFlags.
	//  straightPathRefs - (out, opt) References to polygons at point locations.
	//	maxStraightPathSize - (in) The max number of points the straight path array can hold.
	// Returns: Number of points in the path.
	int findStraightPath(const float* startPos, const float* endPos,
						 const dtPolyRef* path, const int pathSize,
						 float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
						 const int maxStraightPathSize) const;
	
	// Moves from startPos to endPos constrained to the navmesh.
	// If the endPos is reachable, the resultPos will be endPos,
	// or else the resultPos will be the nearest point in navmesh.
	// Note: The resulting point is not projected to the ground, use getPolyHeight() to get height.
	// Note: The algorithm is optimized for small delta movement and small number of polygons. 
	// Params:
	//  startRef - (in) ref to the polygon where startPos lies.
	//  startPos[3] - (in) start position of the mover.
	//  endPos[3] - (in) desired end position of the mover.
	//  filter - (in) path polygon filter.
	//  resultPos[3] - (out) new position of the mover.
	//  visited - (out) array of visited polygons.
	//  maxVisitedSize - (in) max number of polygons in the visited array.
	// Returns: Number of entries in the visited array.
	int moveAlongSurface(dtPolyRef startRef, const float* startPos, const float* endPos,
						 const dtQueryFilter* filter,
						 float* resultPos, dtPolyRef* visited, const int maxVisitedSize) const;
	
	// Casts 'walkability' ray along the navmesh surface from startPos towards the endPos.
	// Params:
	//	startRef - (in) ref to the polygon where the start lies.
	//	startPos[3] - (in) start position of the query.
	//	endPos[3] - (in) end position of the query.
	//	t - (out) hit parameter along the segment, FLT_MAX if no hit.
	//	hitNormal[3] - (out) normal of the nearest hit.
	//  filter - (in) path polygon filter.
	//  path - (out) visited path polygons.
	//  pathSize - (in) max number of polygons in the path array.
	// Returns: Number of polygons visited or 0 if failed.
	int raycast(dtPolyRef startRef, const float* startPos, const float* endPos, const dtQueryFilter* filter,
				float& t, float* hitNormal, dtPolyRef* path, const int pathSize) const;
	
	// Returns distance to nearest wall from the specified location.
	// Params:
	//	centerRef - (in) ref to the polygon where the center lies.
	//	centerPos[3] - (in) center if the query circle.
	//	maxRadius - (in) max search radius.
	//  filter - (in) path polygon filter.
	//	hitPos[3] - (out) location of the nearest hit.
	//	hitNormal[3] - (out) normal of the nearest hit.
	// Returns: Distance to nearest wall from the test location.
	float findDistanceToWall(dtPolyRef centerRef, const float* centerPos, float maxRadius,
							 const dtQueryFilter* filter, float* hitPos, float* hitNormal) const;
	
	// Finds polygons found along the navigation graph which touch the specified circle.
	// Params:
	//	startRef - (in) ref to the polygon where the search starts.
	//	centerPos[3] - (in) center if the query circle.
	//	radius - (in) radius of the query circle.
	//  filter - (in) path polygon filter.
	//	resultRef - (out, opt) refs to the polygons touched by the circle.
	//	resultParent - (out, opt) parent of each result polygon.
	//	resultCost - (out, opt) search cost at each result polygon.
	//	maxResult - (int) maximum capacity of search results.
	// Returns: Number of results.
	int	findPolysAroundCircle(dtPolyRef startRef, const float* centerPos, const float radius,
							  const dtQueryFilter* filter,
							  dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
							  const int maxResult) const;
	
	// Finds polygons found along the navigation graph which touch the convex polygon shape.
	// Params:
	//	startRef - (in) ref to the polygon where the search starts.
	//	verts[3*n] - (in) vertices describing convex polygon shape (CCW).
	//	nverts - (in) number of vertices in the polygon.
	//  filter - (in) path polygon filter.
	//	resultRef - (out, opt) refs to the polygons touched by the circle.
	//	resultParent - (out, opt) parent of each result polygon.
	//	resultCost - (out, opt) search cost at each result polygon.
	//	maxResult - (int) maximum capacity of search results.
	// Returns: Number of results.
	int	findPolysAroundShape(dtPolyRef startRef, const float* verts, const int nverts,
							 const dtQueryFilter* filter,
							 dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
							 const int maxResult) const;
	
	// Finds non-overlapping local neighbourhood around center location.
	// Note: The algorithm is optimized for small query radius and small number of polygons. 
	// Params:
	//	startRef - (in) ref to the polygon where the search starts.
	//	centerPos[3] - (in) center if the query circle.
	//	radius - (in) radius of the query circle.
	//  filter - (in) path polygon filter.
	//	resultRef - (out) refs to the polygons touched by the circle.
	//	resultParent - (out, opt) parent of each result polygon.
	//	maxResult - (int) maximum capacity of search results.
	// Returns: Number of results.
	int	findLocalNeighbourhood(dtPolyRef startRef, const float* centerPos, const float radius,
							   const dtQueryFilter* filter,
							   dtPolyRef* resultRef, dtPolyRef* resultParent, const int maxResult) const;
	
	// Returns wall segments of specified polygon.
	// Params:
	//  ref - (in) ref to the polygon.
	//  filter - (in) path polygon filter.
	//  segments[DT_VERTS_PER_POLYGON*3*2] - (out) wall segments.
	// Returns: Number of wall segments.
	int getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter, float* segments);
	
	// Returns closest point on navigation polygon.
	// Uses detail polygons to find the closest point to the navigation polygon surface. 
	// Params:
	//	ref - (in) ref to the polygon.
	//	pos[3] - (in) the point to check.
	//	closest[3] - (out) closest point.
	// Returns: true if closest point found.
	bool closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest) const;
	
	// Returns closest point on navigation polygon boundary.
	// Uses the navigation polygon boundary to snap the point to poly boundary
	// if it is outside the polygon. Much faster than closestPointToPoly. Does not affect height.
	// Params:
	//	ref - (in) ref to the polygon.
	//	pos[3] - (in) the point to check.
	//	closest[3] - (out) closest point.
	// Returns: true if closest point found.
	bool closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, float* closest) const;
	
	// Returns start and end location of an off-mesh link polygon.
	// Params:
	//	prevRef - (in) ref to the polygon before the link (used to select direction).
	//	polyRef - (in) ref to the off-mesh link polygon.
	//	startPos[3] - (out) start point of the link.
	//	endPos[3] - (out) end point of the link.
	// Returns: true if link is found.
	bool getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const;
	
	// Returns height of the polygon at specified location.
	// Params:
	//	ref - (in) ref to the polygon.
	//	pos[3] - (in) the point where to locate the height.
	//	height - (out) height at the location.
	// Returns: true if over polygon.
	bool getPolyHeight(dtPolyRef ref, const float* pos, float* height) const;
		
	// Returns true if poly reference ins in closed list.
	bool isInClosedList(dtPolyRef ref) const;
	
private:
	
	// Returns neighbour tile based on side. 
	dtMeshTile* getNeighbourTileAt(int x, int y, int side) const;

	// Queries polygons within a tile.
	int queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax, const dtQueryFilter* filter,
							dtPolyRef* polys, const int maxPolys) const;
	// Find nearest polygon within a tile.
	dtPolyRef findNearestPolyInTile(const dtMeshTile* tile, const float* center, const float* extents,
									const dtQueryFilter* filter, float* nearestPt) const;
	// Returns closest point on polygon.
	bool closestPointOnPolyInTile(const dtMeshTile* tile, const dtPoly* poly, const float* pos, float* closest) const;
	
	// Returns portal points between two polygons.
	bool getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right,
						 unsigned char& fromType, unsigned char& toType) const;
	bool getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
						 dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
						 float* left, float* right) const;
	
	// Returns edge mid point between two polygons.
	bool getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const;
	bool getEdgeMidPoint(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
						 dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
						 float* mid) const;
	
	dtNavMesh* m_nav;					// Pointer to navmesh data.

	struct dtQueryData
	{
		dtQueryState state;
		struct dtNode* lastBestNode;
		float lastBestNodeCost;
		dtPolyRef startRef, endRef;
		float startPos[3], endPos[3];
		dtQueryFilter filter;
	};
	dtQueryData m_query;				// Sliced query state.

	float m_areaCost[DT_MAX_AREAS];		// Cost per area.

	class dtNodePool* m_tinyNodePool;	// Pointer to small node pool.
	class dtNodePool* m_nodePool;		// Pointer to node pool.
	class dtNodeQueue* m_openList;		// Pointer to open list queue.
};

// Helper function to allocate navmesh query class using Detour allocator.
dtNavMeshQuery* dtAllocNavMeshQuery();
void dtFreeNavMeshQuery(dtNavMeshQuery* query);

#endif // DETOURNAVMESHQUERY_H
