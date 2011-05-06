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

#include "DetourNavMesh.h"
#include "DetourStatus.h"


// Define DT_VIRTUAL_QUERYFILTER if you wish to derive a custom filter from dtQueryFilter.
// On certain platforms indirect or virtual function call is expensive. The default
// setting is to use non-virtual functions, the actualy implementations of the functions
// are declared as inline for maximum speed. 

//#define DT_VIRTUAL_QUERYFILTER 1

/// Class for polygon filtering and cost calculation during query operations.
/// - It is possible to derive a custom query filter from dtQueryFilter by overriding
///   the virtual functions passFilter() and getCost().
/// - Both functions should be as fast as possible. Use cached local copy of data
///   instead of accessing your own objects where possible.
/// - You do not need to adhere to the flags and cost logic provided by the default
///   implementation.
/// - In order for the A* to work properly, the cost should be proportional to
///   the travel distance. Using cost modifier less than 1.0 is likely to lead
///   to problems during pathfinding.
class dtQueryFilter
{
	float m_areaCost[DT_MAX_AREAS];		///< Array storing cost per area type, used by default implementation.
	unsigned short m_includeFlags;		///< Include poly flags, used by default implementation.
	unsigned short m_excludeFlags;		///< Exclude poly flags, used by default implementation.
	
public:
	dtQueryFilter();
	
	/// Returns true if the polygon is can visited.
	///  @param ref [in] reference to the polygon test.
	///  @param tile [in] pointer to the tile of the polygon test.
	///  @param poly [in] pointer to the polygon test.
#ifdef DT_VIRTUAL_QUERYFILTER
	virtual bool passFilter(const dtPolyRef ref,
							const dtMeshTile* tile,
							const dtPoly* poly) const;
#else
	bool passFilter(const dtPolyRef ref,
					const dtMeshTile* tile,
					const dtPoly* poly) const;
#endif

	/// Returns cost to travel from 'pa' to 'pb'.'
	/// The segment is fully contained inside 'cur'.
	/// 'pa' lies on the edge between 'prev' and 'cur',
	/// 'pb' lies on the edge between 'cur' and 'next'.
	///  @param pa [in] segment start position.
	///  @param pb [in] segment end position.
	///  @param prevRef, prevTile, prevPoly [in] data describing the previous polygon, can be null.
	///  @param curRef, curTile, curPoly [in] data describing the current polygon.
	///  @param nextRef, nextTile, nextPoly [in] data describing the next polygon, can be null.
#ifdef DT_VIRTUAL_QUERYFILTER
	virtual float getCost(const float* pa, const float* pb,
						  const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
						  const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
						  const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const;
#else
	float getCost(const float* pa, const float* pb,
				  const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
				  const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
				  const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const;
#endif
	
	/// @name Getters and setters for the default implementation data.
	///@{
	inline float getAreaCost(const int i) const { return m_areaCost[i]; }
	inline void setAreaCost(const int i, const float cost) { m_areaCost[i] = cost; } 

	inline unsigned short getIncludeFlags() const { return m_includeFlags; }
	inline void setIncludeFlags(const unsigned short flags) { m_includeFlags = flags; }

	inline unsigned short getExcludeFlags() const { return m_excludeFlags; }
	inline void setExcludeFlags(const unsigned short flags) { m_excludeFlags = flags; }	
	///@}
};

class dtNavMeshQuery
{
public:
	dtNavMeshQuery();
	~dtNavMeshQuery();
	
	/// Initializes the nav mesh query.
	///  @param nav [in] pointer to navigation mesh data.
	///  @param maxNodes [in] Maximum number of search nodes to use (max 65536).
	dtStatus init(const dtNavMesh* nav, const int maxNodes);
	
	/// Finds the nearest navigation polygon around the center location.
	///  @param center[3] [in] The center of the search box.
	///  @param extents[3] [in] The extents of the search box.
	///  @param filter [in] path polygon filter.
	///  @param nearestRef [out] Reference to the nearest polygon.
	///  @param nearestPt[3] [out, opt] The nearest point on found polygon, null if not needed.
	dtStatus findNearestPoly(const float* center, const float* extents,
							 const dtQueryFilter* filter,
							 dtPolyRef* nearestRef, float* nearestPt) const;
	
	/// Returns polygons which overlap the query box.
	///  @param center[3] [in] the center of the search box.
	///  @param extents[3] [in] the extents of the search box.
	///  @param filter [in] path polygon filter.
	///  @param polys [out] array holding the search result.
	///  @param polyCount [out] Number of polygons in search result array.
	///  @param maxPolys [in] The max number of polygons the polys array can hold.
	dtStatus queryPolygons(const float* center, const float* extents,
						   const dtQueryFilter* filter,
						   dtPolyRef* polys, int* polyCount, const int maxPolys) const;
	
	/// Finds path from start polygon to end polygon.
	/// If target polygon canno be reached through the navigation graph,
	/// the last node on the array is nearest node to the end polygon.
	/// Start end end positions are needed to calculate more accurate
	/// traversal cost at start end end polygons.
	///  @param startRef [in] ref to path start polygon.
	///  @param endRef [in] ref to path end polygon.
	///  @param startPos[3] [in] Path start location.
	///  @param endPos[3] [in] Path end location.
	///  @param filter [in] path polygon filter.
	///  @param path [out] array holding the search result.
	///  @param pathCount [out] Number of polygons in search result array.
	///  @param maxPath [in] The max number of polygons the path array can hold. Must be at least 1.
	dtStatus findPath(dtPolyRef startRef, dtPolyRef endRef,
					  const float* startPos, const float* endPos,
					  const dtQueryFilter* filter,
					  dtPolyRef* path, int* pathCount, const int maxPath) const;
	
	/// Intializes sliced path find query.
	/// Note 1: calling any other dtNavMeshQuery method before calling findPathEnd()
	/// may results in corrupted data!
	/// Note 2: The pointer to filter is store, and used in subsequent
	/// calls to updateSlicedFindPath().
	///  @param startRef [in] ref to path start polygon.
	///  @param endRef [in] ref to path end polygon.
	///  @param startPos[3] [in] Path start location.
	///  @param endPos[3] [in] Path end location.
	///  @param filter [in] path polygon filter.
	dtStatus initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
								const float* startPos, const float* endPos,
								const dtQueryFilter* filter);

	/// Updates sliced path find query.
	///  @param maxIter [in] Max number of iterations to update.
	///  @param doneIters [out,opt] Number of iterations done during the update.
	/// Returns: Path query state.
	dtStatus updateSlicedFindPath(const int maxIter, int* doneIters);

	/// Finalizes sliced path find query and returns found path.
	///  @param path [out] array holding the search result.
	///  @param pathCount [out] Number of polygons in search result array.
	///  @param maxPath [in] The max number of polygons the path array can hold.
	dtStatus finalizeSlicedFindPath(dtPolyRef* path, int* pathCount, const int maxPath);
	
	/// Finalizes partial sliced path find query and returns path to the furthest
	/// polygon on the existing path that was visited during the search.
	///  @param existing [out] Array of polygons in the existing path.
	///  @param existingSize [out] Number of polygons in existing path array.
	///  @param path [out] array holding the search result.
	///  @param pathCount [out] Number of polygons in search result array.
	///  @param maxPath [in] The max number of polygons the path array can hold.
	dtStatus finalizeSlicedFindPathPartial(const dtPolyRef* existing, const int existingSize,
										   dtPolyRef* path, int* pathCount, const int maxPath);
	
	/// Finds a straight path from start to end locations within the corridor
	/// described by the path polygons.
	/// Start and end locations will be clamped on the corridor.
	/// The returned polygon references are point to polygon which was entered when
	/// a path point was added. For the end point, zero will be returned. This allows
	/// to match for example off-mesh link points to their representative polygons.
	///  @param startPos[3] [in] Path start location.
	///  @param endPos[3] [in] Path end location.
	///  @param path [in] Array of connected polygons describing the corridor.
	///  @param pathSize [in] Number of polygons in path array.
	///  @param straightPath [out] Points describing the straight path.
	///  @param straightPathFlags [out, opt] Flags describing each point type, see dtStraightPathFlags.
	///  @param straightPathRefs [out, opt] References to polygons at point locations.
	///  @param straightPathCount [out] Number of points in the path.
	///  @param maxStraightPath [in] The max number of points the straight path array can hold. Must be at least 1.
	dtStatus findStraightPath(const float* startPos, const float* endPos,
							  const dtPolyRef* path, const int pathSize,
							  float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
							  int* straightPathCount, const int maxStraightPath) const;
	
	/// Moves from startPos to endPos constrained to the navmesh.
	/// If the endPos is reachable, the resultPos will be endPos,
	/// or else the resultPos will be the nearest point in navmesh.
	/// Note: The resulting point is not projected to the ground, use getPolyHeight() to get height.
	/// Note: The algorithm is optimized for small delta movement and small number of polygons.
	///  @param startRef [in] ref to the polygon where startPos lies.
	///  @param startPos[3] [in] start position of the mover.
	///  @param endPos[3] [in] desired end position of the mover.
	///  @param filter [in] path polygon filter.
	///  @param resultPos[3] [out] new position of the mover.
	///  @param visited [out] array of visited polygons.
	///  @param visitedCount [out] Number of entries in the visited array.
	///  @param maxVisitedSize [in] max number of polygons in the visited array.
	dtStatus moveAlongSurface(dtPolyRef startRef, const float* startPos, const float* endPos,
							  const dtQueryFilter* filter,
							  float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) const;
	
	/// Casts 'walkability' ray along the navmesh surface from startPos towards the endPos.
	///  @param startRef [in] ref to the polygon where the start lies.
	///  @param startPos[3] [in] start position of the query.
	///  @param endPos[3] [in] end position of the query.
	///  @param t [out] hit parameter along the segment, FLT_MAX if no hit.
	///  @param hitNormal[3] [out] normal of the nearest hit.
	///  @param filter [in] path polygon filter.
	///  @param path [out,opt] visited path polygons.
	///  @param pathCount [out,opt] Number of polygons visited.
	///  @param maxPath [in] max number of polygons in the path array.
	dtStatus raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
					 const dtQueryFilter* filter,
					 float* t, float* hitNormal, dtPolyRef* path, int* pathCount, const int maxPath) const;
	
	/// Returns distance to nearest wall from the specified location.
	///  @param startRef [in] ref to the polygon where the center lies.
	///  @param centerPos[3] [in] center if the query circle.
	///  @param maxRadius [in] max search radius.
	///  @param filter [in] path polygon filter.
	///  @param hitDist [out] distance to nearest wall from the test location.
	///  @param hitPos[3] [out] location of the nearest hit.
	///  @param hitNormal[3] [out] normal of the nearest hit.
	dtStatus findDistanceToWall(dtPolyRef startRef, const float* centerPos, const float maxRadius,
								const dtQueryFilter* filter,
								float* hitDist, float* hitPos, float* hitNormal) const;
	
	/// Finds polygons found along the navigation graph which touch the specified circle.
	///  @param startRef [in] ref to the polygon where the search starts.
	///  @param centerPos[3] [in] center if the query circle.
	///  @param radius [in] radius of the query circle.
	///  @param filter [in] path polygon filter.
	///  @param resultRef [out, opt] refs to the polygons touched by the circle.
	///  @param resultParent [out, opt] parent of each result polygon.
	///  @param resultCost [out, opt] search cost at each result polygon.
	///  @param resultCount [out, opt] Number of results.
	///  @param maxResult [int] maximum capacity of search results.
	dtStatus findPolysAroundCircle(dtPolyRef startRef, const float* centerPos, const float radius,
								   const dtQueryFilter* filter,
								   dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
								   int* resultCount, const int maxResult) const;
	
	/// Finds polygons found along the navigation graph which touch the convex polygon shape.
	///  @param startRef [in] ref to the polygon where the search starts.
	///  @param verts[3*n] [in] vertices describing convex polygon shape (CCW).
	///  @param nverts [in] number of vertices in the polygon.
	///  @param filter [in] path polygon filter.
	///  @param resultRef [out, opt] refs to the polygons touched by the circle.
	///  @param resultParent [out, opt] parent of each result polygon.
	///  @param resultCost [out, opt] search cost at each result polygon.
	///  @param resultCount [out] number of results.
	///  @param maxResult [int] maximum capacity of search results.
	dtStatus findPolysAroundShape(dtPolyRef startRef, const float* verts, const int nverts,
								  const dtQueryFilter* filter,
								  dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
								  int* resultCount, const int maxResult) const;
	
	/// Finds non-overlapping local neighbourhood around center location.
	/// Note: The algorithm is optimized for small query radius and small number of polygons.
	///  @param startRef [in] ref to the polygon where the search starts.
	///  @param centerPos[3] [in] center if the query circle.
	///  @param radius [in] radius of the query circle.
	///  @param filter [in] path polygon filter.
	///  @param resultRef [out] refs to the polygons touched by the circle.
	///  @param resultParent [out, opt] parent of each result polygon.
	///  @param resultCount [out] number of results.
	///  @param maxResult [int] maximum capacity of search results.
	dtStatus findLocalNeighbourhood(dtPolyRef startRef, const float* centerPos, const float radius,
									const dtQueryFilter* filter,
									dtPolyRef* resultRef, dtPolyRef* resultParent,
									int* resultCount, const int maxResult) const;
	
	/// Returns wall segments of specified polygon.
	/// If 'segmentRefs' is specified, both the wall and portal segments are returned.
	/// Wall segments will have null (0) polyref, and portal segments store the polygon they lead to.
	///  @param ref [in] ref to the polygon.
	///  @param filter [in] path polygon filter.
	///  @param segmentVerts[6*maxSegments] [out] wall segments (2 endpoints per segment).
	///  @param segmentRefs[maxSegments] [out,opt] reference to a neighbour.
	///  @param segmentCount [out] number of wall segments.
	///  @param maxSegments [in] max number of segments that can be stored in 'segments'.
	dtStatus getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter,
								 float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
								 const int maxSegments) const;
	
	/// Returns closest point on navigation polygon.
	/// Uses detail polygons to find the closest point to the navigation polygon surface.
	///  @param ref [in] ref to the polygon.
	///  @param pos[3] [in] the point to check.
	///  @param closest[3] [out] closest point.
	///  @returns true if closest point found.
	dtStatus closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest) const;
	
	/// Returns closest point on navigation polygon boundary.
	/// Uses the navigation polygon boundary to snap the point to poly boundary
	/// if it is outside the polygon. Much faster than closestPointToPoly. Does not affect height.
	///  @param ref [in] ref to the polygon.
	///  @param pos[3] [in] the point to check.
	///  @param closest[3] [out] closest point.
	///  @returns true if closest point found.
	dtStatus closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, float* closest) const;
	
	/// Returns height of the polygon at specified location.
	///  @param ref [in] ref to the polygon.
	///  @param pos[3] [in] the point where to locate the height.
	///  @param height [out] height at the location.
	///  @returns true if over polygon.
	dtStatus getPolyHeight(dtPolyRef ref, const float* pos, float* height) const;
		
	/// Returns true if poly reference ins in closed list.
	bool isInClosedList(dtPolyRef ref) const;
	
	class dtNodePool* getNodePool() const { return m_nodePool; }
	
	const dtNavMesh* getAttachedNavMesh() const { return m_nav; }
	
private:
	
	/// Returns neighbour tile based on side.
	dtMeshTile* getNeighbourTileAt(int x, int y, int side) const;

	/// Queries polygons within a tile.
	int queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax, const dtQueryFilter* filter,
							dtPolyRef* polys, const int maxPolys) const;
	/// Find nearest polygon within a tile.
	dtPolyRef findNearestPolyInTile(const dtMeshTile* tile, const float* center, const float* extents,
									const dtQueryFilter* filter, float* nearestPt) const;
	/// Returns closest point on polygon.
	void closestPointOnPolyInTile(const dtMeshTile* tile, const dtPoly* poly, const float* pos, float* closest) const;
	
	/// Returns portal points between two polygons.
	dtStatus getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right,
							 unsigned char& fromType, unsigned char& toType) const;
	dtStatus getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
							 dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
							 float* left, float* right) const;
	
	/// Returns edge mid point between two polygons.
	dtStatus getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const;
	dtStatus getEdgeMidPoint(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
							 dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
							 float* mid) const;
	
	const dtNavMesh* m_nav;				///< Pointer to navmesh data.

	struct dtQueryData
	{
		dtStatus status;
		struct dtNode* lastBestNode;
		float lastBestNodeCost;
		dtPolyRef startRef, endRef;
		float startPos[3], endPos[3];
		const dtQueryFilter* filter;
	};
	dtQueryData m_query;				///< Sliced query state.

	class dtNodePool* m_tinyNodePool;	///< Pointer to small node pool.
	class dtNodePool* m_nodePool;		///< Pointer to node pool.
	class dtNodeQueue* m_openList;		///< Pointer to open list queue.
};

/// Helper function to allocate navmesh query class using Detour allocator.
dtNavMeshQuery* dtAllocNavMeshQuery();
void dtFreeNavMeshQuery(dtNavMeshQuery* query);

#endif // DETOURNAVMESHQUERY_H
