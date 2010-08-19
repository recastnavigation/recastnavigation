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
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"
#include "DetourCommon.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include <new>


// Search heuristic scale.
static const float H_SCALE = 0.999f;

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

inline bool passFilter(const dtQueryFilter* filter, unsigned short flags)
{
	return (flags & filter->includeFlags) != 0 && (flags & filter->excludeFlags) == 0;
}

dtNavMeshQuery* dtAllocNavMeshQuery()
{
	return new(dtAlloc(sizeof(dtNavMeshQuery), DT_ALLOC_PERM)) dtNavMeshQuery;
}

void dtFreeNavMeshQuery(dtNavMeshQuery* navmesh)
{
	if (!navmesh) return;
	navmesh->~dtNavMeshQuery();
	dtFree(navmesh);
}

//////////////////////////////////////////////////////////////////////////////////////////
dtNavMeshQuery::dtNavMeshQuery() :
	m_tinyNodePool(0),
	m_nodePool(0),
	m_openList(0)
{
	for (int i = 0; i < DT_MAX_AREAS; ++i)
		m_areaCost[i] = 1.0f;
	memset(&m_query, 0, sizeof(dtQueryData));
}

dtNavMeshQuery::~dtNavMeshQuery()
{
	if (m_tinyNodePool)
		m_tinyNodePool->~dtNodePool();
	if (m_nodePool)
		m_nodePool->~dtNodePool();
	if (m_openList)
		m_openList->~dtNodeQueue();
	dtFree(m_tinyNodePool);
	dtFree(m_nodePool);
	dtFree(m_openList);
}

bool dtNavMeshQuery::init(dtNavMesh* nav, const int maxNodes)
{
	m_nav = nav;
	
	if (!m_nodePool || m_nodePool->getMaxNodes() < maxNodes)
	{
		if (m_nodePool)
		{
			m_nodePool->~dtNodePool();
			dtFree(m_nodePool);
			m_nodePool = 0;
		}
		m_nodePool = new (dtAlloc(sizeof(dtNodePool), DT_ALLOC_PERM)) dtNodePool(maxNodes, dtNextPow2(maxNodes/4));
		if (!m_nodePool)
			return false;
	}
	else
	{
		m_nodePool->clear();
	}
	
	if (!m_tinyNodePool)
	{
		m_tinyNodePool = new (dtAlloc(sizeof(dtNodePool), DT_ALLOC_PERM)) dtNodePool(64, 32);
		if (!m_tinyNodePool)
			return false;
	}
	else
	{
		m_tinyNodePool->clear();
	}
	
	// TODO: check the open list size too.
	if (!m_openList || m_openList->getCapacity() < maxNodes)
	{
		if (m_openList)
		{
			m_openList->~dtNodeQueue();
			dtFree(m_openList);
			m_openList = 0;
		}
		m_openList = new (dtAlloc(sizeof(dtNodeQueue), DT_ALLOC_PERM)) dtNodeQueue(maxNodes);
		if (!m_openList)
			return false;
	}
	else
	{
		m_openList->clear();
	}
	
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool dtNavMeshQuery::closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest) const
{
	dtAssert(m_nav);
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (!m_nav->getTileAndPolyByRef(ref, &tile, &poly))
		return false;
	if (!tile) return false;
	return closestPointOnPolyInTile(tile, poly, pos, closest);
}

bool dtNavMeshQuery::closestPointOnPolyInTile(const dtMeshTile* tile, const dtPoly* poly, const float* pos, float* closest) const
{
	const unsigned int ip = (unsigned int)(poly - tile->polys);
	const dtPolyDetail* pd = &tile->detailMeshes[ip];

	float closestDistSqr = FLT_MAX;
	
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

bool dtNavMeshQuery::closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, float* closest) const
{
	dtAssert(m_nav);
	
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (!m_nav->getTileAndPolyByRef(ref, &tile, &poly))
		return false;
	
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


bool dtNavMeshQuery::getPolyHeight(dtPolyRef ref, const float* pos, float* height) const
{
	dtAssert(m_nav);

	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (!m_nav->getTileAndPolyByRef(ref, &tile, &poly))
		return false;
	
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
		const unsigned int ip = (unsigned int)(poly - tile->polys);
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

void dtNavMeshQuery::setAreaCost(const int area, float cost)
{
	if (area >= 0 && area < DT_MAX_AREAS)
		m_areaCost[area] = cost;
}

float dtNavMeshQuery::getAreaCost(const int area) const
{
	if (area >= 0 && area < DT_MAX_AREAS)
		return m_areaCost[area];
	return -1;
}

dtPolyRef dtNavMeshQuery::findNearestPoly(const float* center, const float* extents,
										  const dtQueryFilter* filter, float* nearestPt) const
{
	dtAssert(m_nav);

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

dtPolyRef dtNavMeshQuery::findNearestPolyInTile(const dtMeshTile* tile, const float* center, const float* extents,
												const dtQueryFilter* filter, float* nearestPt) const
{
	dtAssert(m_nav);
	
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
		const dtPoly* poly = &tile->polys[m_nav->decodePolyIdPoly(ref)];
		float closestPtPoly[3];
		if (!closestPointOnPolyInTile(tile, poly, center, closestPtPoly))
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

int dtNavMeshQuery::queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
										const dtQueryFilter* filter,
										dtPolyRef* polys, const int maxPolys) const
{
	dtAssert(m_nav);

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
		const dtPolyRef base = m_nav->getPolyRefBase(tile);
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
		const dtPolyRef base = m_nav->getPolyRefBase(tile);
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

int dtNavMeshQuery::queryPolygons(const float* center, const float* extents, const dtQueryFilter* filter,
								  dtPolyRef* polys, const int maxPolys) const
{
	dtAssert(m_nav);
	
	float bmin[3], bmax[3];
	dtVsub(bmin, center, extents);
	dtVadd(bmax, center, extents);
	
	// Find tiles the query touches.
	int minx, miny, maxx, maxy;
	m_nav->calcTileLoc(bmin, &minx, &miny);
	m_nav->calcTileLoc(bmax, &maxx, &maxy);

	int n = 0;
	for (int y = miny; y <= maxy; ++y)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const dtMeshTile* tile = m_nav->getTileAt(x,y);
			if (!tile) continue;
			n += queryPolygonsInTile(tile, bmin, bmax, filter, polys+n, maxPolys-n);
			if (n >= maxPolys) return n;
		}
	}
	
	return n;
}

int dtNavMeshQuery::findPath(dtPolyRef startRef, dtPolyRef endRef,
							 const float* startPos, const float* endPos,
							 const dtQueryFilter* filter,
							 dtPolyRef* path, const int maxPathSize) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);
	
	if (!startRef || !endRef)
		return 0;
	
	if (!maxPathSize)
		return 0;
	
	// Validate input
	if (!m_nav->isValidPolyRef(startRef) || !m_nav->isValidPolyRef(endRef))
		return 0;
	
	if (startRef == endRef)
	{
		path[0] = startRef;
		return 1;
	}
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtNode* startNode = m_nodePool->getNode(startRef);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = dtVdist(startPos, endPos) * H_SCALE;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	dtNode* lastBestNode = startNode;
	float lastBestNodeCost = startNode->total;
	
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
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
		{
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);			
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
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Get neighbour poly and tile.
			// The API input has been cheked already, skip checking internal data.
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);			
			
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



dtQueryState dtNavMeshQuery::initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
												const float* startPos, const float* endPos,
												const dtQueryFilter* filter)
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	// Init path state.
	memset(&m_query, 0, sizeof(dtQueryData));
	m_query.state = DT_QUERY_FAILED;
	m_query.startRef = startRef;
	m_query.endRef = endRef;
	dtVcopy(m_query.startPos, startPos);
	dtVcopy(m_query.endPos, endPos);
	m_query.filter = *filter;
	
	if (!startRef || !endRef)
		return DT_QUERY_FAILED;
	
	// Validate input
	if (!m_nav->isValidPolyRef(startRef) || !m_nav->isValidPolyRef(endRef))
		return DT_QUERY_FAILED;

	if (startRef == endRef)
	{
		m_query.state = DT_QUERY_READY;
		return DT_QUERY_READY;
	}
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtNode* startNode = m_nodePool->getNode(startRef);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = dtVdist(startPos, endPos) * H_SCALE;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	m_query.state = DT_QUERY_RUNNING;
	m_query.lastBestNode = startNode;
	m_query.lastBestNodeCost = startNode->total;
	
	return m_query.state;
}
	
dtQueryState dtNavMeshQuery::updateSlicedFindPath(const int maxIter)
{
	if (m_query.state != DT_QUERY_RUNNING)
		return m_query.state;

	// Make sure the request is still valid.
	if (!m_nav->isValidPolyRef(m_query.startRef) || !m_nav->isValidPolyRef(m_query.endRef))
	{
		m_query.state = DT_QUERY_FAILED;
		return DT_QUERY_FAILED;
	}
		
	int iter = 0;
	while (iter < maxIter && !m_openList->empty())
	{
		iter++;
		
		dtNode* bestNode = m_openList->pop();
		// Remove node from open list and put it in closed list.
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Reached the goal, stop searching.
		if (bestNode->id == m_query.endRef)
		{
			m_query.lastBestNode = bestNode;
			m_query.state = DT_QUERY_READY;
			return m_query.state;
		}
		
		float previousEdgeMidPoint[3];
		
		// Get current poly and tile.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		if (!m_nav->getTileAndPolyByRef(bestRef, &bestTile, &bestPoly))
		{
			// The polygon has disappeared during the sliced query, fail.
			m_query.state = DT_QUERY_FAILED;
			return m_query.state;
		}
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		dtNode* parentNode = 0;
		if (bestNode->pidx)
		{
			parentNode = m_nodePool->getNodeAtIdx(bestNode->pidx);
			parentRef = parentNode->id;
		}
		if (parentRef)
		{
			if (!m_nav->getTileAndPolyByRef(parentRef, &parentTile, &parentPoly))
			{
				// The polygon has disappeared during the sliced query, fail.
				m_query.state = DT_QUERY_FAILED;
				return m_query.state;
			}
			getEdgeMidPoint(parentRef, parentPoly, parentTile,
							bestRef, bestPoly, bestTile, previousEdgeMidPoint);
		}
		else
		{
			dtVcopy(previousEdgeMidPoint, m_query.startPos);
		}
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			dtPolyRef neighbourRef = bestTile->links[i].ref;
			
			// Skip invalid ids and do not expand back to where we came from.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Get neighbour poly and tile.
			// The API input has been cheked already, skip checking internal data.
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);			
			
			if (!passFilter(&m_query.filter, neighbourPoly->flags))
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
			if (neighbourRef == m_query.endRef)
			{
				// Cost
				newNode.cost = bestNode->cost +
				dtVdist(previousEdgeMidPoint,edgeMidPoint) * m_areaCost[bestPoly->area] +
				dtVdist(edgeMidPoint, m_query.endPos) * m_areaCost[neighbourPoly->area];
				// Heuristic
				h = 0;
			}
			else
			{
				// Cost
				newNode.cost = bestNode->cost +
				dtVdist(previousEdgeMidPoint,edgeMidPoint) * m_areaCost[bestPoly->area];
				// Heuristic
				h = dtVdist(edgeMidPoint, m_query.endPos)*H_SCALE;
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
			if (h < m_query.lastBestNodeCost)
			{
				m_query.lastBestNodeCost = h;
				m_query.lastBestNode = actualNode;
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
	
	// Exhausted all nodes, but could not find path.
	if (m_openList->empty())
		m_query.state = DT_QUERY_READY;

	return m_query.state;
}

int dtNavMeshQuery::finalizeSlicedFindPath(dtPolyRef* path, const int maxPathSize)
{
	if (m_query.state != DT_QUERY_READY)
	{
		// Reset query.
		memset(&m_query, 0, sizeof(dtQueryData));
		return 0;
	}
	
	// Reverse the path.
	dtAssert(m_query.lastBestNode);
	dtNode* prev = 0;
	dtNode* node = m_query.lastBestNode;
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
	
	// Reset query.
	memset(&m_query, 0, sizeof(dtQueryData));
	
	return n;
}



int dtNavMeshQuery::findStraightPath(const float* startPos, const float* endPos,
									 const dtPolyRef* path, const int pathSize,
									 float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
									 const int maxStraightPathSize) const
{
	dtAssert(m_nav);
	
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
					if (dtDistancePtSegSqr2D(portalApex, left, right, t) < dtSqr(0.001f))
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

int dtNavMeshQuery::moveAlongSurface(dtPolyRef startRef, const float* startPos, const float* endPos,
									 const dtQueryFilter* filter,
									 float* resultPos, dtPolyRef* visited, const int maxVisitedSize) const
{
	dtAssert(m_nav);
	dtAssert(m_tinyNodePool);

	// Validate input
	if (!startRef) return 0;
	if (!m_nav->isValidPolyRef(startRef)) return 0;
	
	static const int MAX_STACK = 48;
	dtNode* stack[MAX_STACK];
	int nstack = 0;
	
	m_tinyNodePool->clear();
	
	dtNode* startNode = m_tinyNodePool->getNode(startRef);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_CLOSED;
	stack[nstack++] = startNode;
	
	float bestPos[3];
	float bestDist = FLT_MAX;
	dtNode* bestNode = 0;
	dtVcopy(bestPos, startPos);
	
	// Search constraints
	float searchPos[3], searchRadSqr;
	dtVlerp(searchPos, startPos, endPos, 0.5f);
	searchRadSqr = dtSqr(dtVdist(startPos, endPos)/2.0f + 0.001f);
	
	float verts[DT_VERTS_PER_POLYGON*3];
	
	while (nstack)
	{
		// Pop front.
		dtNode* curNode = stack[0];
		for (int i = 0; i < nstack-1; ++i)
			stack[i] = stack[i+1];
		nstack--;
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef curRef = curNode->id;
		const dtMeshTile* curTile = 0;
		const dtPoly* curPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(curRef, &curTile, &curPoly);			
		
		// Collect vertices.
		const int nverts = curPoly->vertCount;
		for (int i = 0; i < nverts; ++i)
			dtVcopy(&verts[i*3], &curTile->verts[curPoly->verts[i]*3]);
		
		// If target is inside the poly, stop search.
		if (dtPointInPolygon(endPos, verts, nverts))
		{
			bestNode = curNode;
			dtVcopy(bestPos, endPos);
			break;
		}
		
		// Find wall edges and find nearest point inside the walls.
		for (int i = 0, j = (int)curPoly->vertCount-1; i < (int)curPoly->vertCount; j = i++)
		{
			// Find links to neighbours.
			static const int MAX_NEIS = 8;
			int nneis = 0;
			dtPolyRef neis[MAX_NEIS];
			
			if (curPoly->neis[j] & DT_EXT_LINK)
			{
				// Tile border.
				for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
				{
					const dtLink* link = &curTile->links[k];
					if (link->edge == j)
					{
						if (link->ref != 0 && passFilter(filter, m_nav->getPolyFlags(link->ref)))
						{
							if (nneis < MAX_NEIS)
								neis[nneis++] = link->ref;
						}
					}
				}
			}
			else if (curPoly->neis[j] && passFilter(filter, curTile->polys[curPoly->neis[j]-1].flags))
			{
				// Internal edge, encode id.
				neis[nneis++] = m_nav->getPolyRefBase(curTile) | (unsigned int)(curPoly->neis[j]-1);
			}
			
			if (!nneis)
			{
				// Wall edge, calc distance.
				const float* vj = &verts[j*3];
				const float* vi = &verts[i*3];
				float tseg;
				const float distSqr = dtDistancePtSegSqr2D(endPos, vj, vi, tseg);
				if (distSqr < bestDist)
				{
                    // Update nearest distance.
					dtVlerp(bestPos, vj,vi, tseg);
					bestDist = distSqr;
					bestNode = curNode;
				}
			}
			else
			{
				for (int k = 0; k < nneis; ++k)
				{
					// Skip if no node can be allocated.
					dtNode* neighbourNode = m_tinyNodePool->getNode(neis[k]);
					if (!neighbourNode)
						continue;
					// Skip if already visited.
					if (neighbourNode->flags & DT_NODE_CLOSED)
						continue;
					
					// Skip the link if it is too far from search constraint.
					// TODO: Maybe should use getPortalPoints(), but this one is way faster.
					const float* vj = &verts[j*3];
					const float* vi = &verts[i*3];
					float tseg;
					float distSqr = dtDistancePtSegSqr2D(searchPos, vj, vi, tseg);
					if (distSqr > searchRadSqr)
						continue;
					
					// Mark as the node as visited and push to queue.
					if (nstack < MAX_STACK)
					{
						neighbourNode->pidx = m_tinyNodePool->getNodeIdx(curNode);
						neighbourNode->flags |= DT_NODE_CLOSED;
						stack[nstack++] = neighbourNode;
					}
				}
			}
		}
	}
	
	int n = 0;
	if (bestNode)
	{
		// Reverse the path.
		dtNode* prev = 0;
		dtNode* node = bestNode;
		do
		{
			dtNode* next = m_tinyNodePool->getNodeAtIdx(node->pidx);
			node->pidx = m_tinyNodePool->getNodeIdx(prev);
			prev = node;
			node = next;
		}
		while (node);
		
		// Store result
		node = prev;
		do
		{
			visited[n++] = node->id;
			node = m_tinyNodePool->getNodeAtIdx(node->pidx);
		}
		while (node && n < maxVisitedSize);
	}
	
	dtVcopy(resultPos, bestPos);
	
	return n;
}


bool dtNavMeshQuery::getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right,
									 unsigned char& fromType, unsigned char& toType) const
{
	dtAssert(m_nav);
	
	const dtMeshTile* fromTile = 0;
	const dtPoly* fromPoly = 0;
	if (!m_nav->getTileAndPolyByRef(from, &fromTile, &fromPoly))
		return false;
	fromType = fromPoly->type;

	const dtMeshTile* toTile = 0;
	const dtPoly* toPoly = 0;
	if (!m_nav->getTileAndPolyByRef(to, &toTile, &toPoly))
		return false;
	toType = toPoly->type;
		
	return getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right);
}

// Returns portal points between two polygons.
bool dtNavMeshQuery::getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
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
	if (link->side != 0xff)
	{
		// Unpack portal limits.
		if (link->bmin != 0 || link->bmax != 255)
		{
			const float s = 1.0f/255.0f;
			const float tmin = link->bmin*s;
			const float tmax = link->bmax*s;
			dtVlerp(left, &fromTile->verts[v0*3], &fromTile->verts[v1*3], tmin);
			dtVlerp(right, &fromTile->verts[v0*3], &fromTile->verts[v1*3], tmax);
		}
	}
	
	return true;
}

// Returns edge mid point between two polygons.
bool dtNavMeshQuery::getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const
{
	float left[3], right[3];
	unsigned char fromType, toType;
	if (!getPortalPoints(from, to, left,right, fromType, toType)) return false;
	mid[0] = (left[0]+right[0])*0.5f;
	mid[1] = (left[1]+right[1])*0.5f;
	mid[2] = (left[2]+right[2])*0.5f;
	return true;
}

bool dtNavMeshQuery::getEdgeMidPoint(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
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

int dtNavMeshQuery::raycast(dtPolyRef centerRef, const float* startPos, const float* endPos, const dtQueryFilter* filter,
							float& t, float* hitNormal, dtPolyRef* path, const int pathSize) const
{
	dtAssert(m_nav);
	
	t = 0;
	
	// Validate input
	if (!centerRef || !m_nav->isValidPolyRef(centerRef))
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
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		m_nav->getTileAndPolyByRefUnsafe(curRef, &tile, &poly);
		
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
			const dtMeshTile* nextTile = 0;
			const dtPoly* nextPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(link->ref, &nextTile, &nextPoly);
			
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
			
			// Check if the link spans the whole edge, and accept.
			if (link->bmin == 0 && link->bmax == 255)
			{
				nextRef = link->ref;
				break;
			}
			
			// Check for partial edge links.
			const int v0 = poly->verts[link->edge];
			const int v1 = poly->verts[(link->edge+1) % poly->vertCount];
			const float* left = &tile->verts[v0*3];
			const float* right = &tile->verts[v1*3];
			
			// Check that the intersection lies inside the link portal.
			if (link->side == 0 || link->side == 4)
			{
				// Calculate link size.
				const float s = 1.0f/255.0f;
				float lmin = left[2] + (right[2] - left[2])*(link->bmin*s);
				float lmax = left[2] + (right[2] - left[2])*(link->bmax*s);
				if (lmin > lmax) dtSwap(lmin, lmax);
				
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
				const float s = 1.0f/255.0f;
				float lmin = left[0] + (right[0] - left[0])*(link->bmin*s);
				float lmax = left[0] + (right[0] - left[0])*(link->bmax*s);
				if (lmin > lmax) dtSwap(lmin, lmax);
				
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

int dtNavMeshQuery::findPolysAroundCircle(dtPolyRef centerRef, const float* centerPos, const float radius,
										  const dtQueryFilter* filter,
										  dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
										  const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	// Validate input
	if (!centerRef) return 0;
	if (!m_nav->isValidPolyRef(centerRef)) return 0;
	
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
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		
		float previousEdgeMidPoint[3];
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
		{
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
			
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
			
			// Expand to neighbour
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
		
			// Do not advance if the polygon is excluded by the filter.
			if (!passFilter(filter, neighbourPoly->flags))
				continue;
			
			// Find edge and calc distance to the edge.
			float va[3], vb[3];
			if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;
			
			// If the circle is not touching the next polygon, skip it.
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			if (distSqr > radiusSqr)
				continue;
			
			dtNode newNode;
			newNode.pidx = m_nodePool->getNodeIdx(bestNode);
			newNode.id = neighbourRef;
			
			// Cost
			float edgeMidPoint[3];
			dtVlerp(edgeMidPoint, va, vb, 0.5f);
			
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

int	dtNavMeshQuery::findPolysAroundShape(dtPolyRef centerRef, const float* verts, const int nverts,
										 const dtQueryFilter* filter,
										 dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
										 const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);
	
	// Validate input
	if (!centerRef) return 0;
	if (!m_nav->isValidPolyRef(centerRef)) return 0;
	
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
	
	float centerPos[3] = {0,0,0};
	for (int i = 0; i < nverts; ++i)
		dtVadd(centerPos,centerPos,&verts[i*3]);
	dtVscale(centerPos,centerPos,1.0f/nverts);
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		
		float previousEdgeMidPoint[3];
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
		{
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
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
			
			// Expand to neighbour
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
			// Do not advance if the polygon is excluded by the filter.
			if (!passFilter(filter, neighbourPoly->flags))
				continue;
			
			// Find edge and calc distance to the edge.
			float va[3], vb[3];
			if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;
			
			// If the poly is not touching the edge to the next polygon, skip the connection it.
			float tmin, tmax;
			int segMin, segMax;
			if (!dtIntersectSegmentPoly2D(va, vb, verts, nverts, tmin, tmax, segMin, segMax))
				continue;
			if (tmin > 1.0f || tmax < 0.0f)
				continue;
			
			dtNode newNode;
			newNode.pidx = m_nodePool->getNodeIdx(bestNode);
			newNode.id = neighbourRef;
			
			// Cost
			float edgeMidPoint[3];
			dtVlerp(edgeMidPoint, va, vb, 0.5f);
			
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


int dtNavMeshQuery::findLocalNeighbourhood(dtPolyRef centerRef, const float* centerPos, const float radius,
										   const dtQueryFilter* filter,
										   dtPolyRef* resultRef, dtPolyRef* resultParent, const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_tinyNodePool);

	// Validate input
	if (!centerRef) return 0;
	if (!m_nav->isValidPolyRef(centerRef)) return 0;
	
	static const int MAX_STACK = 48;
	dtNode* stack[MAX_STACK];
	int nstack = 0;
	
	m_tinyNodePool->clear();
	
	dtNode* startNode = m_tinyNodePool->getNode(centerRef);
	startNode->pidx = 0;
	startNode->id = centerRef;
	startNode->flags = DT_NODE_CLOSED;
	stack[nstack++] = startNode;
	
	const float radiusSqr = dtSqr(radius);
	
	float pa[DT_VERTS_PER_POLYGON*3];
	float pb[DT_VERTS_PER_POLYGON*3];
	
	int n = 0;
	if (n < maxResult)
	{
		resultRef[n] = startNode->id;
		if (resultParent)
			resultParent[n] = 0;
		++n;
	}
	
	while (nstack)
	{
		// Pop front.
		dtNode* curNode = stack[0];
		for (int i = 0; i < nstack-1; ++i)
			stack[i] = stack[i+1];
		nstack--;
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef curRef = curNode->id;
		const dtMeshTile* curTile = 0;
		const dtPoly* curPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(curRef, &curTile, &curPoly);
		
		for (unsigned int i = curPoly->firstLink; i != DT_NULL_LINK; i = curTile->links[i].next)
		{
			const dtLink* link = &curTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours.
			if (!neighbourRef)
				continue;
			
			// Skip if cannot alloca more nodes.
			dtNode* neighbourNode = m_tinyNodePool->getNode(neighbourRef);
			if (!neighbourNode)
				continue;
			// Skip visited.
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;
			
			// Expand to neighbour
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
			// Skip off-mesh connections.
			if (neighbourPoly->type == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			
			// Do not advance if the polygon is excluded by the filter.
			if (!passFilter(filter, neighbourPoly->flags))
				continue;
			
			// Find edge and calc distance to the edge.
			float va[3], vb[3];
			if (!getPortalPoints(curRef, curPoly, curTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;
			
			// If the circle is not touching the next polygon, skip it.
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			if (distSqr > radiusSqr)
				continue;
			
			// Mark node visited, this is done before the overlap test so that
			// we will not visit the poly again if the test fails.
			neighbourNode->flags |= DT_NODE_CLOSED;
			neighbourNode->pidx = m_tinyNodePool->getNodeIdx(curNode);
			
			// Check that the polygon does not collide with existing polygons.
			
			// Collect vertices of the neighbour poly.
			const int npa = neighbourPoly->vertCount;
			for (int k = 0; k < npa; ++k)
				dtVcopy(&pa[k*3], &neighbourTile->verts[neighbourPoly->verts[k]*3]);
			
			bool overlap = false;
			for (int j = 0; j < n; ++j)
			{
				dtPolyRef pastRef = resultRef[j];
				
				// Connected polys do not overlap.
				bool areConnected = false;
				for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
				{
					if (curTile->links[k].ref == pastRef)
					{
						areConnected = true;
						break;
					}
				}
				if (areConnected)
					continue;
				
				// Potentially overlapping.
				const dtMeshTile* pastTile = 0;
				const dtPoly* pastPoly = 0;
				m_nav->getTileAndPolyByRefUnsafe(pastRef, &pastTile, &pastPoly);
				
				// Get vertices and test overlap
				const int npb = pastPoly->vertCount;
				for (int k = 0; k < npb; ++k)
					dtVcopy(&pb[k*3], &pastTile->verts[pastPoly->verts[k]*3]);
				
				if (dtOverlapPolyPoly2D(pa,npa, pb,npb))
				{
					overlap = true;
					break;
				}
			}
			if (overlap)
				continue;
			
			// This poly is fine, store and advance to the poly.
			if (n < maxResult)
			{
				resultRef[n] = neighbourRef;
				if (resultParent)
					resultParent[n] = curRef;
				++n;
			}
			
			if (nstack < MAX_STACK)
			{
				stack[nstack++] = neighbourNode;
			}
		}
	}
	
	return n;
}


struct dtSegInterval
{
	short tmin, tmax;
};

static void insertInterval(dtSegInterval* ints, int& nints, const int maxInts,
						   const short tmin, const short tmax)
{
	if (nints+1 > maxInts) return;
	// Find insertion point.
	int idx = 0;
	while (idx < nints)
	{
		if (tmax <= ints[idx].tmin)
			break;
		idx++;
	}
	// Move current results.
	if (nints-idx)
		memmove(ints+idx+1, ints+idx, sizeof(dtSegInterval)*(nints-idx));
	// Store
	ints[idx].tmin = tmin;
	ints[idx].tmax = tmax;
	nints++;
}

int dtNavMeshQuery::getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter, float* segments)
{
	dtAssert(m_nav);
	
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (!m_nav->getTileAndPolyByRef(ref, &tile, &poly))
		return 0;
	
	int n = 0;
	static const int MAX_INTERVAL = 16;
	dtSegInterval ints[MAX_INTERVAL];
	int nints;
	
	for (int i = 0, j = (int)poly->vertCount-1; i < (int)poly->vertCount; j = i++)
	{
		// Skip non-solid edges.
		nints = 0;
		if (poly->neis[j] & DT_EXT_LINK)
		{
			// Tile border.
			for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
			{
				const dtLink* link = &tile->links[k];
				if (link->edge == j)
				{
					if (link->ref != 0 && passFilter(filter, m_nav->getPolyFlags(link->ref)))
					{
						insertInterval(ints, nints, MAX_INTERVAL, link->bmin, link->bmax);
					}
				}
			}
		}
		else if (poly->neis[j] && passFilter(filter, tile->polys[poly->neis[j]-1].flags))
		{
			// Internal edge
			continue;
		}
		
		// Add sentinels
		insertInterval(ints, nints, MAX_INTERVAL, -1, 0);
		insertInterval(ints, nints, MAX_INTERVAL, 255, 256);
		
		// Store segment.
		const float* vj = &tile->verts[poly->verts[j]*3];
		const float* vi = &tile->verts[poly->verts[i]*3];
		for (int k = 1; k < nints; ++k)
		{
			// Find the space inbetween the opening areas.
			const int imin = ints[k-1].tmax;
			const int imax = ints[k].tmin;
			if (imin == imax) continue;
			if (imin == 0 && imax == 255)
			{
				if (n < DT_VERTS_PER_POLYGON)
				{
					float* seg = &segments[n*6];
					n++;
					dtVcopy(seg+0, vj);
					dtVcopy(seg+3, vi);
				}
			}
			else
			{
				const float tmin = imin/255.0f; 
				const float tmax = imax/255.0f; 
				if (n < DT_VERTS_PER_POLYGON)
				{
					float* seg = &segments[n*6];
					n++;
					dtVlerp(seg+0, vj,vi, tmin);
					dtVlerp(seg+3, vj,vi, tmax);
				}
			}
		}
	}
	
	return n;
}

float dtNavMeshQuery::findDistanceToWall(dtPolyRef centerRef, const float* centerPos, float maxRadius, const dtQueryFilter* filter,
									float* hitPos, float* hitNormal) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);
	
	// Validate input
	if (!centerRef) return 0;
	if (!m_nav->isValidPolyRef(centerRef)) return 0;
	
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
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		
		float previousEdgeMidPoint[3];
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
		{
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
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
						if (link->ref != 0 && passFilter(filter, m_nav->getPolyFlags(link->ref)))
							solid = false;
						break;
					}
				}
				if (!solid) continue;
			}
			else if (bestPoly->neis[j] && passFilter(filter, bestTile->polys[bestPoly->neis[j]-1].flags))
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
			
			// Expand to neighbour.
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
			// Skip off-mesh connections.
			if (neighbourPoly->type == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			
			// Calc distance to the edge.
			const float* va = &bestTile->verts[bestPoly->verts[link->edge]*3];
			const float* vb = &bestTile->verts[bestPoly->verts[(link->edge+1) % bestPoly->vertCount]*3];
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			
			// If the circle is not touching the next polygon, skip it.
			if (distSqr > radiusSqr)
				continue;
			
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

bool dtNavMeshQuery::isInClosedList(dtPolyRef ref) const
{
	if (!m_nodePool) return false;
	const dtNode* node = m_nodePool->findNode(ref);
	return node && node->flags & DT_NODE_CLOSED;
}
