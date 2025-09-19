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
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

/// Check whether two bounding boxes overlap
///
/// @param[in]	aMin	Min axis extents of bounding box A
/// @param[in]	aMax	Max axis extents of bounding box A
/// @param[in]	bMin	Min axis extents of bounding box B
/// @param[in]	bMax	Max axis extents of bounding box B
/// @returns true if the two bounding boxes overlap.  False otherwise.
static bool overlapBounds(const float* aMin, const float* aMax, const float* bMin, const float* bMax)
{
	return
		aMin[0] <= bMax[0] && aMax[0] >= bMin[0] &&
		aMin[1] <= bMax[1] && aMax[1] >= bMin[1] &&
		aMin[2] <= bMax[2] && aMax[2] >= bMin[2];
}
static bool overlapBounds(const Vector3& aMin, const Vector3& aMax, const Vector3& bMin, const Vector3&  bMax)
{
	return
		aMin.x <= bMax.x && aMax.x >= bMin.x &&
		aMin.y <= bMax.y && aMax.y >= bMin.y &&
		aMin.z <= bMax.z && aMax.z >= bMin.z;
}

/// Allocates a new span in the heightfield.
/// Use a memory pool and free list to minimize actual allocations.
/// 
/// @param[in]	heightfield		The heightfield
/// @returns A pointer to the allocated or re-used span memory. 
static rcSpan* allocSpan(rcHeightfield& heightfield)
{
	// If necessary, allocate new page and update the freelist.
	if (heightfield.freelist == NULL || heightfield.freelist->next == NULL)
	{
		// Create new page.
		// Allocate memory for the new pool.
		rcSpanPool* spanPool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (spanPool == NULL)
		{
			return NULL;
		}

		// Add the pool into the list of pools.
		spanPool->next = heightfield.pools;
		heightfield.pools = spanPool;
		
		// Add new spans to the free list.
		rcSpan* freeList = heightfield.freelist;
		rcSpan* head = &spanPool->items[0];
		rcSpan* it = &spanPool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freeList;
			freeList = it;
		}
		while (it != head);
		heightfield.freelist = it;
	}

	// Pop item from the front of the free list.
	rcSpan* newSpan = heightfield.freelist;
	heightfield.freelist = heightfield.freelist->next;
	return newSpan;
}

/// Releases the memory used by the span back to the heightfield, so it can be re-used for new spans.
/// @param[in]	heightfield		The heightfield.
/// @param[in]	span	A pointer to the span to free
static void freeSpan(rcHeightfield& heightfield, rcSpan* span)
{
	if (span == NULL)
	{
		return;
	}
	// Add the span to the front of the free list.
	span->next = heightfield.freelist;
	heightfield.freelist = span;
}

/// Adds a span to the heightfield.  If the new span overlaps existing spans,
/// it will merge the new span with the existing ones.
///
/// @param[in]	heightfield					Heightfield to add spans to
/// @param[in]	x					The new span's column cell x index
/// @param[in]	z					The new span's column cell z index
/// @param[in]	min					The new span's minimum cell index
/// @param[in]	max					The new span's maximum cell index
/// @param[in]	areaID				The new span's area type ID
/// @param[in]	flagMergeThreshold	How close two spans maximum extents need to be to merge area type IDs
static bool addSpan(rcHeightfield& heightfield,
                    const int x, const int z,
                    const unsigned short min, const unsigned short max,
                    const unsigned char areaID, const int flagMergeThreshold)
{
	// Create the new span.
	rcSpan* newSpan = allocSpan(heightfield);
	if (newSpan == NULL)
	{
		return false;
	}
	newSpan->smin = min;
	newSpan->smax = max;
	newSpan->area = areaID;
	newSpan->next = NULL;
	
	const int columnIndex = x + z * heightfield.width;
	rcSpan* previousSpan = NULL;
	rcSpan* currentSpan = heightfield.spans[columnIndex];
	
	// Insert the new span, possibly merging it with existing spans.
	while (currentSpan != NULL)
	{
		if (currentSpan->smin > newSpan->smax)
		{
			// Current span is completely after the new span, break.
			break;
		}
		
		if (currentSpan->smax < newSpan->smin)
		{
			// Current span is completely before the new span.  Keep going.
			previousSpan = currentSpan;
			currentSpan = currentSpan->next;
		}
		else
		{
			// The new span overlaps with an existing span.  Merge them.
			if (currentSpan->smin < newSpan->smin)
			{
				newSpan->smin = currentSpan->smin;
			}
			if (currentSpan->smax > newSpan->smax)
			{
				newSpan->smax = currentSpan->smax;
			}
			
			// Merge flags.
			if (rcAbs((int)newSpan->smax - (int)currentSpan->smax) <= flagMergeThreshold)
			{
				// Higher area ID numbers indicate higher resolution priority.
				newSpan->area = rcMax(newSpan->area, currentSpan->area);
			}
			
			// Remove the current span since it's now merged with newSpan.
			// Keep going because there might be other overlapping spans that also need to be merged.
			rcSpan* next = currentSpan->next;
			freeSpan(heightfield, currentSpan);
			if (previousSpan)
			{
				previousSpan->next = next;
			}
			else
			{
				heightfield.spans[columnIndex] = next;
			}
			currentSpan = next;
		}
	}
	
	// Insert new span after prev
	if (previousSpan != NULL)
	{
		newSpan->next = previousSpan->next;
		previousSpan->next = newSpan;
	}
	else
	{
		// This span should go before the others in the list
		newSpan->next = heightfield.spans[columnIndex];
		heightfield.spans[columnIndex] = newSpan;
	}

	return true;
}

bool rcAddSpan(rcContext* context, rcHeightfield& heightfield,
               const int x, const int z,
               const unsigned short spanMin, const unsigned short spanMax,
               const unsigned char areaID, const int flagMergeThreshold)
{
	rcAssert(context);

	if (!addSpan(heightfield, x, z, spanMin, spanMax, areaID, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcAddSpan: Out of memory.");
		return false;
	}

	return true;
}

enum rcAxis
{
	RC_AXIS_X = 0,
	RC_AXIS_Y = 1,
	RC_AXIS_Z = 2
};

/// Divides a convex polygon of max 12 vertices into two convex polygons
/// across a separating axis.
/// 
/// @param[in]	inVerts			The input polygon vertices
/// @param[in]	inVertsCount	The number of input polygon vertices
/// @param[out]	outVerts1		Resulting polygon 1's vertices
/// @param[out]	outVerts1Count	The number of resulting polygon 1 vertices
/// @param[out]	outVerts2		Resulting polygon 2's vertices
/// @param[out]	outVerts2Count	The number of resulting polygon 2 vertices
/// @param[in]	axisOffset		THe offset along the specified axis
/// @param[in]	axis			The separating axis
static void dividePoly(const float* inVerts, int inVertsCount,
                       float* outVerts1, int* outVerts1Count,
                       float* outVerts2, int* outVerts2Count,
                       float axisOffset, rcAxis axis)
{
	rcAssert(inVertsCount <= 12);
	
	// How far positive or negative away from the separating axis is each vertex.
	float inVertAxisDelta[12];
	for (int inVert = 0; inVert < inVertsCount; ++inVert)
	{
		inVertAxisDelta[inVert] = axisOffset - inVerts[inVert * 3 + axis];
	}

	int poly1Vert = 0;
	int poly2Vert = 0;
	for (int inVertA = 0, inVertB = inVertsCount - 1; inVertA < inVertsCount; inVertB = inVertA, ++inVertA)
	{
		// If the two vertices are on the same side of the separating axis
		bool sameSide = (inVertAxisDelta[inVertA] >= 0) == (inVertAxisDelta[inVertB] >= 0);

		if (!sameSide)
		{
			float s = inVertAxisDelta[inVertB] / (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA]);
			outVerts1[poly1Vert * 3 + 0] = inVerts[inVertB * 3 + 0] + (inVerts[inVertA * 3 + 0] - inVerts[inVertB * 3 + 0]) * s;
			outVerts1[poly1Vert * 3 + 1] = inVerts[inVertB * 3 + 1] + (inVerts[inVertA * 3 + 1] - inVerts[inVertB * 3 + 1]) * s;
			outVerts1[poly1Vert * 3 + 2] = inVerts[inVertB * 3 + 2] + (inVerts[inVertA * 3 + 2] - inVerts[inVertB * 3 + 2]) * s;
			rcVcopy(&outVerts2[poly2Vert * 3], &outVerts1[poly1Vert * 3]);
			poly1Vert++;
			poly2Vert++;
			
			// add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (inVertAxisDelta[inVertA] > 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
			}
			else if (inVertAxisDelta[inVertA] < 0)
			{
				rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
				poly2Vert++;
			}
		}
		else
		{
			// add the inVertA point to the right polygon. Addition is done even for points on the dividing line
			if (inVertAxisDelta[inVertA] >= 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
				if (inVertAxisDelta[inVertA] != 0)
				{
					continue;
				}
			}
			rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
			poly2Vert++;
		}
	}

	*outVerts1Count = poly1Vert;
	*outVerts2Count = poly2Vert;
}

static void dividePoly(const Vector3* inVerts, int inVertsCount,
	Vector3* outVerts1, int* outVerts1Count,
	Vector3* outVerts2, int* outVerts2Count,
	float axisOffset, rcAxis axis)
{
	rcAssert(inVertsCount <= 12);

	// How far positive or negative away from the separating axis is each vertex.
	float inVertAxisDelta[12];
	for (int inVert = 0; inVert < inVertsCount; ++inVert)
	{
		inVertAxisDelta[inVert] = axisOffset;
		if (axis == RC_AXIS_X)
		{
			inVertAxisDelta[inVert] -= inVerts[inVert].x;
		}
		else if (axis == RC_AXIS_Y)
		{
			inVertAxisDelta[inVert] -= inVerts[inVert].y;
		}
		else
		{
			inVertAxisDelta[inVert] -= inVerts[inVert].z;
		}
	}

	int poly1Vert = 0;
	int poly2Vert = 0;
	// inVertA 0 1 2
	// inVertB 2 0 1
	for (int inVertA = 0, inVertB = inVertsCount - 1; inVertA < inVertsCount; inVertB = inVertA, ++inVertA)
	{
		// If the two vertices are on the same side of the separating axis
		bool sameSide = (inVertAxisDelta[inVertA] >= 0) == (inVertAxisDelta[inVertB] >= 0);

		if (!sameSide)
		{
			//通过相似三角形原理, 算出切割轴与d[j],d[i]这条边的交点
			//与切割轴的交点肯定是切割后的两个多边形共有的

			float s = inVertAxisDelta[inVertB] / (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA]);

			outVerts1[poly1Vert].x = inVerts[inVertB].x + (inVerts[inVertA].x - inVerts[inVertB].x) * s;
			outVerts1[poly1Vert].y = inVerts[inVertB].y + (inVerts[inVertA].y - inVerts[inVertB].y) * s;
			outVerts1[poly1Vert].z = inVerts[inVertB].z + (inVerts[inVertA].z - inVerts[inVertB].z) * s;
			rcVcopy(outVerts2[poly2Vert], outVerts1[poly1Vert]);
			poly1Vert++;
			poly2Vert++;

			// add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (inVertAxisDelta[inVertA] > 0)
			{
				rcVcopy(outVerts1[poly1Vert], inVerts[inVertA]);
				poly1Vert++;
			}
			else if (inVertAxisDelta[inVertA] < 0)
			{
				rcVcopy(outVerts2[poly2Vert], inVerts[inVertA]);
				poly2Vert++;
			}
		}
		else
		{
			//符号相同,说明d[j],d[i]这条边在切割轴的一侧,此时与切割轴无交点
			// add the inVertA point to the right polygon. Addition is done even for points on the dividing line
			if (inVertAxisDelta[inVertA] >= 0)
			{
				rcVcopy(outVerts1[poly1Vert], inVerts[inVertA]);
				poly1Vert++;
				if (inVertAxisDelta[inVertA] != 0)
				{
					continue;
				}
			}
			rcVcopy(outVerts2[poly2Vert], inVerts[inVertA]);
			poly2Vert++;
		}
	}

	*outVerts1Count = poly1Vert;
	*outVerts2Count = poly2Vert;
}

///	Rasterize a single triangle to the heightfield.
///
///	This code is extremely hot, so much care should be given to maintaining maximum perf here.
/// 
/// @param[in] 	v0					Triangle vertex 0
/// @param[in] 	v1					Triangle vertex 1
/// @param[in] 	v2					Triangle vertex 2
/// @param[in] 	areaID				The area ID to assign to the rasterized spans
/// @param[in] 	heightfield			Heightfield to rasterize into
/// @param[in] 	heightfieldBBMin	The min extents of the heightfield bounding box
/// @param[in] 	heightfieldBBMax	The max extents of the heightfield bounding box
/// @param[in] 	cellSize			The x and z axis size of a voxel in the heightfield
/// @param[in] 	inverseCellSize		1 / cellSize
/// @param[in] 	inverseCellHeight	1 / cellHeight
/// @param[in] 	flagMergeThreshold	The threshold in which area flags will be merged 
/// @returns true if the operation completes successfully.  false if there was an error adding spans to the heightfield.
static bool rasterizeTri(const Vector3& v0, const Vector3& v1, const Vector3& v2,
						 // const float* v0, const float* v1, const float* v2,
                         const unsigned char areaID, rcHeightfield& heightfield,
                         //const float* heightfieldBBMin, const float* heightfieldBBMax,
						 const Vector3& heightfieldBBMin, const Vector3& heightfieldBBMax,
                         const float cellSize, const float inverseCellSize, const float inverseCellHeight,
                         const int flagMergeThreshold)
{
	// Calculate the bounding box of the triangle.
	//float triBBMin[3];
	//rcVcopy(triBBMin, v0);
	//rcVmin(triBBMin, v1);
	//rcVmin(triBBMin, v2);
	Vector3 triBBMin;
	rcVcopy(triBBMin, v0);
	rcVmin(triBBMin, v1);
	rcVmin(triBBMin, v2);

	//float triBBMax[3];
	//rcVcopy(triBBMax, v0);
	//rcVmax(triBBMax, v1);
	//rcVmax(triBBMax, v2);
	Vector3 triBBMax;
	rcVcopy(triBBMax, v0);
	rcVmax(triBBMax, v1);
	rcVmax(triBBMax, v2);

	// If the triangle does not touch the bounding box of the heightfield, skip the triangle.
	if (!overlapBounds(triBBMin, triBBMax, heightfieldBBMin, heightfieldBBMax))
	{
		return true;
	}

	const int w = heightfield.width;
	const int h = heightfield.height;
	const float by = heightfieldBBMax.y - heightfieldBBMin.y;

	// Calculate the footprint of the triangle on the grid's z-axis
	int z0 = (int)((triBBMin.z - heightfieldBBMin.z) * inverseCellSize);
	int z1 = (int)((triBBMax.z - heightfieldBBMin.z) * inverseCellSize);

	// use -1 rather than 0 to cut the polygon properly at the start of the tile
	z0 = rcClamp(z0, -1, h - 1);
	z1 = rcClamp(z1, 0, h - 1);

	// Clip the triangle into all grid cells it touches.
	//float buf[7 * 3 * 4];
	//float* in = buf;
	//float* inRow = buf + 7 * 3;
	//float* p1 = inRow + 7 * 3;
	//float* p2 = p1 + 7 * 3;

	Vector3 buf[7 * 4];
	Vector3* in = buf;
	Vector3* inRow = buf + 7;
	Vector3* p1 = inRow + 7;
	Vector3* p2 = p1 + 7;

	rcVcopy(in[0], v0);
	rcVcopy(in[1], v1);
	rcVcopy(in[2], v2);
	int nvRow;
	int nvIn = 3;

	for (int z = z0; z <= z1; ++z)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float cellZ = heightfieldBBMin.z + (float)z * cellSize;
		dividePoly(in, nvIn, inRow, &nvRow, p1, &nvIn, cellZ + cellSize, RC_AXIS_Z);
		rcSwap(in, p1);
		
		if (nvRow < 3)
		{
			continue;
		}
		if (z < 0)
		{
			continue;
		}
		
		// find X-axis bounds of the row
		float minX = inRow[0].x;
		float maxX = inRow[0].x;
		for (int vert = 1; vert < nvRow; ++vert)
		{
			if (minX > inRow[vert].x)
			{
				minX = inRow[vert].x;
			}
			if (maxX < inRow[vert].x)
			{
				maxX = inRow[vert].x;
			}
		}
		int x0 = (int)((minX - heightfieldBBMin.x) * inverseCellSize);
		int x1 = (int)((maxX - heightfieldBBMin.x) * inverseCellSize);
		if (x1 < 0 || x0 >= w)
		{
			continue;
		}
		x0 = rcClamp(x0, -1, w - 1);
		x1 = rcClamp(x1, 0, w - 1);

		int nv;
		int nv2 = nvRow;

		for (int x = x0; x <= x1; ++x)
		{
			// Clip polygon to column. store the remaining polygon as well
			const float cx = heightfieldBBMin.x + (float)x * cellSize;
			dividePoly(inRow, nv2, p1, &nv, p2, &nv2, cx + cellSize, RC_AXIS_X);
			rcSwap(inRow, p2);
			
			if (nv < 3)
			{
				continue;
			}
			if (x < 0)
			{
				continue;
			}
			
			// Calculate min and max of the span.
			float spanMin = p1[0].y;
			float spanMax = p1[0].y;
			for (int vert = 1; vert < nv; ++vert)
			{
				spanMin = rcMin(spanMin, p1[vert].y);
				spanMax = rcMax(spanMax, p1[vert].y);
			}
			spanMin -= heightfieldBBMin.y;
			spanMax -= heightfieldBBMin.y;
			
			// Skip the span if it's completely outside the heightfield bounding box
			if (spanMax < 0.0f)
			{
				continue;
			}
			if (spanMin > by)
			{
				continue;
			}
			
			// Clamp the span to the heightfield bounding box.
			if (spanMin < 0.0f)
			{
				spanMin = 0;
			}
			if (spanMax > by)
			{
				spanMax = by;
			}

			// Snap the span to the heightfield height grid.
			unsigned short spanMinCellIndex = (unsigned short)rcClamp((int)floorf(spanMin * inverseCellHeight), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short spanMaxCellIndex = (unsigned short)rcClamp((int)ceilf(spanMax * inverseCellHeight), (int)spanMinCellIndex + 1, RC_SPAN_MAX_HEIGHT);

			if (!addSpan(heightfield, x, z, spanMinCellIndex, spanMaxCellIndex, areaID, flagMergeThreshold))
			{
				return false;
			}
		}
	}

	return true;
}

bool rcRasterizeTriangle(rcContext* context,
                         //const float* v0, const float* v1, const float* v2,
						 const Vector3& v0, const Vector3& v1, const Vector3& v2,
                         const unsigned char areaID, rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the single triangle.
	auto& bounds = heightfield.bounds;
	const float inverseCellSize = 1.0f / bounds.cs;
	const float inverseCellHeight = 1.0f / bounds.ch;
	if (!rasterizeTri(v0, v1, v2, areaID, heightfield, bounds.bmin, bounds.bmax, bounds.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangle: Out of memory.");
		return false;
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
						const std::vector<Vector3>& verts,
						const std::vector<Triangle>& tris, const unsigned char* triAreaIDs,
						rcHeightfield& heightfield, int flagMergeThreshold, int minIdx/* = 0*/, int maxIdx/* = 0 */)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// Rasterize the triangles.
	if (maxIdx == 0) maxIdx = (int)tris.size();
	auto& bounds = heightfield.bounds;
	const float inverseCellSize = 1.0f / bounds.cs;
	const float inverseCellHeight = 1.0f / bounds.ch;
	for (int triIndex = 0; triIndex < maxIdx; ++triIndex)
	{
		const auto& tri = tris[triIndex + minIdx];
		if (!rasterizeTri(verts[tri.v0], verts[tri.v1], verts[tri.v2], triAreaIDs[triIndex], heightfield, bounds.bmin, bounds.bmax, bounds.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

//bool rcRasterizeTriangles(rcContext* context,
//                          const float* verts, const int /*nv*/,
//                          const unsigned short* tris, const unsigned char* triAreaIDs, const int numTris,
//                          rcHeightfield& heightfield, const int flagMergeThreshold)
//{
//	rcAssert(context != NULL);
//
//	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
//
//	// Rasterize the triangles.
//	auto& bounds = heightfield.bounds;
//	const float inverseCellSize = 1.0f / bounds.cs;
//	const float inverseCellHeight = 1.0f / bounds.ch;
//	for (int triIndex = 0; triIndex < numTris; ++triIndex)
//	{
//		const float* v0 = &verts[tris[triIndex * 3 + 0] * 3];
//		const float* v1 = &verts[tris[triIndex * 3 + 1] * 3];
//		const float* v2 = &verts[tris[triIndex * 3 + 2] * 3];
//		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, bounds.bmin, bounds.bmax, bounds.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
//		{
//			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
//			return false;
//		}
//	}
//
//	return true;
//}
//
//bool rcRasterizeTriangles(rcContext* context,
//                          const float* verts, const unsigned char* triAreaIDs, const int numTris,
//                          rcHeightfield& heightfield, const int flagMergeThreshold)
//{
//	rcAssert(context != NULL);
//
//	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
//	
//	
//	// Rasterize the triangles.
//	auto& bounds = heightfield.bounds;
//	const float inverseCellSize = 1.0f / bounds.cs;
//	const float inverseCellHeight = 1.0f / bounds.ch;
//	for (int triIndex = 0; triIndex < numTris; ++triIndex)
//	{
//		const float* v0 = &verts[(triIndex * 3 + 0) * 3];
//		const float* v1 = &verts[(triIndex * 3 + 1) * 3];
//		const float* v2 = &verts[(triIndex * 3 + 2) * 3];
//		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, bounds.bmin, bounds.bmax, bounds.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
//		{
//			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
//			return false;
//		}
//	}
//
//	return true;
//}
