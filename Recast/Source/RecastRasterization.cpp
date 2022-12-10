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

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

static bool overlapBounds(const float* amin, const float* amax, const float* bmin, const float* bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

static rcSpan* allocSpan(rcHeightfield& hf)
{
	// If running out of memory, allocate new page and update the freelist.
	if (!hf.freelist || !hf.freelist->next)
	{
		// Create new page.
		// Allocate memory for the new pool.
		rcSpanPool* pool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (!pool) return 0;

		// Add the pool into the list of pools.
		pool->next = hf.pools;
		hf.pools = pool;
		// Add new items to the free list.
		rcSpan* freelist = hf.freelist;
		rcSpan* head = &pool->items[0];
		rcSpan* it = &pool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freelist;
			freelist = it;
		}
		while (it != head);
		hf.freelist = it;
	}
	
	// Pop item from in front of the free list.
	rcSpan* it = hf.freelist;
	hf.freelist = hf.freelist->next;
	return it;
}

static void freeSpan(rcHeightfield& hf, rcSpan* ptr)
{
	if (!ptr) return;
	// Add the node in front of the free list.
	ptr->next = hf.freelist;
	hf.freelist = ptr;
}

static bool addSpan(rcHeightfield& hf, const int x, const int y,
                    const unsigned short smin, const unsigned short smax,
                    const unsigned char area, const int flagMergeThr)
{
	int idx = x + y*hf.width;
	
	rcSpan* s = allocSpan(hf);
	if (!s)
		return false;
	s->smin = smin;
	s->smax = smax;
	s->area = area;
	s->next = 0;
	
	// Empty cell, add the first span.
	if (!hf.spans[idx])
	{
		hf.spans[idx] = s;
		return true;
	}
	rcSpan* prev = 0;
	rcSpan* cur = hf.spans[idx];
	
	// Insert and merge spans.
	while (cur)
	{
		if (cur->smin > s->smax)
		{
			// Current span is further than the new span, break.
			break;
		}
		else if (cur->smax < s->smin)
		{
			// Current span is before the new span advance.
			prev = cur;
			cur = cur->next;
		}
		else
		{
			// Merge spans.
			if (cur->smin < s->smin)
				s->smin = cur->smin;
			if (cur->smax > s->smax)
				s->smax = cur->smax;
			
			// Merge flags.
			if (rcAbs((int)s->smax - (int)cur->smax) <= flagMergeThr)
				s->area = rcMax(s->area, cur->area);
			
			// Remove current span.
			rcSpan* next = cur->next;
			freeSpan(hf, cur);
			if (prev)
				prev->next = next;
			else
				hf.spans[idx] = next;
			cur = next;
		}
	}
	
	// Insert new span.
	if (prev)
	{
		s->next = prev->next;
		prev->next = s;
	}
	else
	{
		s->next = hf.spans[idx];
		hf.spans[idx] = s;
	}

	return true;
}

bool rcAddSpan(rcContext* ctx, rcHeightfield& hf, const int x, const int y,
               const unsigned short smin, const unsigned short smax,
               const unsigned char area, const int flagMergeThr)
{
	rcAssert(ctx);

	if (!addSpan(hf, x, y, smin, smax, area, flagMergeThr))
	{
		ctx->log(RC_LOG_ERROR, "rcAddSpan: Out of memory.");
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
		// If the two vertices are on either side of the separating axis
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
			
			// add the i'th point to the right polygon. Do NOT add points that are on the dividing line
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
			// add the i'th point to the right polygon. Addition is done even for points on the dividing line
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

///	Rasterize a single triangle to the heightfield.
///
///	This code is extremely hot, so much care should be given to maintaining maximum perf here.
/// 
/// @param[in] 	v0				Triangle vertex 0
/// @param[in] 	v1				Triangle vertex 1
/// @param[in] 	v2				Triangle vertex 2
/// @param[in] 	area			The area ID to assign to the rasterized spans
/// @param[in] 	hf				Heightfield to rasterize into
/// @param[in] 	bmin			The min extents of the heightfield bounding box
/// @param[in] 	bmax			The max extents of the heightfield bounding box
/// @param[in] 	cs				The x and z axis size of a voxel in the heightfield
/// @param[in] 	ics				1 / cs
/// @param[in] 	ich				1 / ch
/// @param[in] 	flagMergeThr	The threshold in which area flags will be merged 
/// @returns true if the operation completes successfully.  false if there was an error adding spans to the heightfield.
static bool rasterizeTri(const float* v0, const float* v1, const float* v2,
                         const unsigned char area, rcHeightfield& hf,
                         const float* bmin, const float* bmax,
                         const float cs, const float ics, const float ich,
                         const int flagMergeThr)
{
	// Calculate the bounding box of the triangle.
	float tmin[3];
	float tmax[3];
	rcVcopy(tmin, v0);
	rcVcopy(tmax, v0);
	rcVmin(tmin, v1);
	rcVmin(tmin, v2);
	rcVmax(tmax, v1);
	rcVmax(tmax, v2);

	// If the triangle does not touch the bounding box of the heightfield, skip the triangle.
	if (!overlapBounds(bmin, bmax, tmin, tmax))
	{
		return true;
	}

	const int w = hf.width;
	const int h = hf.height;
	const float by = bmax[1] - bmin[1];

	// Calculate the footprint of the triangle on the grid's z-axis
	int z0 = (int)((tmin[2] - bmin[2]) * ics);
	int z1 = (int)((tmax[2] - bmin[2]) * ics);

	// use -1 rather than 0 to cut the polygon properly at the start of the tile
	z0 = rcClamp(z0, -1, h - 1);
	z1 = rcClamp(z1, 0, h - 1);

	// Clip the triangle into all grid cells it touches.
	float buf[7 * 3 * 4];
	float* in = buf;
	float* inrow = buf + 7 * 3;
	float* p1 = inrow + 7 * 3;
	float* p2 = p1 + 7 * 3;

	rcVcopy(&in[0], v0);
	rcVcopy(&in[1 * 3], v1);
	rcVcopy(&in[2 * 3], v2);
	int nvrow;
	int nvIn = 3;

	for (int z = z0; z <= z1; ++z)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float cz = bmin[2] + z * cs;
		dividePoly(in, nvIn, inrow, &nvrow, p1, &nvIn, cz + cs, RC_AXIS_Z);
		rcSwap(in, p1);
		
		if (nvrow < 3)
		{
			continue;
		}
		if (z < 0)
		{
			continue;
		}
		
		// find the horizontal bounds in the row
		float minX = inrow[0];
		float maxX = inrow[0];
		for (int i = 1; i < nvrow; ++i)
		{
			if (minX > inrow[i * 3])
			{
				minX = inrow[i * 3];
			}
			if (maxX < inrow[i * 3])
			{
				maxX = inrow[i * 3];
			}
		}
		int x0 = (int)((minX - bmin[0]) * ics);
		int x1 = (int)((maxX - bmin[0]) * ics);
		if (x1 < 0 || x0 >= w)
		{
			continue;
		}
		x0 = rcClamp(x0, -1, w - 1);
		x1 = rcClamp(x1, 0, w - 1);

		int nv;
		int nv2 = nvrow;

		for (int x = x0; x <= x1; ++x)
		{
			// Clip polygon to column. store the remaining polygon as well
			const float cx = bmin[0] + x * cs;
			dividePoly(inrow, nv2, p1, &nv, p2, &nv2, cx + cs, RC_AXIS_X);
			rcSwap(inrow, p2);
			
			if (nv < 3)
			{
				continue;
			}
			if (x < 0)
			{
				continue;
			}
			
			// Calculate min and max of the span.
			float smin = p1[1];
			float smax = p1[1];
			for (int i = 1; i < nv; ++i)
			{
				smin = rcMin(smin, p1[i * 3 + 1]);
				smax = rcMax(smax, p1[i * 3 + 1]);
			}
			smin -= bmin[1];
			smax -= bmin[1];
			
			// Skip the span if it is outside the heightfield bbox
			if (smax < 0.0f)
			{
				continue;
			}
			if (smin > by)
			{
				continue;
			}
			
			// Clamp the span to the heightfield bbox.
			if (smin < 0.0f)
			{
				smin = 0;
			}
			if (smax > by)
			{
				smax = by;
			}

			// Snap the span to the heightfield height grid.
			unsigned short ismin = (unsigned short)rcClamp((int)floorf(smin * ich), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short ismax = (unsigned short)rcClamp((int)ceilf(smax * ich), (int)ismin + 1, RC_SPAN_MAX_HEIGHT);

			if (!addSpan(hf, x, z, ismin, ismax, area, flagMergeThr))
			{
				return false;
			}
		}
	}

	return true;
}

bool rcRasterizeTriangle(rcContext* ctx, const float* v0, const float* v1, const float* v2,
                         const unsigned char area, rcHeightfield& solid,
                         const int flagMergeThr)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_RASTERIZE_TRIANGLES);

	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	if (!rasterizeTri(v0, v1, v2, area, solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr))
	{
		ctx->log(RC_LOG_ERROR, "rcRasterizeTriangle: Out of memory.");
		return false;
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const int /*nv*/,
                          const int* tris, const unsigned char* areas, const int nt,
                          rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[tris[i*3+0]*3];
		const float* v1 = &verts[tris[i*3+1]*3];
		const float* v2 = &verts[tris[i*3+2]*3];
		// Rasterize.
		if (!rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr))
		{
			ctx->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const int /*nv*/,
                          const unsigned short* tris, const unsigned char* areas, const int nt,
                          rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[tris[i*3+0]*3];
		const float* v1 = &verts[tris[i*3+1]*3];
		const float* v2 = &verts[tris[i*3+2]*3];
		// Rasterize.
		if (!rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr))
		{
			ctx->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const unsigned char* areas, const int nt,
                          rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[(i*3+0)*3];
		const float* v1 = &verts[(i*3+1)*3];
		const float* v2 = &verts[(i*3+2)*3];
		// Rasterize.
		if (!rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr))
		{
			ctx->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}
