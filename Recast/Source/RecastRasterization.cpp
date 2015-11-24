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
#include <float.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

inline bool overlapBounds(const float* amin, const float* amax, const float* bmin, const float* bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

inline bool overlapInterval(unsigned short amin, unsigned short amax,
							unsigned short bmin, unsigned short bmax)
{
	if (amax < bmin) return false;
	if (amin > bmax) return false;
	return true;
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

static void addSpan(rcHeightfield& hf, const int x, const int y,
					const unsigned short smin, const unsigned short smax,
					const unsigned char area, const int flagMergeThr)
{
	
	int idx = x + y*hf.width;
	
	rcSpan* s = allocSpan(hf);
	s->smin = smin;
	s->smax = smax;
	s->area = area;
	s->next = 0;
	
	// Empty cell, add the first span.
	if (!hf.spans[idx])
	{
		hf.spans[idx] = s;
		return;
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
}

/// @par
///
/// The span addition can be set to favor flags. If the span is merged to
/// another span and the new @p smax is within @p flagMergeThr units
/// from the existing span, the span flags are merged.
///
/// @see rcHeightfield, rcSpan.
void rcAddSpan(rcContext* /*ctx*/, rcHeightfield& hf, const int x, const int y,
			   const unsigned short smin, const unsigned short smax,
			   const unsigned char area, const int flagMergeThr)
{
//	rcAssert(ctx);
	addSpan(hf, x,y, smin, smax, area, flagMergeThr);
}

// divides a convex polygons into two convex polygons on both sides of a line
static void dividePoly(const float* in, int nin,
					  float* out1, int* nout1,
					  float* out2, int* nout2,
					  float x, int axis)
{
	float d[12];
	for (int i = 0; i < nin; ++i)
		d[i] = x - in[i*3+axis];

	int m = 0, n = 0;
	for (int i = 0, j = nin-1; i < nin; j=i, ++i)
	{
		bool ina = d[j] >= 0;
		bool inb = d[i] >= 0;
		if (ina != inb)
		{
			float s = d[j] / (d[j] - d[i]);
			out1[m*3+0] = in[j*3+0] + (in[i*3+0] - in[j*3+0])*s;
			out1[m*3+1] = in[j*3+1] + (in[i*3+1] - in[j*3+1])*s;
			out1[m*3+2] = in[j*3+2] + (in[i*3+2] - in[j*3+2])*s;
			rcVcopy(out2 + n*3, out1 + m*3);
			m++;
			n++;
			// add the i'th point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (d[i] > 0)
			{
				rcVcopy(out1 + m*3, in + i*3);
				m++;
			}
			else if (d[i] < 0)
			{
				rcVcopy(out2 + n*3, in + i*3);
				n++;
			}
		}
		else // same side
		{
			// add the i'th point to the right polygon. Addition is done even for points on the dividing line
			if (d[i] >= 0)
			{
				rcVcopy(out1 + m*3, in + i*3);
				m++;
				if (d[i] != 0)
					continue;
			}
			rcVcopy(out2 + n*3, in + i*3);
			n++;
		}
	}

	*nout1 = m;
	*nout2 = n;
}



static void rasterizeTri(const float* v0, const float* v1, const float* v2,
						 const unsigned char area, rcHeightfield& hf,
						 const float* bmin, const float* bmax,
						 const float cs, const float ics, const float ich,
						 const int flagMergeThr)
{
	const int w = hf.width;
	const int h = hf.height;
	float tmin[3], tmax[3];
	const float by = bmax[1] - bmin[1];
	
	// Calculate the bounding box of the triangle.
	rcVcopy(tmin, v0);
	rcVcopy(tmax, v0);
	rcVmin(tmin, v1);
	rcVmin(tmin, v2);
	rcVmax(tmax, v1);
	rcVmax(tmax, v2);
	
	// If the triangle does not touch the bbox of the heightfield, skip the triagle.
	if (!overlapBounds(bmin, bmax, tmin, tmax))
		return;
	
	// Calculate the footprint of the triangle on the grid's y-axis
	int y0 = (int)((tmin[2] - bmin[2])*ics);
	int y1 = (int)((tmax[2] - bmin[2])*ics);
	y0 = rcClamp(y0, 0, h-1);
	y1 = rcClamp(y1, 0, h-1);
	
	// Clip the triangle into all grid cells it touches.
	float buf[7*3*4];
	float *in = buf, *inrow = buf+7*3, *p1 = inrow+7*3, *p2 = p1+7*3;

	rcVcopy(&in[0], v0);
	rcVcopy(&in[1*3], v1);
	rcVcopy(&in[2*3], v2);
	int nvrow, nvIn = 3;
	
	for (int y = y0; y <= y1; ++y)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float cz = bmin[2] + y*cs;
		dividePoly(in, nvIn, inrow, &nvrow, p1, &nvIn, cz+cs, 2);
		rcSwap(in, p1);
		if (nvrow < 3) continue;
		
		// find the horizontal bounds in the row
		float minX = inrow[0], maxX = inrow[0];
		for (int i=1; i<nvrow; ++i)
		{
			if (minX > inrow[i*3])	minX = inrow[i*3];
			if (maxX < inrow[i*3])	maxX = inrow[i*3];
		}
		int x0 = (int)((minX - bmin[0])*ics);
		int x1 = (int)((maxX - bmin[0])*ics);
		x0 = rcClamp(x0, 0, w-1);
		x1 = rcClamp(x1, 0, w-1);

		int nv, nv2 = nvrow;

		for (int x = x0; x <= x1; ++x)
		{
			// Clip polygon to column. store the remaining polygon as well
			const float cx = bmin[0] + x*cs;
			dividePoly(inrow, nv2, p1, &nv, p2, &nv2, cx+cs, 0);
			rcSwap(inrow, p2);
			if (nv < 3) continue;
			
			// Calculate min and max of the span.
			float smin = p1[1], smax = p1[1];
			for (int i = 1; i < nv; ++i)
			{
				smin = rcMin(smin, p1[i*3+1]);
				smax = rcMax(smax, p1[i*3+1]);
			}
			smin -= bmin[1];
			smax -= bmin[1];
			// Skip the span if it is outside the heightfield bbox
			if (smax < 0.0f) continue;
			if (smin > by) continue;
			// Clamp the span to the heightfield bbox.
			if (smin < 0.0f) smin = 0;
			if (smax > by) smax = by;
			
			// Snap the span to the heightfield height grid.
			unsigned short ismin = (unsigned short)rcClamp((int)floorf(smin * ich), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short ismax = (unsigned short)rcClamp((int)ceilf(smax * ich), (int)ismin+1, RC_SPAN_MAX_HEIGHT);
			
			addSpan(hf, x, y, ismin, ismax, area, flagMergeThr);
		}
	}
}

static bool isHullTri(const float* vertices, const int numVertices, const int* triIndices)
{
	// Classify all the points in the hull against the plane of the triangle
	// If they're not all on the same side, this triangle is not part of the hull
	float va[3], vb[3];
	rcVcopy(va, &vertices[triIndices[1]*3]);
	rcVcopy(vb, &vertices[triIndices[2]*3]);
	rcVsub(va, va, &vertices[triIndices[0]*3]);
	rcVsub(vb, vb, &vertices[triIndices[0]*3]);
	
	float normal[3];
	rcVcross(normal, va, vb);
	
	bool anyFront = false;
	bool anyBack = false;
	
	for (int i = 0; i < numVertices; ++i)
	{
		float v[3];
		rcVsub(v, &vertices[i*3], &vertices[triIndices[0]*3]);
		float dot = rcVdot(v, normal);
		if (dot > 0.001f)
			anyFront = true;
		if (dot < -0.001f)
			anyBack = true;
		
		if (anyFront && anyBack)
			return false;
	}
	
	return true;
}

static void rasterizeFilledConvexVolume(rcContext* ctx, const float* vertices, const int numVertices,
									  const unsigned char area, rcHeightfield& hf,
									  const float* bmin, const float* bmax,
									  const float cs, const float ics, const float ch, const float ich,
									  const int flagMergeThr)
{
	if (numVertices < 3) return;
	
	const int w = hf.width;
	const int h = hf.height;
	
	// Calculate the bounding box of all the points
	float tmin[3], tmax[3];
	rcVcopy(tmin, &vertices[0]);
	rcVcopy(tmax, &vertices[0]);
	for (int i = 1; i < numVertices; ++i)
	{
		rcVmin(tmin, &vertices[i*3]);
		rcVmax(tmax, &vertices[i*3]);
	}
	
	// If the OBB does not touch the bbox of the heightfield, skip it.
	if (!overlapBounds(bmin, bmax, tmin, tmax))
		return;

	// Calculate the footprint of the hull on the grid
	int y0 = (int)((tmin[2] - bmin[2])*ics);
	int y1 = (int)((tmax[2] - bmin[2])*ics);
	y0 = rcClamp(y0, 0, h-1);
	y1 = rcClamp(y1, 0, h-1);
	
	int x0 = (int)((tmin[0] - bmin[0])*ics);
	int x1 = (int)((tmax[0] - bmin[0])*ics);
	x0 = rcClamp(x0, 0, w-1);
	x1 = rcClamp(x1, 0, w-1);
	
	// Allocate a temporary heightfield
	rcHeightfield* tempField = rcAllocHeightfield();
	rcCreateHeightfield(ctx, *tempField, x1-x0+1, y1-y0+1, tmin, tmax, cs, ch);
	
	// Rasterize every triple of vertices as a triangle into the temp heightfield
	int vIds[3];
	for(vIds[0] = 0; vIds[0] < numVertices; ++vIds[0])
	{
		for (vIds[1] = vIds[0] + 1; vIds[1] < numVertices; ++vIds[1])
		{
			for (vIds[2] = vIds[1] + 1; vIds[2] < numVertices; ++vIds[2])
			{
				if (!isHullTri(vertices, numVertices, vIds)) continue;
				
				rasterizeTri(&vertices[vIds[0]*3], &vertices[vIds[1]*3], &vertices[vIds[2]*3], area, *tempField, tmin, tmax, cs, ics, ich, flagMergeThr);
			}
		}
	}
	
	// Combine all spans in the temp heightfield to fill in gaps, and merge into master heightfield
	for (int y = 0; y < tempField->height; ++y)
	{
		for (int x = 0; x < tempField->width; ++x)
		{
			rcSpan* firstSpan = tempField->spans[y * tempField->width + x];
			if (firstSpan == NULL) continue;
			
			rcSpan* span = firstSpan;
			while (span->next)
			{
				span = span->next;
				firstSpan->smax = span->smax;
			}
			
			rcAddSpan(ctx, hf, x0 + x, y0 + y, firstSpan->smin, firstSpan->smax, area, flagMergeThr);
		}
	}
	
	rcFreeHeightField(tempField);
}

/// @par
///
/// No spans will be added if the triangle does not overlap the heightfield grid.
///
/// @see rcHeightfield
void rcRasterizeTriangle(rcContext* ctx, const float* v0, const float* v1, const float* v2,
						 const unsigned char area, rcHeightfield& solid,
						 const int flagMergeThr)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);

	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	rasterizeTri(v0, v1, v2, area, solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);

	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
}

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const int /*nv*/,
						  const int* tris, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[tris[i*3+0]*3];
		const float* v1 = &verts[tris[i*3+1]*3];
		const float* v2 = &verts[tris[i*3+2]*3];
		// Rasterize.
		rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);
	}
	
	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
}

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const int /*nv*/,
						  const unsigned short* tris, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[tris[i*3+0]*3];
		const float* v1 = &verts[tris[i*3+1]*3];
		const float* v2 = &verts[tris[i*3+2]*3];
		// Rasterize.
		rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);
	}
	
	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
}

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[(i*3+0)*3];
		const float* v1 = &verts[(i*3+1)*3];
		const float* v2 = &verts[(i*3+2)*3];
		// Rasterize.
		rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);
	}
	
	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
}

void rcRasterizeFilledConvexVolume(rcContext* ctx, const float* vertices, const int numVertices, const unsigned char area, rcHeightfield& solid, int flagMergeThr)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_RASTERIZE_CONVEX_VOLUMES);

	rasterizeFilledConvexVolume(ctx, vertices, numVertices, area, solid, solid.bmin, solid.bmax, solid.cs, 1.0f/solid.cs, solid.ch, 1.0f/solid.ch, flagMergeThr);
	
	ctx->stopTimer(RC_TIMER_RASTERIZE_CONVEX_VOLUMES);
}

void rcRasterizeFilledOBB(rcContext* ctx, const float* center, const float* extent, const float* forward, const float* up,
					const unsigned char area, rcHeightfield& solid, int flagMergeThr)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_RASTERIZE_CONVEX_VOLUMES);
	
	float right[3];
	rcVcross(right, forward, up);
	rcVnormalize(right);
	
	float trX[3] = {right[0], up[0], forward[0]};
	float trY[3] = {right[1], up[1], forward[1]};
	float trZ[3] = {right[2], up[2], forward[2]};
	
	// Generate the vertices for the OBB
	float vertices[3*8];
	for (int i = 0; i < 8; ++i)
	{
		float pt[3] = { (i & 1) ? extent[0] : -extent[0],
						(i & 2) ? extent[1] : -extent[1],
						(i & 4) ? extent[2] : -extent[2] };
		vertices[i * 3 + 0] = rcVdot(pt, trX);
		vertices[i * 3 + 1] = rcVdot(pt, trY);
		vertices[i * 3 + 2] = rcVdot(pt, trZ);
		rcVadd(&vertices[i*3], &vertices[i*3], center);
	}
	
	rasterizeFilledConvexVolume(ctx, vertices, 8, area, solid, solid.bmin, solid.bmax, solid.cs, 1.0f/solid.cs, solid.ch, 1.0f/solid.ch, flagMergeThr);
	
	ctx->stopTimer(RC_TIMER_RASTERIZE_CONVEX_VOLUMES);
}