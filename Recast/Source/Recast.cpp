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

#include <float.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

float rcSqrt(float x)
{
	return sqrtf(x);
}

/// @class rcContext
/// @par
///
/// This class does not provide logging or timer functionality on its 
/// own.  Both must be provided by a concrete implementation 
/// by overriding the protected member functions.  Also, this class does not 
/// provide an interface for extracting log messages. (Only adding them.) 
/// So concrete implementations must provide one.
///
/// If no logging or timers are required, just pass an instance of this 
/// class through the Recast build process.
///

/// @par
///
/// Example:
/// @code
/// // Where ctx is an instance of rcContext and filepath is a char array.
/// ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath);
/// @endcode
void rcContext::log(const rcLogCategory category, const char* format, ...)
{
	if (!m_logEnabled)
		return;
	static const int MSG_SIZE = 512;
	char msg[MSG_SIZE];
	va_list ap;
	va_start(ap, format);
	int len = vsnprintf(msg, MSG_SIZE, format, ap);
	if (len >= MSG_SIZE)
	{
		len = MSG_SIZE-1;
		msg[MSG_SIZE-1] = '\0';
	}
	va_end(ap);
	doLog(category, msg, len);
}

rcHeightfield* rcAllocHeightfield()
{
	rcHeightfield* hf = (rcHeightfield*)rcAlloc(sizeof(rcHeightfield), RC_ALLOC_PERM);
	memset(hf, 0, sizeof(rcHeightfield));
	return hf;
}

void rcFreeHeightField(rcHeightfield* hf)
{
	if (!hf) return;
	// Delete span array.
	rcFree(hf->spans);
	// Delete span pools.
	while (hf->pools)
	{
		rcSpanPool* next = hf->pools->next;
		rcFree(hf->pools);
		hf->pools = next;
	}
	rcFree(hf);
}

rcCompactHeightfield* rcAllocCompactHeightfield()
{
	rcCompactHeightfield* chf = (rcCompactHeightfield*)rcAlloc(sizeof(rcCompactHeightfield), RC_ALLOC_PERM);
	memset(chf, 0, sizeof(rcCompactHeightfield));
	return chf;
}

void rcFreeCompactHeightfield(rcCompactHeightfield* chf)
{
	if (!chf) return;
	rcFree(chf->cells);
	rcFree(chf->spans);
	rcFree(chf->dist);
	rcFree(chf->areas);
	rcFree(chf);
}


rcHeightfieldLayerSet* rcAllocHeightfieldLayerSet()
{
	rcHeightfieldLayerSet* lset = (rcHeightfieldLayerSet*)rcAlloc(sizeof(rcHeightfieldLayerSet), RC_ALLOC_PERM);
	memset(lset, 0, sizeof(rcHeightfieldLayerSet));
	return lset;
}

void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* lset)
{
	if (!lset) return;
	for (int i = 0; i < lset->nlayers; ++i)
	{
		rcFree(lset->layers[i].heights);
		rcFree(lset->layers[i].areas);
		rcFree(lset->layers[i].cons);
	}
	rcFree(lset->layers);
	rcFree(lset);
}


rcContourSet* rcAllocContourSet()
{
	rcContourSet* cset = (rcContourSet*)rcAlloc(sizeof(rcContourSet), RC_ALLOC_PERM);
	memset(cset, 0, sizeof(rcContourSet));
	return cset;
}

void rcFreeContourSet(rcContourSet* cset)
{
	if (!cset) return;
	for (int i = 0; i < cset->nconts; ++i)
	{
		rcFree(cset->conts[i].verts);
		rcFree(cset->conts[i].rverts);
	}
	rcFree(cset->conts);
	rcFree(cset);
}

rcPolyMesh* rcAllocPolyMesh()
{
	rcPolyMesh* pmesh = (rcPolyMesh*)rcAlloc(sizeof(rcPolyMesh), RC_ALLOC_PERM);
	memset(pmesh, 0, sizeof(rcPolyMesh));
	return pmesh;
}

void rcFreePolyMesh(rcPolyMesh* pmesh)
{
	if (!pmesh) return;
	rcFree(pmesh->verts);
	rcFree(pmesh->polys);
	rcFree(pmesh->regs);
	rcFree(pmesh->flags);
	rcFree(pmesh->areas);
	rcFree(pmesh);
}

rcPolyMeshDetail* rcAllocPolyMeshDetail()
{
	rcPolyMeshDetail* dmesh = (rcPolyMeshDetail*)rcAlloc(sizeof(rcPolyMeshDetail), RC_ALLOC_PERM);
	memset(dmesh, 0, sizeof(rcPolyMeshDetail));
	return dmesh;
}

void rcFreePolyMeshDetail(rcPolyMeshDetail* dmesh)
{
	if (!dmesh) return;
	rcFree(dmesh->meshes);
	rcFree(dmesh->verts);
	rcFree(dmesh->tris);
	rcFree(dmesh);
}

void rcCalcBounds(const float* verts, int nv, float* bmin, float* bmax)
{
	// Calculate bounding box.
	rcVcopy(bmin, verts);
	rcVcopy(bmax, verts);
	for (int i = 1; i < nv; ++i)
	{
		const float* v = &verts[i*3];
		rcVmin(bmin, v);
		rcVmax(bmax, v);
	}
}

void rcCalcGridSize(const float* bmin, const float* bmax, float cs, int* w, int* h)
{
	*w = (int)((bmax[0] - bmin[0])/cs+0.5f);
	*h = (int)((bmax[2] - bmin[2])/cs+0.5f);
}

/// @par
///
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcAllocHeightfield, rcHeightfield 
bool rcCreateHeightfield(rcContext* ctx, rcHeightfield& hf, int width, int height,
						 const float* bmin, const float* bmax,
						 float cs, float ch)
{
	rcIgnoreUnused(ctx);
	
	hf.width = width;
	hf.height = height;
	rcVcopy(hf.bmin, bmin);
	rcVcopy(hf.bmax, bmax);
	hf.cs = cs;
	hf.ch = ch;
	hf.spans = (rcSpan**)rcAlloc(sizeof(rcSpan*)*hf.width*hf.height, RC_ALLOC_PERM);
	if (!hf.spans)
		return false;
	memset(hf.spans, 0, sizeof(rcSpan*)*hf.width*hf.height);
	return true;
}

static void calcTriNormal(const float* v0, const float* v1, const float* v2, float* norm)
{
	float e0[3], e1[3];
	rcVsub(e0, v1, v0);
	rcVsub(e1, v2, v0);
	rcVcross(norm, e0, e1);
	rcVnormalize(norm);
}

/// @par
///
/// Only sets the area id's for the walkable triangles.  Does not alter the
/// area id's for unwalkable triangles.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
void rcMarkWalkableTriangles(rcContext* ctx, const float walkableSlopeAngle,
							 const float* verts, int /*nv*/,
							 const int* tris, int nt,
							 unsigned char* areas)
{
	rcIgnoreUnused(ctx);
	
	const float walkableThr = cosf(walkableSlopeAngle/180.0f*RC_PI);

	float norm[3];
	
	for (int i = 0; i < nt; ++i)
	{
		const int* tri = &tris[i*3];
		calcTriNormal(&verts[tri[0]*3], &verts[tri[1]*3], &verts[tri[2]*3], norm);
		// Check if the face is walkable.
		if (norm[1] > walkableThr)
			areas[i] = RC_WALKABLE_AREA;
	}
}

/// @par
///
/// Only sets the area id's for the unwalkable triangles.  Does not alter the
/// area id's for walkable triangles.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
void rcClearUnwalkableTriangles(rcContext* ctx, const float walkableSlopeAngle,
								const float* verts, int /*nv*/,
								const int* tris, int nt,
								unsigned char* areas)
{
	rcIgnoreUnused(ctx);
	
	const float walkableThr = cosf(walkableSlopeAngle/180.0f*RC_PI);
	
	float norm[3];
	
	for (int i = 0; i < nt; ++i)
	{
		const int* tri = &tris[i*3];
		calcTriNormal(&verts[tri[0]*3], &verts[tri[1]*3], &verts[tri[2]*3], norm);
		// Check if the face is walkable.
		if (norm[1] <= walkableThr)
			areas[i] = RC_NULL_AREA;
	}
}

int rcGetHeightFieldSpanCount(rcContext* ctx, rcHeightfield& hf)
{
	rcIgnoreUnused(ctx);
	
	const int w = hf.width;
	const int h = hf.height;
	int spanCount = 0;
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			for (rcSpan* s = hf.spans[x + y*w]; s; s = s->next)
			{
				if (s->area != RC_NULL_AREA)
					spanCount++;
			}
		}
	}
	return spanCount;
}


/*
static int getHeightfieldMemoryUsage(const rcHeightfield& hf)
{
	int size = 0;
	size += sizeof(hf);
	size += hf.width * hf.height * sizeof(rcSpan*);
	
	rcSpanPool* pool = hf.pools;
	while (pool)
	{
		size += (sizeof(rcSpanPool) - sizeof(rcSpan)) + sizeof(rcSpan)*rcSpanPool::RC_SPANS_PER_POOL;
		pool = pool->next;
	}
	return size;
}

static int getCompactHeightFieldMemoryusage(const rcCompactHeightfield& chf)
{
	int size = 0;
	size += sizeof(rcCompactHeightfield);
	size += sizeof(rcCompactSpan) * chf.spanCount;
	size += sizeof(rcCompactCell) * chf.width * chf.height;
	return size;
}
*/
