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
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"


static const int RC_MAX_LAYERS = RC_NOT_CONNECTED;
static const int RC_MAX_NEIS = 16;

struct rcLayerRegion
{
	unsigned char layers[RC_MAX_LAYERS];
	unsigned char neis[RC_MAX_NEIS];
	
	unsigned short ymin, ymax;
	
	unsigned short count;
	
	unsigned char layerId;
	unsigned char nlayers;
	unsigned char nneis;
	
	unsigned char start;
};


static void addUnique(unsigned char* a, unsigned char& an, unsigned char v)
{
	const int n = (int)an;
	for (int i = 0; i < n; ++i)
		if (a[i] == v)
			return;
	a[an] = v;
	an++;
}

static void addUniqueLast(unsigned char* a, unsigned char& an, unsigned char v)
{
	const int n = (int)an;
	if (n > 0 && a[n-1] == v) return;
	a[an] = v;
	an++;
}

static bool contains(const unsigned char* a, const unsigned char an, const unsigned char v)
{
	const int n = (int)an;
	for (int i = 0; i < n; ++i)
		if (a[i] == v)
			return true;
	return false;
}

inline bool overlapRange(const unsigned short amin, const unsigned short amax,
						 const unsigned short bmin, const unsigned short bmax)
{
	return (amin > bmax || amax < bmin) ? false : true;
}


struct rcLayerSweepSpan
{
	unsigned short ns;	// number samples
	unsigned char id;	// region id
	unsigned char nei;	// neighbour id
};


rcHeightfieldLayerPortal* allocPortal(rcHeightfieldLayerPortal** portals, int& nportals, int& cportals)
{
	if (nportals+1 >= cportals)
	{
		cportals *= 2;
		rcHeightfieldLayerPortal* np = (rcHeightfieldLayerPortal*)rcAlloc(sizeof(rcHeightfieldLayerPortal)*cportals,RC_ALLOC_PERM);
		if (!np)
			return 0;
		if (nportals > 0)
			memcpy(np,*portals,sizeof(rcHeightfieldLayerPortal)*nportals);
		rcFree(*portals);
		*portals = np;
	}
	nportals++;
	return &(*portals)[nportals-1];
}


bool rcBuildHeightfieldLayers(rcContext* ctx, rcCompactHeightfield& chf,
							  const int borderSize, const int walkableHeight,
							  rcHeightfieldLayerSet& lset)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_BUILD_LAYERS);
	
	const int w = chf.width;
	const int h = chf.height;
	
	rcScopedDelete<unsigned char> srcReg = (unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_TEMP);
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'srcReg' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0xff,sizeof(unsigned char)*chf.spanCount);
	
	const int nsweeps = chf.width;
	rcScopedDelete<rcLayerSweepSpan> sweeps = (rcLayerSweepSpan*)rcAlloc(sizeof(rcLayerSweepSpan)*nsweeps, RC_ALLOC_TEMP);
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// Partition walkable area into monotone regions.
	int prevCount[256];
	unsigned char regId = 0;

//	for (int y = 0; y < h; ++y)
	for (int y = borderSize; y < h-borderSize; ++y)
	{
		memset(prevCount,0,sizeof(int)*regId);
		unsigned char sweepId = 0;
		
//		for (int x = 0; x < w; ++x)
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;

				unsigned char sid = 0xff;

				// -x
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					if (chf.areas[ai] != RC_NULL_AREA && srcReg[ai] != 0xff)
						sid = srcReg[ai];
				}
				
				if (sid == 0xff)
				{
					sid = sweepId++;
					sweeps[sid].nei = 0xff;
					sweeps[sid].ns = 0;
				}
				
				// -y
				if (rcGetCon(s,3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					const unsigned char nr = srcReg[ai];
					if (nr != 0xff)
					{
						// Set neighbour when first valid neighbour is encoutered.
						if (sweeps[sid].ns == 0)
							sweeps[sid].nei = nr;
						
						if (sweeps[sid].nei == nr)
						{
							// Update existing neighbour
							sweeps[sid].ns++;
							prevCount[nr]++;
						}
						else
						{
							// This is hit if there is nore than one neighbour.
							// Invalidate the neighbour.
							sweeps[sid].nei = 0xff;
						}
					}
				}
				
				srcReg[i] = sid;
			}
		}
		
		// Create unique ID.
		for (int i = 0; i < sweepId; ++i)
		{
			// If the neighbour is set and there is only one continuous connection to it,
			// the sweep will be merged with the previous one, else new region is created.
			if (sweeps[i].nei != 0xff && prevCount[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				if (regId == 255)
				{
					ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Region ID overflow.");
					return false;
				}
				sweeps[i].id = regId++;
			}
		}
		
		// Remap local sweep ids to region ids.
//		for (int x = 0; x < w; ++x)
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] != 0xff)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}

	// Allocate and init layer regions.
	const int nregs = (int)regId;
	rcScopedDelete<rcLayerRegion> regs = (rcLayerRegion*)rcAlloc(sizeof(rcLayerRegion)*nregs, RC_ALLOC_TEMP);
	if (!regs)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'regs' (%d).", nregs);
		return false;
	}
	memset(regs, 0, sizeof(rcLayerRegion)*nregs);
	for (int i = 0; i < nregs; ++i)
	{
		regs[i].layerId = 0xff;
		regs[i].ymin = 0xffff;
		regs[i].ymax = 0;
	}
	
	// Find region neighbours and overlapping regions.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			unsigned char lregs[RC_MAX_LAYERS];
			int nlregs = 0;
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned char ri = srcReg[i];
				if (ri == 0xff) continue;
				
				regs[ri].ymin = rcMin(regs[ri].ymin, s.y);
				regs[ri].ymax = rcMax(regs[ri].ymax, s.y);
				
				// Collect all region layers.
				if (nlregs < RC_MAX_LAYERS)
					lregs[nlregs++] = ri;
				
				// Update neighbours
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						const unsigned char rai = srcReg[ai];
						if (rai != 0xff && rai != ri)
							addUnique(regs[ri].neis, regs[ri].nneis, rai);
					}
				}
				
			}
			
			// Update overlapping regions.
			for (int i = 0; i < nlregs-1; ++i)
			{
				for (int j = i+1; j < nlregs; ++j)
				{
					if (lregs[i] != lregs[j])
					{
						rcLayerRegion& ri = regs[lregs[i]];
						rcLayerRegion& rj = regs[lregs[j]];
						addUnique(ri.layers, ri.nlayers, lregs[j]);
						addUnique(rj.layers, rj.nlayers, lregs[i]);
					}
				}
			}
			
		}
	}
	
	// Create 2D layers from regions.
	unsigned char layerId = 0;
	
	static const int MAX_STACK = 64;
	unsigned char stack[MAX_STACK];
	int nstack = 0;
	
	for (int i = 0; i < nregs; ++i)
	{
		rcLayerRegion& root = regs[i];
		// Skip alreadu visited.
		if (root.layerId != 0xff)
			continue;

		// Start search.
		root.layerId = layerId;
		root.start = 1;
		
		nstack = 0;
		stack[nstack++] = (unsigned char)i;
		
		while (nstack)
		{
			// Pop front
			rcLayerRegion& reg = regs[stack[0]];
			nstack--;
			for (int j = 0; j < nstack; ++j)
				stack[j] = stack[j+1];
			
			const int nneis = (int)reg.nneis;
			for (int j = 0; j < nneis; ++j)
			{
				const unsigned char nei = reg.neis[j];
				// Skip already visited.
				if (regs[nei].layerId != 0xff)
					continue;
				// Skip if the neighbour is overlapping root region.
				if (contains(root.layers, root.nlayers, nei))
					continue;
				
				if (nstack < MAX_STACK)
				{
					// Deepen
					stack[nstack++] = (unsigned char)nei;
					
					rcLayerRegion& regn = regs[nei];
					// Mark layer id
					regn.layerId = layerId;
					// Merge current layers to root.
					for (int k = 0; k < regn.nlayers; ++k)
						addUnique(root.layers, root.nlayers, regn.layers[k]);
					root.ymin = rcMin(root.ymin, regn.ymin);
					root.ymax = rcMax(root.ymax, regn.ymax);
				}
			}
		}
		
		layerId++;
	}
	
	// Merge non-overlapping regions that are close in height.
	const int mergeHeight = walkableHeight * 4;
	
	for (int i = 0; i < nregs; ++i)
	{
		rcLayerRegion& ri = regs[i];
		if (!ri.start) continue;
		
		unsigned char newId = ri.layerId;
		
		for (;;)
		{
			unsigned char oldId = 0xff;
			
			for (int j = 0; j < nregs; ++j)
			{
				if (i == j) continue;
				rcLayerRegion& rj = regs[j];
				if (!rj.start) continue;
				
				// Skip if teh regions are not close to each other.
				if (!overlapRange(ri.ymin,ri.ymax+mergeHeight, rj.ymin,rj.ymax+mergeHeight))
					continue;
				
				// Make sure that there is no overlap when mergin 'ri' and 'rj'.
				bool overlap = false;
				// Iterate over all regions which have the same layerId as 'rj'
				for (int k = 0; k < nregs; ++k)
				{
					if (regs[k].layerId != rj.layerId)
						continue;
					// Check if region 'k' is overlapping region 'ri'
					// Index to 'regs' is the same as region id.
					if (contains(ri.layers,ri.nlayers, (unsigned char)k))
					{
						overlap = true;
						break;
					}
				}
				// Cannot merge of regions overlap.
				if (overlap)
					continue;
				
				// Can merge i and j.
				oldId = rj.layerId;
				break;
			}
			
			// Could not find anything to merge with, stop.
			if (oldId == 0xff)
				break;
			
			// Merge
			for (int j = 0; j < nregs; ++j)
			{
				rcLayerRegion& rj = regs[j];
				if (rj.layerId == oldId)
				{
					rj.start = 0;
					// Remap layerIds.
					rj.layerId = newId;
					// Add overlaid layers from 'rj' to 'ri'.
					for (int k = 0; k < rj.nlayers; ++k)
						addUnique(ri.layers, ri.nlayers, rj.layers[k]);
					// Update heigh bounds.
					ri.ymin = rcMin(ri.ymin, rj.ymin);
					ri.ymax = rcMax(ri.ymax, rj.ymax);
				}
			}
		}
	}
	
	// Compact layerIds
	unsigned char remap[256];
	memset(remap, 0, 256);

	// Find number of unique layers.
	layerId = 0;
	for (int i = 0; i < nregs; ++i)
		remap[regs[i].layerId] = 1;
	for (int i = 0; i < 256; ++i)
	{
		if (remap[i])
			remap[i] = layerId++;
		else
			remap[i] = 0xff;
	}
	// Remap ids.
	for (int i = 0; i < nregs; ++i)
		regs[i].layerId = remap[regs[i].layerId];
	
	// No layers, return empty.
	if (layerId == 0)
	{
		ctx->stopTimer(RC_TIMER_BUILD_REGIONS);
		return true;
	}
	
	// Create layers.
	rcAssert(lset.layers == 0);
	
	const int lw = w - borderSize*2;
	const int lh = h - borderSize*2;

	// Build contracted bbox for layers.
	float bmin[3], bmax[3];
	rcVcopy(bmin, chf.bmin);
	rcVcopy(bmax, chf.bmax);
	bmin[0] += borderSize*chf.cs;
	bmin[2] += borderSize*chf.cs;
	bmax[0] -= borderSize*chf.cs;
	bmax[2] -= borderSize*chf.cs;
	
	lset.nlayers = (int)layerId;
	
	lset.layers = (rcHeightfieldLayer*)rcAlloc(sizeof(rcHeightfieldLayer)*lset.nlayers, RC_ALLOC_PERM);
	if (!lset.layers)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'layers' (%d).", lset.nlayers);
		return false;
	}
	memset(lset.layers, 0, sizeof(rcHeightfieldLayer)*lset.nlayers);
	
	rcScopedDelete<unsigned char> cons = (unsigned char*)rcAlloc(sizeof(unsigned char)*lw*lh, RC_ALLOC_TEMP);
	if (!cons)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'con' (%d).", lw*lh);
		return false;
	}

	
	// Store layers.
	for (int i = 0; i < lset.nlayers; ++i)
	{
		unsigned char curId = (unsigned char)i;
		
		// Allocate memory for the current layer.
		rcHeightfieldLayer* layer = &lset.layers[i];
		
		layer->width = lw;
		layer->height = lh;
		layer->cs = chf.cs;
		layer->ch = chf.ch;
		// TODO: Should this be local bbox instead?
		rcVcopy(layer->bmin, bmin);
		rcVcopy(layer->bmax, bmax);

		layer->heights = (unsigned short*)rcAlloc(sizeof(unsigned short)*lw*lh, RC_ALLOC_PERM);
		if (!layer->heights)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'heights' (%d).", w*h);
			return false;
		}
		memset(layer->heights, 0xff, sizeof(unsigned short)*lw*lh);
		
		layer->areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*lw*lh, RC_ALLOC_PERM);
		if (!layer->areas)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'areas' (%d).", w*h);
			return false;
		}
		memset(layer->areas, RC_NULL_AREA, sizeof(unsigned char)*lw*lh);
		
		memset(cons, 0, sizeof(unsigned char)*lw*lh);
		
		// Find layer height bounds.
		for (int j = 0; j < nregs; ++j)
		{
			if (regs[j].start && regs[j].layerId == curId)
			{
				layer->ymin = regs[j].ymin;
				layer->ymax = regs[j].ymax;
			}
		}
		
		// Copy height and area from compact heighfield. 
		for (int y = 0; y < lh; ++y)
		{
			for (int x = 0; x < lw; ++x)
			{
				const int cx = borderSize+x;
				const int cy = borderSize+y;
				const rcCompactCell& c = chf.cells[cx+cy*w];
				for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
				{
					const rcCompactSpan& s = chf.spans[i];
					// Skip unassigned regions.
					if (srcReg[i] == 0xff)
						continue;
					// Skip of does nto belong to current layer.
					unsigned char lid = regs[srcReg[i]].layerId;
					if (lid != curId)
						continue;
					// Store height and area type.
					const int idx = x+y*lw;
					layer->heights[idx] = s.y;
					layer->areas[idx] = chf.areas[i];
					
					// Check connection.
					unsigned char con = 0;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int ax = cx + rcGetDirOffsetX(dir);
							const int ay = cy + rcGetDirOffsetY(dir);
							const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
							unsigned char alid = srcReg[ai] != 0xff ? regs[srcReg[ai]].layerId : 0xff;
							if (chf.areas[ai] != RC_NULL_AREA && lid != alid)
								con |= (unsigned char)(1<<dir);
						}
					}
					cons[idx] = con;
				}
			}
		}
		
		// Create portals
		int cportals = 6;
		layer->portals = (rcHeightfieldLayerPortal*)rcAlloc(sizeof(rcHeightfieldLayerPortal)*cportals,RC_ALLOC_PERM);
		if (!layer->portals)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'portals' (%d).", cportals);
			return false;
		}
		layer->nportals = 0;
		
		// Directions same as rcGetCon()
		const unsigned char XM = 1<<0;	// x-
		const unsigned char YP = 1<<1;	// y+
		const unsigned char XP = 1<<2;	// x+
		const unsigned char YM = 1<<3;	// y-

		// Portals along x-axis
		for (int y = 0; y < lh; ++y)
		{
			const unsigned char dir[2] = {3,1};
			const unsigned char mask[2] = {YM,YP};
			int start[2] = { -1, -1};
			
			for (int x = 0; x < lw+1; ++x)
			{
				const int idx = x+y*lw;
				for (int j = 0; j < 2; ++j)
				{
					unsigned char set = x<lw ? (cons[idx] & mask[j]) : 0;
					if (set)
					{
						if (start[j] == -1)
							start[j] = x;
					}
					else
					{
						if (start[j] != -1)
						{
							// Add portal.
							rcHeightfieldLayerPortal* portal = allocPortal(&layer->portals, layer->nportals, cportals);
							if (!portal)
							{
								ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'portals' (%d).", cportals);
								return false;
							}
							portal->pos = (unsigned char)y/*+off[j]*/;
							portal->smin = (unsigned char)start[j];
							portal->smax = (unsigned char)x;
							portal->dir = dir[j];
							
							start[j] = -1;
						}
					}
				}
			}
		}
		
		// Portals along y-axis
		for (int x = 0; x < lw; ++x)
		{
			const unsigned char dir[2] = {0,2};
			const unsigned char mask[2] = {XM,XP};
			int start[2] = { -1, -1};
			
			for (int y = 0; y < lh+1; ++y)
			{
				const int idx = x+y*lw;
				for (int j = 0; j < 2; ++j)
				{
					unsigned char set = y<lh ? (cons[idx] & mask[j]) : 0;
					if (set)
					{
						if (start[j] == -1)
							start[j] = y;
					}
					else
					{
						if (start[j] != -1)
						{
							// Add portal.
							rcHeightfieldLayerPortal* portal = allocPortal(&layer->portals,layer->nportals,cportals);
							if (!portal)
							{
								ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'portals' (%d).", cportals);
								return false;
							}
							portal->pos = (unsigned char)x/*+off[j]*/;
							portal->smin = (unsigned char)start[j];
							portal->smax = (unsigned char)y;
							portal->dir = dir[j];
							
							start[j] = -1;
						}
					}
				}
			}
		}
		
	}
	
	ctx->stopTimer(RC_TIMER_BUILD_LAYERS);
	
	return true;
}

inline bool isConnected(rcHeightfieldLayer& layer, const int ia, const int ib, const int walkableClimb)
{
	if (layer.areas[ia] != layer.areas[ib]) return false;
	if (rcAbs((int)layer.heights[ia] - (int)layer.heights[ib]) > walkableClimb) return false;
	return true;
}

struct rcMonotoneRegion
{
	int area;
	unsigned char neis[RC_MAX_NEIS];
	unsigned char nneis;
	unsigned char regId;
};

static bool canMerge(unsigned char oldRegId, unsigned char newRegId, const rcMonotoneRegion* regs, const int nregs)
{
	int count = 0;
	for (int i = 0; i < nregs; ++i)
	{
		const rcMonotoneRegion& reg = regs[i];
		if (reg.regId != oldRegId) continue;
		const int nnei = (int)reg.nneis;
		for (int j = 0; j < nnei; ++j)
		{
			if (regs[reg.neis[j]].regId == newRegId)
				count++;
		}
	}
	return count == 1;
}


// TODO: move this somewhere else, once the layer meshing is done.
bool rcBuildLayerRegions(rcContext* ctx, rcHeightfieldLayer& layer, const int walkableClimb)
{
	rcAssert(ctx);
	
//	ctx->startTimer(RC_TIMER_BUILD_LAYERS);
	
	const int w = layer.width;
	const int h = layer.height;

	rcAssert(layer.regs == 0);
	
	layer.regs = (unsigned char*)rcAlloc(sizeof(unsigned char)*w*h, RC_ALLOC_TEMP);
	if (!layer.regs)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'regs' (%d).", w*h);
		return false;
	}
	memset(layer.regs,0xff,sizeof(unsigned char)*w*h);
	
	const int nsweeps = w;
	rcScopedDelete<rcLayerSweepSpan> sweeps = (rcLayerSweepSpan*)rcAlloc(sizeof(rcLayerSweepSpan)*nsweeps, RC_ALLOC_TEMP);
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	memset(sweeps,0,sizeof(rcLayerSweepSpan)*nsweeps);
	
	// Partition walkable area into monotone regions.
	int prevCount[256];
	unsigned char regId = 0;
	
	for (int y = 0; y < h; ++y)
	{
		if (regId > 0)
			memset(prevCount,0,sizeof(int)*regId);
		unsigned char sweepId = 0;
		
		for (int x = 0; x < w; ++x)
		{
			const int idx = x + y*w;
			if (layer.areas[idx] == RC_NULL_AREA) continue;
				
			unsigned char sid = 0xff;
				
			// -x
			const int xidx = (x-1)+y*w;
			if (x > 0 && isConnected(layer, idx, xidx, walkableClimb))
			{
				if (layer.regs[xidx] != 0xff)
					sid = layer.regs[xidx];
			}
				
			if (sid == 0xff)
			{
				sid = sweepId++;
				sweeps[sid].nei = 0xff;
				sweeps[sid].ns = 0;
			}
				
			// -y
			const int yidx = x+(y-1)*w;
			if (y > 0 && isConnected(layer, idx, yidx, walkableClimb))
			{
				const unsigned char nr = layer.regs[yidx];
				if (nr != 0xff)
				{
					// Set neighbour when first valid neighbour is encoutered.
					if (sweeps[sid].ns == 0)
						sweeps[sid].nei = nr;
					
					if (sweeps[sid].nei == nr)
					{
						// Update existing neighbour
						sweeps[sid].ns++;
						prevCount[nr]++;
					}
					else
					{
						// This is hit if there is nore than one neighbour.
						// Invalidate the neighbour.
						sweeps[sid].nei = 0xff;
					}
				}
			}
			
			layer.regs[idx] = sid;
		}
		
		// Create unique ID.
		for (int i = 0; i < sweepId; ++i)
		{
			// If the neighbour is set and there is only one continuous connection to it,
			// the sweep will be merged with the previous one, else new region is created.
			if (sweeps[i].nei != 0xff && prevCount[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				if (regId == 255)
				{
					ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Region ID overflow.");
					return false;
				}
				sweeps[i].id = regId++;
			}
		}
		
		// Remap local sweep ids to region ids.
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			if (layer.regs[idx] != 0xff)
				layer.regs[idx] = sweeps[layer.regs[idx]].id;
		}
	}

	// Allocate and init layer regions.
	const int nregs = (int)regId;
	rcScopedDelete<rcMonotoneRegion> regs = (rcMonotoneRegion*)rcAlloc(sizeof(rcMonotoneRegion)*nregs, RC_ALLOC_TEMP);
	if (!regs)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'regs' (%d).", nregs);
		return false;
	}
	memset(regs, 0, sizeof(rcMonotoneRegion)*nregs);
	for (int i = 0; i < nregs; ++i)
		regs[i].regId = 0xff;
	
	// Find region neighbours.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			const unsigned char ri = layer.regs[idx];
			if (ri == 0xff)
				continue;
			
			// Update area.
			regs[ri].area++;
			
			// Update neighbours
			const int ymi = x+(y-1)*w;
			if (y > 0 && isConnected(layer, idx, ymi, walkableClimb))
			{
				const unsigned char rai = layer.regs[ymi];
				if (rai != 0xff && rai != ri)
				{
					addUniqueLast(regs[ri].neis, regs[ri].nneis, rai);
					addUniqueLast(regs[rai].neis, regs[rai].nneis, ri);
				}
			}
		}
	}

	for (int i = 0; i < nregs; ++i)
		regs[i].regId = (unsigned char)i;
		
	for (int i = 0; i < nregs; ++i)
	{
		rcMonotoneRegion& reg = regs[i];

		int merge = -1;
		int mergea = 0;
		for (int j = 0; j < (int)reg.nneis; ++j)
		{
			const unsigned char nei = reg.neis[j];
			rcMonotoneRegion& regn = regs[nei];
			if (reg.regId == regn.regId)
				continue;
			if (regn.area > mergea)
			{
				if (canMerge(reg.regId, regn.regId, regs, nregs))
				{
					mergea = regn.area;
					merge = (int)nei;
				}
			}
		}
		if (merge != -1)
		{
			const unsigned char oldId = reg.regId;
			const unsigned char newId = regs[merge].regId;
			for (int j = 0; j < nregs; ++j)
				if (regs[j].regId == oldId)
					regs[j].regId = newId;
		}
	}
	
	// Compact ids.
	unsigned char remap[256];
	memset(remap, 0, 256);
	// Find number of unique regions.
	regId = 0;
	for (int i = 0; i < nregs; ++i)
		remap[regs[i].regId] = 1;
	for (int i = 0; i < 256; ++i)
		if (remap[i])
			remap[i] = regId++;
	// Remap ids.
	for (int i = 0; i < nregs; ++i)
		regs[i].regId = remap[regs[i].regId];

	layer.regCount = regId;
	
	for (int i = 0; i < w*h; ++i)
	{
		if (layer.regs[i] != 0xff)
			layer.regs[i] = regs[layer.regs[i]].regId;
	}
	
	return true;
}

static bool allocVert(rcLayerContour& cont, int& cverts)
{
	if (cont.nverts+1 > cverts)
	{
		cverts = !cverts ? 16 : cverts*2;
		unsigned char* nv = (unsigned char*)rcAlloc(cverts*4, RC_ALLOC_TEMP);
		if (!nv) return false;
		if (cont.nverts)
			memcpy(nv, cont.verts, cont.nverts*4);
		rcFree(cont.verts);
		cont.verts = nv;
	}
	return true;
}

static bool addVertex(rcLayerContour& cont, int x, int y, int z, int r, int& cverts)
{
	// Try to merge with existing segments.
	if (cont.nverts > 1)
	{
		unsigned char* pa = &cont.verts[(cont.nverts-2)*4];
		unsigned char* pb = &cont.verts[(cont.nverts-1)*4];
		if ((int)pb[3] == r)
		{
			if (pa[0] == pb[0] && (int)pb[0] == x)
			{
				// The verts are aligned aling x-axis, update z.
				pb[1] = (unsigned char)y;
				pb[2] = (unsigned char)z;
				pb[3] = (unsigned char)r;
				return true;
			}
			else if (pa[2] == pb[2] && (int)pb[2] == z)
			{
				// The verts are aligned aling z-axis, update x.
				pb[0] = (unsigned char)x;
				pb[1] = (unsigned char)y;
				pb[3] = (unsigned char)r;
				return true;
			}
		}
	}
				
	// Add new point.
	if (!allocVert(cont, cverts))
		return false;
	
	unsigned char* v = &cont.verts[cont.nverts*4];
	v[0] = (unsigned char)x;
	v[1] = (unsigned char)y;
	v[2] = (unsigned char)z;
	v[3] = (unsigned char)r;
	cont.nverts++;
	
	return true;
}


static unsigned char getNeighbourReg(rcHeightfieldLayer& layer,
									 const unsigned char* cons,
									 const int ax, const int ay, const int dir,
									 const int walkableClimb)
{
	const int ia = ax+ay*layer.width;
	
	const int bx = ax + rcGetDirOffsetX(dir);
	const int by = ay + rcGetDirOffsetY(dir);
	if (bx < 0 || by < 0 || bx >= layer.width || by >= layer.height)
	{
		if (cons[ia] & (1<<dir))
			return 0xfe - dir;
		return 0xff;
	}

	const int ib = bx+by*layer.width;
	
	if (rcAbs((int)layer.heights[ia] - (int)layer.heights[ib]) > walkableClimb)
	{
		if (cons[ia] & (1<<dir))
			return 0xfe - dir;
		return 0xff;
	}
	
	return layer.regs[ib];
}

static int getCornerHeight(rcHeightfieldLayer& layer,
						   const int x, const int y, const int dir,
						   const int walkableClimb)
{
	return layer.heights[x+y*layer.width];
}

static bool walkContour(int x, int y, rcHeightfieldLayer& layer,
						const unsigned char* cons,
						const int walkableClimb, rcLayerContour& cont)
{
	const int w = layer.width;
	const int h = layer.height;
	int cverts = cont.nverts;
	
	int startX = x;
	int startY = y;
	int startDir = -1;
	
	for (int i = 0; i < 4; ++i)
	{
		const int dir = (i+3)&3;
		unsigned char rn = getNeighbourReg(layer, cons, x, y, dir, walkableClimb);
		if (rn != layer.regs[x+y*w])
		{
			startDir = dir;
			break;
		}
	}
	if (startDir == -1)
		return true;
	
	int dir = startDir;
	const int maxIter = w*h;
	
	int iter = 0;
	while (iter < maxIter)
	{
		unsigned char rn = getNeighbourReg(layer, cons, x, y, dir, walkableClimb);

		int nx = x;
		int ny = y;
		int ndir = dir;
		
		if (rn != layer.regs[x+y*w])
		{
			// Solid edge.
			int px = x;
			int py = getCornerHeight(layer, x, y, dir, walkableClimb);
			int pz = y;
			switch(dir)
			{
				case 0: pz++; break;
				case 1: px++; pz++; break;
				case 2: px++; break;
			}
			
			// Try to merge with previous vertex.
			if (!addVertex(cont,px,py,pz,rn,cverts))
				return false;
			
			ndir = (dir+1) & 0x3;  // Rotate CW
		}
		else
		{
			// Move to next.
			nx = x + rcGetDirOffsetX(dir);
			ny = y + rcGetDirOffsetY(dir);
			ndir = (dir+3) & 0x3;	// Rotate CCW
		}
		
		if (iter > 0 && x == startX && y == startY && dir == startDir)
			break;

		x = nx;
		y = ny;
		dir = ndir;
		
		iter++;
	}
	
	// Remove last vertex if it is duplicate of the first one.
	unsigned char* pa = &cont.verts[(cont.nverts-1)*4];
	unsigned char* pb = &cont.verts[0];
	if (pa[0] == pb[0] && pa[2] == pb[2])
		cont.nverts--;
	
	return true;
}	


static float distancePtSeg(const int x, const int z,
						   const int px, const int pz,
						   const int qx, const int qz)
{
	/*	float pqx = (float)(qx - px);
	 float pqy = (float)(qy - py);
	 float pqz = (float)(qz - pz);
	 float dx = (float)(x - px);
	 float dy = (float)(y - py);
	 float dz = (float)(z - pz);
	 float d = pqx*pqx + pqy*pqy + pqz*pqz;
	 float t = pqx*dx + pqy*dy + pqz*dz;
	 if (d > 0)
	 t /= d;
	 if (t < 0)
	 t = 0;
	 else if (t > 1)
	 t = 1;
	 
	 dx = px + t*pqx - x;
	 dy = py + t*pqy - y;
	 dz = pz + t*pqz - z;
	 
	 return dx*dx + dy*dy + dz*dz;*/
	
	float pqx = (float)(qx - px);
	float pqz = (float)(qz - pz);
	float dx = (float)(x - px);
	float dz = (float)(z - pz);
	float d = pqx*pqx + pqz*pqz;
	float t = pqx*dx + pqz*dz;
	if (d > 0)
		t /= d;
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	
	dx = px + t*pqx - x;
	dz = pz + t*pqz - z;
	
	return dx*dx + dz*dz;
}

static bool simplifyContour(rcLayerContour& cont, const float maxError)
{
	int* poly = (int*)rcAlloc(sizeof(int)*cont.nverts, RC_ALLOC_TEMP);
	if (!poly)
		return false;
	int npoly = 0;
	
	for (int i = 0; i < cont.nverts; ++i)
	{
		int j = (i+1) % cont.nverts;
		// Check for start of a wall segment.
		unsigned char ra = cont.verts[j*4+3];
		unsigned char rb = cont.verts[i*4+3];
		if (ra != rb)
			poly[npoly++] = i;
	}
	if (npoly < 2)
	{
		// If there is no transitions at all,
		// create some initial points for the simplification process. 
		// Find lower-left and upper-right vertices of the contour.
		int llx = cont.verts[0];
		int llz = cont.verts[2];
		int lli = 0;
		int urx = cont.verts[0];
		int urz = cont.verts[2];
		int uri = 0;
		for (int i = 1; i < cont.nverts; ++i)
		{
			int x = cont.verts[i*4+0];
			int z = cont.verts[i*4+2];
			if (x < llx || (x == llx && z < llz))
			{
				llx = x;
				llz = z;
				lli = i;
			}
			if (x > urx || (x == urx && z > urz))
			{
				urx = x;
				urz = z;
				uri = i;
			}
		}
		npoly = 0;
		poly[npoly++] = lli;
		poly[npoly++] = uri;
	}

	// Add points until all raw points are within
	// error tolerance to the simplified shape.
	for (int i = 0; i < npoly; )
	{
		int ii = (i+1) % npoly;
		
		const int ai = poly[i];
		const int ax = cont.verts[ai*4+0];
		const int az = cont.verts[ai*4+2];
		
		const int bi = poly[ii];
		const int bx = cont.verts[bi*4+0];
		const int bz = cont.verts[bi*4+2];
		
		// Find maximum deviation from the segment.
		float maxd = 0;
		int maxi = -1;
		int ci, cinc, endi;
		
		// Traverse the segment in lexilogical order so that the
		// max deviation is calculated similarly when traversing
		// opposite segments.
		if (bx > ax || (bx == ax && bz > az))
		{
			cinc = 1;
			ci = (ai+cinc) % cont.nverts;
			endi = bi;
		}
		else
		{
			cinc = cont.nverts-1;
			ci = (bi+cinc) % cont.nverts;
			endi = ai;
		}
		
		// Tessellate only outer edges or edges between areas.
		while (ci != endi)
		{
			float d = distancePtSeg(cont.verts[ci*4+0], cont.verts[ci*4+2], ax, az, bx, bz);
			if (d > maxd)
			{
				maxd = d;
				maxi = ci;
			}
			ci = (ci+cinc) % cont.nverts;
		}
		
		
		// If the max deviation is larger than accepted error,
		// add new point, else continue to next segment.
		if (maxi != -1 && maxd > (maxError*maxError))
		{
			npoly++;
			for (int j = npoly-1; j > i; --j)
				poly[j] = poly[j-1];
			poly[i+1] = maxi;
		}
		else
		{
			++i;
		}
	}

	// Remap vertices
	int start = 0;
	for (int i = 1; i < npoly; ++i)
		if (poly[i] < poly[start])
			start = i;
	
	cont.nverts = 0;
	for (int i = 0; i < npoly; ++i)
	{
		const int j = (start+i) % npoly;
		unsigned char* src = &cont.verts[poly[j]*4];
		unsigned char* dst = &cont.verts[cont.nverts*4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		cont.nverts++;
	}
	
	rcFree(poly);
	
	return true;
}

// TODO: move this somewhere else, once the layer meshing is done.
bool rcBuildLayerContours(rcContext* ctx,
						  rcHeightfieldLayer& layer,
						  const int walkableClimb, 	const float maxError,
						  rcLayerContourSet& lcset)
{
	rcAssert(ctx);

	const int w = layer.width;
	const int h = layer.height;
	
	rcAssert(lcset.conts == 0);
	
	rcVcopy(lcset.bmin, layer.bmin);
	rcVcopy(lcset.bmax, layer.bmax);
	lcset.cs = layer.cs;
	lcset.ch = layer.ch;
	lcset.nconts = layer.regCount;
	lcset.conts = (rcLayerContour*)rcAlloc(sizeof(rcLayerContour)*lcset.nconts, RC_ALLOC_TEMP);
	if (!lcset.conts)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerContours: Out of memory 'conts' (%d).", lcset.nconts);
		return false;
	}
	memset(lcset.conts, 0, sizeof(rcLayerContour)*lcset.nconts);

	rcScopedDelete<unsigned char> cons = (unsigned char*)rcAlloc(sizeof(unsigned char)*w*h, RC_ALLOC_TEMP);
	if (!cons)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerContours: Out of memory 'cons' (%d).", w*h);
		return false;
	}
	memset(cons,0,sizeof(unsigned char)*w*h);
	
	
/*	if (portal->dir == 0 || portal->dir == 2)
	{
		const int xx = portal->dir == 0 ? (int)portal->pos : (int)portal->pos+1;
		const float fx = layer->bmin[0] + xx*cs;
		const float fya = layer->bmin[1] + (layer->ymin)*ch;
		const float fyb = layer->bmin[1] + (layer->ymin)*ch;
		const float fza = layer->bmin[2] + portal->smin*cs;
		const float fzb = layer->bmin[2] + portal->smax*cs;
		dd->vertex(fx, fya+h, fza, pcol);
		dd->vertex(fx, fyb+h, fzb, pcol);
	}
	else if (portal->dir == 3 || portal->dir == 1)
	{
		const int yy = portal->dir == 3 ? (int)portal->pos : (int)portal->pos+1;
		const float fxa = layer->bmin[0] + portal->smin*cs;
		const float fxb = layer->bmin[0] + portal->smax*cs;
		const float fya = layer->bmin[1] + (layer->ymin)*ch;
		const float fyb = layer->bmin[1] + (layer->ymin)*ch;
		const float fz = layer->bmin[2] + yy*cs;
		dd->vertex(fxa, fya+h, fz, pcol);
		dd->vertex(fxb, fyb+h, fz, pcol);
	}*/
	
	// Paint portals
	for (int i = 0; i < layer.nportals; ++i)
	{
		const rcHeightfieldLayerPortal* portal = &layer.portals[i];
		if (portal->dir == 0 || portal->dir == 2)
		{
			const unsigned char mask = (const unsigned char)(1 << portal->dir);
			for (int j = (int)portal->smin; j < (int)portal->smax; ++j)
				cons[(int)portal->pos + j*w] |= mask;
		}
		else
		{
			const unsigned char mask = (const unsigned char)(1 << portal->dir);
			for (int j = (int)portal->smin; j < (int)portal->smax; ++j)
				cons[j + (int)portal->pos*w] |= mask;
		}
	}
	
/*	printf("cons:\n");
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			unsigned char c = cons[x+y*w];
			if (c == 0)
				printf(".");
			else
				printf("%x",c);
		}
		printf("\n");
	}*/
			
	
	// Find contours.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			const unsigned char ri = layer.regs[idx];
			if (ri == 0xff)
				continue;

			rcLayerContour& cont = lcset.conts[ri];
			
			if (cont.nverts > 0)
				continue;
			
			cont.reg = ri;
			cont.area = layer.areas[idx];
			
			if (!walkContour(x, y, layer, cons, walkableClimb, cont))
			{
				ctx->log(RC_LOG_ERROR, "rcBuildLayerContours: Failed to walk contour.");
				return false;
			}
			
			if (!simplifyContour(cont, maxError))
			{
				ctx->log(RC_LOG_ERROR, "rcBuildLayerContours: Failed to simplify contour.");
				return false;
			}

		}
	}
	
	return true;
}	


