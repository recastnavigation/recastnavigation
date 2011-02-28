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

	lset.nlayers = (int)layerId;
	rcVcopy(lset.bmin, chf.bmin);
	rcVcopy(lset.bmax, chf.bmax);
	lset.bmin[0] += borderSize*chf.cs;
	lset.bmin[2] += borderSize*chf.cs;
	lset.bmax[0] -= borderSize*chf.cs;
	lset.bmax[2] -= borderSize*chf.cs;
	lset.cs = chf.cs;
	lset.ch = chf.ch;
	
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
							rcHeightfieldLayerPortal* portal = allocPortal(&layer->portals,layer->nportals,cportals);
							if (!portal)
							{
								ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'portals' (%d).", cportals);
								return false;
							}
							portal->pos = (unsigned short)y;
							portal->smin = (unsigned short)start[j];
							portal->smax = (unsigned short)x;
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
							portal->pos = (unsigned short)x;
							portal->smin = (unsigned short)start[j];
							portal->smax = (unsigned short)y;
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
	if (layer.areas[ib] == RC_NULL_AREA) return false;
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

	for (int i = 0; i < w*h; ++i)
	{
		if (layer.regs[i] != 0xff)
			layer.regs[i] = regs[layer.regs[i]].regId;
	}
	
	return true;
}
