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
				rcLayerRegion& regn = regs[nei];
				// Skip already visited.
				if (regn.layerId != 0xff)
					continue;
				// Skip if the neighbour is overlapping root region.
				if (contains(root.layers, root.nlayers, nei))
					continue;
				// Skip if the height range would become too large.
				const int ymin = rcMin(root.ymin, regn.ymin);
				const int ymax = rcMin(root.ymax, regn.ymax);
				if ((ymax - ymin) >= 255)
					 continue;

				if (nstack < MAX_STACK)
				{
					// Deepen
					stack[nstack++] = (unsigned char)nei;
					
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
				// Skip if the height range would become too large.
				const int ymin = rcMin(ri.ymin, rj.ymin);
				const int ymax = rcMin(ri.ymax, rj.ymax);
				if ((ymax - ymin) >= 255)
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

		layer->heights = (unsigned char*)rcAlloc(sizeof(unsigned char)*lw*lh, RC_ALLOC_PERM);
		if (!layer->heights)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'heights' (%d).", w*h);
			return false;
		}
		memset(layer->heights, 0xff, sizeof(unsigned char)*lw*lh);
		
		layer->areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*lw*lh, RC_ALLOC_PERM);
		if (!layer->areas)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'areas' (%d).", w*h);
			return false;
		}
		memset(layer->areas, RC_NULL_AREA, sizeof(unsigned char)*lw*lh);

		layer->cons = (unsigned char*)rcAlloc(sizeof(unsigned char)*lw*lh, RC_ALLOC_PERM);
		if (!layer->cons)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'cons' (%d).", w*h);
			return false;
		}
		memset(layer->cons, 0, sizeof(unsigned char)*lw*lh);
		
		// Find layer height bounds.
		int ymin = 0, ymax = 0;
		for (int j = 0; j < nregs; ++j)
		{
			if (regs[j].start && regs[j].layerId == curId)
			{
				ymin = (int)regs[j].ymin;
				ymax = (int)regs[j].ymax;
			}
		}

		// Adjust the bbox to fit the heighfield.
		rcVcopy(layer->bmin, bmin);
		rcVcopy(layer->bmax, bmax);
		layer->bmin[1] = bmin[1] + ymin*chf.ch;
		layer->bmax[1] = bmin[1] + ymax*chf.ch;		
		
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
					layer->heights[idx] = (unsigned char)(s.y - ymin);
					layer->areas[idx] = chf.areas[i];
					
					// Check connection.
					unsigned char portal = 0;
					unsigned char con = 0;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int ax = cx + rcGetDirOffsetX(dir);
							const int ay = cy + rcGetDirOffsetY(dir);
							const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
							unsigned char alid = srcReg[ai] != 0xff ? regs[srcReg[ai]].layerId : 0xff;
							// Portal mask
							if (chf.areas[ai] != RC_NULL_AREA && lid != alid)
								portal |= (unsigned char)(1<<dir);
							// Valid connection mask
							if (chf.areas[ai] != RC_NULL_AREA && lid == alid)
							{
								const int nx = ax - borderSize;
								const int ny = ay - borderSize;
								if (nx >= 0 && ny >= 0 && nx < lw && ny < lh)
									con |= (unsigned char)(1<<dir);
							}
						}
					}
					
					layer->cons[idx] = (portal << 4) | con;
				}
			}
		}
				
	}
	
	ctx->stopTimer(RC_TIMER_BUILD_LAYERS);
	
	return true;
}




// Runtime stuff....

struct rcMonotoneRegion
{
	int area;
	unsigned char neis[RC_MAX_NEIS];
	unsigned char nneis;
	unsigned char regId;
};

inline bool isConnected(rcHeightfieldLayer& layer, const int ia, const int ib, const int walkableClimb)
{
	if (layer.areas[ia] != layer.areas[ib]) return false;
	if (rcAbs((int)layer.heights[ia] - (int)layer.heights[ib]) > walkableClimb) return false;
	return true;
}

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
	unsigned char prevCount[256];
	unsigned char regId = 0;
	
	for (int y = 0; y < h; ++y)
	{
		if (regId > 0)
			memset(prevCount,0,sizeof(unsigned char)*regId);
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
			if (sweeps[i].nei != 0xff && (unsigned short)prevCount[sweeps[i].nei] == sweeps[i].ns)
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



struct rcTempContour
{
	inline rcTempContour() : verts(0), poly(0) {}
	inline ~rcTempContour() { rcFree(verts); rcFree(poly); }
	unsigned char* verts;
	int nverts;
	int cverts;
	unsigned short* poly;
	int npoly;
	int cpoly;
};

static bool appendVertex(rcTempContour& cont, const int x, const int y, const int z, const int r)
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
				return true;
			}
			else if (pa[2] == pb[2] && (int)pb[2] == z)
			{
				// The verts are aligned aling z-axis, update x.
				pb[0] = (unsigned char)x;
				pb[1] = (unsigned char)y;
				return true;
			}
		}
	}
				
	// Add new point.
	if (cont.nverts+1 > cont.cverts)
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
									 const int ax, const int ay, const int dir)
{
	const int ia = ax+ay*layer.width;
	
	const unsigned char con = layer.cons[ia] & 0xf;
	const unsigned char portal = layer.cons[ia] >> 4;
	const unsigned char mask = (unsigned char)(1<<dir);
	
	if ((con & mask) == 0)
	{
		// No connection, return portal or hard edge.
		if (portal & mask)
			return 0xfe - dir;
		return 0xff;
	}
	
	const int bx = ax + rcGetDirOffsetX(dir);
	const int by = ay + rcGetDirOffsetY(dir);
	const int ib = bx+by*layer.width;
	
	return layer.regs[ib];
}

static bool walkContour(rcHeightfieldLayer& layer, int x, int y, rcTempContour& cont)
{
	const int w = layer.width;
	const int h = layer.height;
	
	cont.nverts = 0;
	
	int startX = x;
	int startY = y;
	int startDir = -1;
	
	for (int i = 0; i < 4; ++i)
	{
		const int dir = (i+3)&3;
		unsigned char rn = getNeighbourReg(layer, x, y, dir);
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
		unsigned char rn = getNeighbourReg(layer, x, y, dir);

		int nx = x;
		int ny = y;
		int ndir = dir;
		
		if (rn != layer.regs[x+y*w])
		{
			// Solid edge.
			int px = x;
			int pz = y;
			switch(dir)
			{
				case 0: pz++; break;
				case 1: px++; pz++; break;
				case 2: px++; break;
			}
			
			// Try to merge with previous vertex.
			if (!appendVertex(cont, px, (int)layer.heights[x+y*w], pz,rn))
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

static void simplifyContour(rcTempContour& cont, const float maxError)
{
	cont.npoly = 0;
	
	for (int i = 0; i < cont.nverts; ++i)
	{
		int j = (i+1) % cont.nverts;
		// Check for start of a wall segment.
		unsigned char ra = cont.verts[j*4+3];
		unsigned char rb = cont.verts[i*4+3];
		if (ra != rb)
			cont.poly[cont.npoly++] = i;
	}
	if (cont.npoly < 2)
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
		cont.npoly = 0;
		cont.poly[cont.npoly++] = lli;
		cont.poly[cont.npoly++] = uri;
	}

	// Add points until all raw points are within
	// error tolerance to the simplified shape.
	for (int i = 0; i < cont.npoly; )
	{
		int ii = (i+1) % cont.npoly;
		
		const int ai = (int)cont.poly[i];
		const int ax = (int)cont.verts[ai*4+0];
		const int az = (int)cont.verts[ai*4+2];
		
		const int bi = (int)cont.poly[ii];
		const int bx = (int)cont.verts[bi*4+0];
		const int bz = (int)cont.verts[bi*4+2];
		
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
			cont.npoly++;
			for (int j = cont.npoly-1; j > i; --j)
				cont.poly[j] = cont.poly[j-1];
			cont.poly[i+1] = (unsigned short)maxi;
		}
		else
		{
			++i;
		}
	}

	// Remap vertices
	int start = 0;
	for (int i = 1; i < cont.npoly; ++i)
		if (cont.poly[i] < cont.poly[start])
			start = i;
	
	cont.nverts = 0;
	for (int i = 0; i < cont.npoly; ++i)
	{
		const int j = (start+i) % cont.npoly;
		unsigned char* src = &cont.verts[cont.poly[j]*4];
		unsigned char* dst = &cont.verts[cont.nverts*4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		cont.nverts++;
	}
}

static int getCornerHeight(rcHeightfieldLayer& layer,
						   const int x, const int y, const int z,
						   const int walkableClimb,
						   bool& shouldRemove)
{
	const int w = layer.width;
	const int h = layer.height;

	int n = 0;
	
	unsigned char portal = 0xf;
	int height = 0;
	
	for (int dz = -1; dz <= 0; ++dz)
	{
		for (int dx = -1; dx <= 0; ++dx)
		{
			const int px = x+dx;
			const int pz = z+dz;
			if (px >= 0 && pz >= 0 && px < w && pz < h)
			{
				const int idx  = px + pz*w;
				const int h = (int)layer.heights[idx];
				if (rcAbs(h-y) <= walkableClimb)
				{
					height = rcMax(height, h);
					portal &= (layer.cons[idx] >> 4);
					n++;
				}
			}
		}
	}
	
	int portalCount = 0;
	for (int dir = 0; dir < 4; ++dir)
		if (portal & (1<<dir))
			portalCount++;
	
	shouldRemove = false;
	if (n > 1 && portalCount == 1)
	{
		shouldRemove = true;
	}
	
	return height;
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
	
	// Allocate temp buffer for contour tracing.
	const int maxTempVerts = (w*h*2)*2; // Twice around the layer.
	rcTempContour temp;
	temp.nverts = 0;
	temp.cverts = maxTempVerts;
	temp.npoly = 0;
	temp.cpoly = maxTempVerts;

	temp.verts = (unsigned char*)rcAlloc(sizeof(unsigned char)*temp.cverts, RC_ALLOC_TEMP);
	if (!temp.verts)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerContours: Out of memory 'temp.verts' (%d).", temp.cverts);
		return false;
	}
	
	temp.poly = (unsigned short*)rcAlloc(sizeof(unsigned short)*temp.cpoly, RC_ALLOC_TEMP);
	if (!temp.poly)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerContours: Out of memory 'temp.poly' (%d).", temp.cpoly);
		return false;
	}
			
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
			
			if (!walkContour(layer, x, y, temp))
			{
				ctx->log(RC_LOG_ERROR, "rcBuildLayerContours: Failed to walk contour (nverts=%d cverts=%d).", temp.nverts, temp.cverts);
				return false;
			}
			
			simplifyContour(temp, maxError);

			// Store contour.
			cont.nverts = temp.nverts;
			if (cont.nverts > 0)
			{
				cont.verts = (unsigned char*)rcAlloc(sizeof(unsigned char)*4*temp.nverts, RC_ALLOC_PERM);
				if (!cont.verts)
				{
					ctx->log(RC_LOG_ERROR, "rcBuildLayerContours: Out of memory 'cont.verts' (%d).", temp.nverts);
					return false;
				}
				
				for (int i = 0, j = temp.nverts-1; i < temp.nverts; j=i++)
				{
					unsigned char* dst = &cont.verts[j*4];
					unsigned char* v = &temp.verts[j*4];
					unsigned char* vn = &temp.verts[i*4];
					unsigned char nei = vn[3]; // The neighbour reg is stored at segment vertex of a segment. 
					bool shouldRemove = false;
					unsigned char h = getCornerHeight(layer, (int)v[0], (int)v[1], (int)v[2], walkableClimb, shouldRemove);

					dst[0] = v[0];
					dst[1] = h;
					dst[2] = v[2];
					
					// Store portal direction and remove status to the fourth component.
					dst[3] = 0x0f;
					const int dir = 0xfe - (int)nei;
					if (dir >= 0 && dir <= 3)
						dst[3] = (unsigned char)dir;
					if (shouldRemove)
						dst[3] |= 0x80;
				}
			}
		}
	}
	
	return true;
}	







static const int VERTEX_BUCKET_COUNT2 = (1<<8);

inline int computeVertexHash2(int x, int y, int z)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	const unsigned int h3 = 0xcb1ab31f;
	unsigned int n = h1 * x + h2 * y + h3 * z;
	return (int)(n & (VERTEX_BUCKET_COUNT2-1));
}

static unsigned short addVertex(unsigned short x, unsigned short y, unsigned short z,
								unsigned short* verts, unsigned short* firstVert, unsigned short* nextVert, int& nv)
{
	int bucket = computeVertexHash2(x, 0, z);
	unsigned short i = firstVert[bucket];
	
	while (i != RC_MESH_NULL_IDX)
	{
		const unsigned short* v = &verts[i*3];
		if (v[0] == x && v[2] == z && (rcAbs(v[1] - y) <= 2))
			return i;
		i = nextVert[i]; // next
	}
	
	// Could not find, create new.
	i = nv; nv++;
	unsigned short* v = &verts[i*3];
	v[0] = x;
	v[1] = y;
	v[2] = z;
	nextVert[i] = firstVert[bucket];
	firstVert[bucket] = i;
	
	return (unsigned short)i;
}


struct rcEdge
{
	unsigned short vert[2];
	unsigned short polyEdge[2];
	unsigned short poly[2];
};

static bool buildMeshAdjacency(unsigned short* polys, const int npolys,
							   const unsigned short* verts, const int nverts,
							   const int vertsPerPoly, const rcLayerContourSet& lcset)
{
	// Based on code by Eric Lengyel from:
	// http://www.terathon.com/code/edges.php
	
	int maxEdgeCount = npolys*vertsPerPoly;
	unsigned short* firstEdge = (unsigned short*)rcAlloc(sizeof(unsigned short)*(nverts + maxEdgeCount), RC_ALLOC_TEMP);
	if (!firstEdge)
		return false;
	unsigned short* nextEdge = firstEdge + nverts;
	int edgeCount = 0;
	
	rcEdge* edges = (rcEdge*)rcAlloc(sizeof(rcEdge)*maxEdgeCount, RC_ALLOC_TEMP);
	if (!edges)
	{
		rcFree(firstEdge);
		return false;
	}
	
	for (int i = 0; i < nverts; i++)
		firstEdge[i] = RC_MESH_NULL_IDX;
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 < v1)
			{
				rcEdge& edge = edges[edgeCount];
				edge.vert[0] = v0;
				edge.vert[1] = v1;
				edge.poly[0] = (unsigned short)i;
				edge.polyEdge[0] = (unsigned short)j;
				edge.poly[1] = (unsigned short)i;
				edge.polyEdge[1] = 0xff;
				// Insert edge
				nextEdge[edgeCount] = firstEdge[v0];
				firstEdge[v0] = (unsigned short)edgeCount;
				edgeCount++;
			}
		}
	}
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 > v1)
			{
				bool found = false;
				for (unsigned short e = firstEdge[v1]; e != RC_MESH_NULL_IDX; e = nextEdge[e])
				{
					rcEdge& edge = edges[e];
					if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1])
					{
						edge.poly[1] = (unsigned short)i;
						edge.polyEdge[1] = (unsigned short)j;
						found = true;
						break;
					}
				}
				if (!found)
				{
					// Matching edge not found, it is an open edge, add it.
					rcEdge& edge = edges[edgeCount];
					edge.vert[0] = v1;
					edge.vert[1] = v0;
					edge.poly[0] = (unsigned short)i;
					edge.polyEdge[0] = (unsigned short)j;
					edge.poly[1] = (unsigned short)i;
					edge.polyEdge[1] = 0xff;
					// Insert edge
					nextEdge[edgeCount] = firstEdge[v1];
					firstEdge[v1] = (unsigned short)edgeCount;
					edgeCount++;
				}
			}
		}
	}

	// Mark portal edges.
	for (int i = 0; i < lcset.nconts; ++i)
	{
		rcLayerContour& cont = lcset.conts[i];
		if (cont.nverts < 3)
			continue;
		
		for (int j = 0, k = cont.nverts-1; j < cont.nverts; k=j++)
		{
			const unsigned char* va = &cont.verts[k*4];
			const unsigned char* vb = &cont.verts[j*4];
			const unsigned char dir = va[3] & 0xf;
			if (dir == 0xf)
				continue;
			
			if (dir == 0 || dir == 2)
			{
				// Find matching vertical edge
				const unsigned short x = (unsigned short)va[0];
				unsigned short zmin = (unsigned short)va[2];
				unsigned short zmax = (unsigned short)vb[2];
				if (zmin > zmax)
					rcSwap(zmin, zmax);
				for (int i = 0; i < edgeCount; ++i)
				{
					rcEdge& e = edges[i];
					// Skip connected edges.
					if (e.poly[0] != e.poly[1])
						continue;
					const unsigned short* eva = &verts[e.vert[0]*3];
					const unsigned short* evb = &verts[e.vert[1]*3];
					if (eva[0] == x && evb[0] == x)
					{
						unsigned short ezmin = eva[2];
						unsigned short ezmax = evb[2];
						if (ezmin > ezmax)
							rcSwap(ezmin, ezmax);
						if (overlapRange(zmin,zmax, ezmin, ezmax))
						{
							// Reuse the other polyedge to store dir.
							e.polyEdge[1] = dir;
						}
					}
				}
			}
			else
			{
				// Find matching vertical edge
				const unsigned short z = (unsigned short)va[2];
				unsigned short xmin = (unsigned short)va[0];
				unsigned short xmax = (unsigned short)vb[0];
				if (xmin > xmax)
					rcSwap(xmin, xmax);
				for (int i = 0; i < edgeCount; ++i)
				{
					rcEdge& e = edges[i];
					// Skip connected edges.
					if (e.poly[0] != e.poly[1])
						continue;
					const unsigned short* eva = &verts[e.vert[0]*3];
					const unsigned short* evb = &verts[e.vert[1]*3];
					if (eva[2] == z && evb[2] == z)
					{
						unsigned short exmin = eva[0];
						unsigned short exmax = evb[0];
						if (exmin > exmax)
							rcSwap(exmin, exmax);
						if (overlapRange(xmin,xmax, exmin, exmax))
						{
							// Reuse the other polyedge to store dir.
							e.polyEdge[1] = dir;
						}
					}
				}
			}
		}
	}
	
	
	// Store adjacency
	for (int i = 0; i < edgeCount; ++i)
	{
		const rcEdge& e = edges[i];
		if (e.poly[0] != e.poly[1])
		{
			unsigned short* p0 = &polys[e.poly[0]*vertsPerPoly*2];
			unsigned short* p1 = &polys[e.poly[1]*vertsPerPoly*2];
			p0[vertsPerPoly + e.polyEdge[0]] = e.poly[1];
			p1[vertsPerPoly + e.polyEdge[1]] = e.poly[0];
		}
		else if (e.polyEdge[1] != 0xff)
		{
			unsigned short* p0 = &polys[e.poly[0]*vertsPerPoly*2];
			p0[vertsPerPoly + e.polyEdge[0]] = 0x8000 | (unsigned short)e.poly[1];
		}

	}
	
	rcFree(firstEdge);
	rcFree(edges);
	
	return true;
}


inline int prev(int i, int n) { return i-1 >= 0 ? i-1 : n-1; }
inline int next(int i, int n) { return i+1 < n ? i+1 : 0; }

inline int area2(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return ((int)b[0] - (int)a[0]) * ((int)c[2] - (int)a[2]) - ((int)c[0] - (int)a[0]) * ((int)b[2] - (int)a[2]);
}

//	Exclusive or: true iff exactly one argument is true.
//	The arguments are negated to ensure that they are 0/1
//	values.  Then the bitwise Xor operator may apply.
//	(This idea is due to Michael Baldwin.)
inline bool xorb(bool x, bool y)
{
	return !x ^ !y;
}

// Returns true iff c is strictly to the left of the directed
// line through a to b.
inline bool left(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) < 0;
}

inline bool leftOn(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) <= 0;
}

inline bool collinear(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) == 0;
}

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
static bool intersectProp(const unsigned char* a, const unsigned char* b,
						  const unsigned char* c, const unsigned char* d)
{
	// Eliminate improper cases.
	if (collinear(a,b,c) || collinear(a,b,d) ||
		collinear(c,d,a) || collinear(c,d,b))
		return false;
	
	return xorb(left(a,b,c), left(a,b,d)) && xorb(left(c,d,a), left(c,d,b));
}

// Returns T iff (a,b,c) are collinear and point c lies 
// on the closed segement ab.
static bool between(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	if (!collinear(a, b, c))
		return false;
	// If ab not vertical, check betweenness on x; else on y.
	if (a[0] != b[0])
		return ((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
	else
		return ((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
}

// Returns true iff segments ab and cd intersect, properly or improperly.
static bool intersect(const unsigned char* a, const unsigned char* b,
					  const unsigned char* c, const unsigned char* d)
{
	if (intersectProp(a, b, c, d))
		return true;
	else if (between(a, b, c) || between(a, b, d) ||
			 between(c, d, a) || between(c, d, b))
		return true;
	else
		return false;
}

static bool vequal(const unsigned char* a, const unsigned char* b)
{
	return a[0] == b[0] && a[2] == b[2];
}

// Returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
static bool diagonalie(int i, int j, int n, const unsigned char* verts, const unsigned short* indices)
{
	const unsigned char* d0 = &verts[(indices[i] & 0x7fff) * 4];
	const unsigned char* d1 = &verts[(indices[j] & 0x7fff) * 4];
	
	// For each edge (k,k+1) of P
	for (int k = 0; k < n; k++)
	{
		int k1 = next(k, n);
		// Skip edges incident to i or j
		if (!((k == i) || (k1 == i) || (k == j) || (k1 == j)))
		{
			const unsigned char* p0 = &verts[(indices[k] & 0x7fff) * 4];
			const unsigned char* p1 = &verts[(indices[k1] & 0x7fff) * 4];
			
			if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1))
				continue;
			
			if (intersect(d0, d1, p0, p1))
				return false;
		}
	}
	return true;
}

// Returns true iff the diagonal (i,j) is strictly internal to the 
// polygon P in the neighborhood of the i endpoint.
static bool	inCone(int i, int j, int n, const unsigned char* verts, const unsigned short* indices)
{
	const unsigned char* pi = &verts[(indices[i] & 0x7fff) * 4];
	const unsigned char* pj = &verts[(indices[j] & 0x7fff) * 4];
	const unsigned char* pi1 = &verts[(indices[next(i, n)] & 0x7fff) * 4];
	const unsigned char* pin1 = &verts[(indices[prev(i, n)] & 0x7fff) * 4];
	
	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if (leftOn(pin1, pi, pi1))
		return left(pi, pj, pin1) && left(pj, pi, pi1);
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}

// Returns T iff (v_i, v_j) is a proper internal
// diagonal of P.
static bool diagonal(int i, int j, int n, const unsigned char* verts, const unsigned short* indices)
{
	return inCone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices);
}

static int triangulate(int n, const unsigned char* verts, unsigned short* indices, unsigned short* tris)
{
	int ntris = 0;
	unsigned short* dst = tris;
	
	// The last bit of the index is used to indicate if the vertex can be removed.
	for (int i = 0; i < n; i++)
	{
		int i1 = next(i, n);
		int i2 = next(i1, n);
		if (diagonal(i, i2, n, verts, indices))
			indices[i1] |= 0x8000;
	}
	
	while (n > 3)
	{
		int minLen = -1;
		int mini = -1;
		for (int i = 0; i < n; i++)
		{
			int i1 = next(i, n);
			if (indices[i1] & 0x8000)
			{
				const unsigned char* p0 = &verts[(indices[i] & 0x7fff) * 4];
				const unsigned char* p2 = &verts[(indices[next(i1, n)] & 0x7fff) * 4];
				
				const int dx = (int)p2[0] - (int)p0[0];
				const int dz = (int)p2[2] - (int)p0[2];
				const int len = dx*dx + dz*dz;
				if (minLen < 0 || len < minLen)
				{
					minLen = len;
					mini = i;
				}
			}
		}
		
		if (mini == -1)
		{
			// Should not happen.
			/*			printf("mini == -1 ntris=%d n=%d\n", ntris, n);
			 for (int i = 0; i < n; i++)
			 {
			 printf("%d ", indices[i] & 0x0fffffff);
			 }
			 printf("\n");*/
			return -ntris;
		}
		
		int i = mini;
		int i1 = next(i, n);
		int i2 = next(i1, n);
		
		*dst++ = indices[i] & 0x7fff;
		*dst++ = indices[i1] & 0x7fff;
		*dst++ = indices[i2] & 0x7fff;
		ntris++;
		
		// Removes P[i1] by copying P[i+1]...P[n-1] left one index.
		n--;
		for (int k = i1; k < n; k++)
			indices[k] = indices[k+1];
		
		if (i1 >= n) i1 = 0;
		i = prev(i1,n);
		// Update diagonal flags.
		if (diagonal(prev(i, n), i1, n, verts, indices))
			indices[i] |= 0x8000;
		else
			indices[i] &= 0x7fff;
		
		if (diagonal(i, next(i1, n), n, verts, indices))
			indices[i1] |= 0x8000;
		else
			indices[i1] &= 0x7fff;
	}
	
	// Append the remaining triangle.
	*dst++ = indices[0] & 0x7fff;
	*dst++ = indices[1] & 0x7fff;
	*dst++ = indices[2] & 0x7fff;
	ntris++;
	
	return ntris;
}

static const int MAX_VERTS_PER_POLY = 6;
static const int MAX_REM_EDGES = 48;


static int countPolyVerts(const unsigned short* p, const int nvp)
{
	for (int i = 0; i < nvp; ++i)
		if (p[i] == RC_MESH_NULL_IDX)
			return i;
	return nvp;
}

inline bool uleft(const unsigned short* a, const unsigned short* b, const unsigned short* c)
{
	return ((int)b[0] - (int)a[0]) * ((int)c[2] - (int)a[2]) -
			((int)c[0] - (int)a[0]) * ((int)b[2] - (int)a[2]) < 0;
}

static int getPolyMergeValue(unsigned short* pa, unsigned short* pb,
							 const unsigned short* verts, int& ea, int& eb,
							 const int nvp)
{
	const int na = countPolyVerts(pa, nvp);
	const int nb = countPolyVerts(pb, nvp);
	
	// If the merged polygon would be too big, do not merge.
	if (na+nb-2 > nvp)
		return -1;
	
	// Check if the polygons share an edge.
	ea = -1;
	eb = -1;
	
	for (int i = 0; i < na; ++i)
	{
		unsigned short va0 = pa[i];
		unsigned short va1 = pa[(i+1) % na];
		if (va0 > va1)
			rcSwap(va0, va1);
		for (int j = 0; j < nb; ++j)
		{
			unsigned short vb0 = pb[j];
			unsigned short vb1 = pb[(j+1) % nb];
			if (vb0 > vb1)
				rcSwap(vb0, vb1);
			if (va0 == vb0 && va1 == vb1)
			{
				ea = i;
				eb = j;
				break;
			}
		}
	}
	
	// No common edge, cannot merge.
	if (ea == -1 || eb == -1)
		return -1;
	
	// Check to see if the merged polygon would be convex.
	unsigned short va, vb, vc;
	
	va = pa[(ea+na-1) % na];
	vb = pa[ea];
	vc = pb[(eb+2) % nb];
	if (!uleft(&verts[va*3], &verts[vb*3], &verts[vc*3]))
		return -1;
	
	va = pb[(eb+nb-1) % nb];
	vb = pb[eb];
	vc = pa[(ea+2) % na];
	if (!uleft(&verts[va*3], &verts[vb*3], &verts[vc*3]))
		return -1;
	
	va = pa[ea];
	vb = pa[(ea+1)%na];
	
	int dx = (int)verts[va*3+0] - (int)verts[vb*3+0];
	int dy = (int)verts[va*3+2] - (int)verts[vb*3+2];
	
	return dx*dx + dy*dy;
}

static void mergePolys(unsigned short* pa, unsigned short* pb, int ea, int eb, const int nvp)
{
	unsigned short tmp[MAX_VERTS_PER_POLY*2];
	
	const int na = countPolyVerts(pa, nvp);
	const int nb = countPolyVerts(pb, nvp);
	
	// Merge polygons.
	memset(tmp, 0xff, sizeof(unsigned short)*nvp);
	int n = 0;
	// Add pa
	for (int i = 0; i < na-1; ++i)
		tmp[n++] = pa[(ea+1+i) % na];
	// Add pb
	for (int i = 0; i < nb-1; ++i)
		tmp[n++] = pb[(eb+1+i) % nb];
	
	memcpy(pa, tmp, sizeof(unsigned short)*nvp);
}


static void pushFront(unsigned short v, unsigned short* arr, int& an)
{
	an++;
	for (int i = an-1; i > 0; --i)
		arr[i] = arr[i-1];
	arr[0] = v;
}

static void pushBack(unsigned short v, unsigned short* arr, int& an)
{
	arr[an] = v;
	an++;
}

static bool canRemoveVertex(rcContext* ctx, rcLayerPolyMesh& mesh, const unsigned short rem)
{
	const int nvp = mesh.nvp;
	
	// Count number of polygons to remove.
	int numRemovedVerts = 0;
	int numTouchedVerts = 0;
	int numRemainingEdges = 0;
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
		int numRemoved = 0;
		int numVerts = 0;
		for (int j = 0; j < nv; ++j)
		{
			if (p[j] == rem)
			{
				numTouchedVerts++;
				numRemoved++;
			}
			numVerts++;
		}
		if (numRemoved)
		{
			numRemovedVerts += numRemoved;
			numRemainingEdges += numVerts-(numRemoved+1);
		}
	}
	
	// There would be too few edges remaining to create a polygon.
	// This can happen for example when a tip of a triangle is marked
	// as deletion, but there are no other polys that share the vertex.
	// In this case, the vertex should not be removed.
	if (numRemainingEdges <= 2)
		return false;
	
	// Check that there is enough memory for the test.
	const int maxEdges = numTouchedVerts*2;
	if (maxEdges > MAX_REM_EDGES)
		return false;
	
	// Find edges which share the removed vertex.
	unsigned short edges[MAX_REM_EDGES];
	int nedges = 0;
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
		
		// Collect edges which touches the removed vertex.
		for (int j = 0, k = nv-1; j < nv; k = j++)
		{
			if (p[j] == rem || p[k] == rem)
			{
				// Arrange edge so that a=rem.
				int a = p[j], b = p[k];
				if (b == rem)
					rcSwap(a,b);
				
				// Check if the edge exists
				bool exists = false;
				for (int k = 0; k < nedges; ++k)
				{
					unsigned short* e = &edges[k*3];
					if (e[1] == b)
					{
						// Exists, increment vertex share count.
						e[2]++;
						exists = true;
					}
				}
				// Add new edge.
				if (!exists)
				{
					unsigned short* e = &edges[nedges*3];
					e[0] = a;
					e[1] = b;
					e[2] = 1;
					nedges++;
				}
			}
		}
	}
	
	// There should be no more than 2 open edges.
	// This catches the case that two non-adjacent polygons
	// share the removed vertex. In that case, do not remove the vertex.
	int numOpenEdges = 0;
	for (int i = 0; i < nedges; ++i)
	{
		if (edges[i*3+2] < 2)
			numOpenEdges++;
	}
	if (numOpenEdges > 2)
		return false;
	
	return true;
}

static bool removeVertex(rcContext* ctx, rcLayerPolyMesh& mesh, const unsigned short rem, const int maxTris)
{
	const int nvp = mesh.nvp;
	
	// Count number of polygons to remove.
	int numRemovedVerts = 0;
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
		for (int j = 0; j < nv; ++j)
		{
			if (p[j] == rem)
				numRemovedVerts++;
		}
	}
	
	int nedges = 0;
	unsigned short edges[MAX_REM_EDGES*3];
	int nhole = 0;
	unsigned short hole[MAX_REM_EDGES];
	int nharea = 0;
	unsigned short harea[MAX_REM_EDGES];
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
		bool hasRem = false;
		for (int j = 0; j < nv; ++j)
			if (p[j] == rem) hasRem = true;
		if (hasRem)
		{
			// Collect edges which does not touch the removed vertex.
			for (int j = 0, k = nv-1; j < nv; k = j++)
			{
				if (p[j] != rem && p[k] != rem)
				{
					if (nedges >= MAX_REM_EDGES)
						return false;
					unsigned short* e = &edges[nedges*3];
					e[0] = p[k];
					e[1] = p[j];
					e[2] = mesh.areas[i];
					nedges++;
				}
			}
			// Remove the polygon.
			unsigned short* p2 = &mesh.polys[(mesh.npolys-1)*nvp*2];
			memcpy(p,p2,sizeof(unsigned short)*nvp);
			memset(p+nvp,0xff,sizeof(unsigned short)*nvp);
			mesh.areas[i] = mesh.areas[mesh.npolys-1];
			mesh.npolys--;
			--i;
		}
	}
	
	// Remove vertex.
	for (int i = (int)rem; i < mesh.nverts; ++i)
	{
		mesh.verts[i*3+0] = mesh.verts[(i+1)*3+0];
		mesh.verts[i*3+1] = mesh.verts[(i+1)*3+1];
		mesh.verts[i*3+2] = mesh.verts[(i+1)*3+2];
	}
	mesh.nverts--;
	
	// Adjust indices to match the removed vertex layout.
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
		for (int j = 0; j < nv; ++j)
			if (p[j] > rem) p[j]--;
	}
	for (int i = 0; i < nedges; ++i)
	{
		if (edges[i*3+0] > rem) edges[i*3+0]--;
		if (edges[i*3+1] > rem) edges[i*3+1]--;
	}
	
	if (nedges == 0)
		return true;
	
	// Start with one vertex, keep appending connected
	// segments to the start and end of the hole.
	pushBack(edges[0], hole, nhole);
	pushBack(edges[2], harea, nharea);
	
	while (nedges)
	{
		bool match = false;
		
		for (int i = 0; i < nedges; ++i)
		{
			const unsigned short ea = edges[i*3+0];
			const unsigned short eb = edges[i*3+1];
			const unsigned short a = edges[i*3+2];
			bool add = false;
			if (hole[0] == eb)
			{
				// The segment matches the beginning of the hole boundary.
				if (nhole >= MAX_REM_EDGES)
					return false;
				pushFront(ea, hole, nhole);
				pushFront(a, harea, nharea);
				add = true;
			}
			else if (hole[nhole-1] == ea)
			{
				// The segment matches the end of the hole boundary.
				if (nhole >= MAX_REM_EDGES)
					return false;
				pushBack(eb, hole, nhole);
				pushBack(a, harea, nharea);
				add = true;
			}
			if (add)
			{
				// The edge segment was added, remove it.
				edges[i*3+0] = edges[(nedges-1)*3+0];
				edges[i*3+1] = edges[(nedges-1)*3+1];
				edges[i*3+2] = edges[(nedges-1)*3+2];
				--nedges;
				match = true;
				--i;
			}
		}
		
		if (!match)
			break;
	}
	
	
	unsigned short tris[MAX_REM_EDGES*3];
	unsigned char tverts[MAX_REM_EDGES*3];
	unsigned short tpoly[MAX_REM_EDGES*3];
	
	// Generate temp vertex array for triangulation.
	for (int i = 0; i < nhole; ++i)
	{
		const unsigned short pi = hole[i];
		tverts[i*4+0] = (unsigned char)mesh.verts[pi*3+0];
		tverts[i*4+1] = (unsigned char)mesh.verts[pi*3+1];
		tverts[i*4+2] = (unsigned char)mesh.verts[pi*3+2];
		tverts[i*4+3] = 0;
		tpoly[i] = (unsigned short)i;
	}
	
	// Triangulate the hole.
	int ntris = triangulate(nhole, tverts, tpoly, tris);
	if (ntris < 0)
	{
		ntris = -ntris;
		ctx->log(RC_LOG_WARNING, "removeVertex: triangulate() returned bad results.");
	}
	
	if (ntris > MAX_REM_EDGES)
		return false;
	
	unsigned short polys[MAX_REM_EDGES*MAX_VERTS_PER_POLY];
	unsigned char pareas[MAX_REM_EDGES];
	
	// Build initial polygons.
	int npolys = 0;
	memset(polys, 0xff, ntris*nvp*sizeof(unsigned short));
	for (int j = 0; j < ntris; ++j)
	{
		unsigned short* t = &tris[j*3];
		if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2])
		{
			polys[npolys*nvp+0] = hole[t[0]];
			polys[npolys*nvp+1] = hole[t[1]];
			polys[npolys*nvp+2] = hole[t[2]];
			pareas[npolys] = (unsigned char)harea[t[0]];
			npolys++;
		}
	}
	if (!npolys)
		return true;
	
	// Merge polygons.
	if (nvp > 3)
	{
		for (;;)
		{
			// Find best polygons to merge.
			int bestMergeVal = 0;
			int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;
			
			for (int j = 0; j < npolys-1; ++j)
			{
				unsigned short* pj = &polys[j*nvp];
				for (int k = j+1; k < npolys; ++k)
				{
					unsigned short* pk = &polys[k*nvp];
					int ea, eb;
					int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb, nvp);
					if (v > bestMergeVal)
					{
						bestMergeVal = v;
						bestPa = j;
						bestPb = k;
						bestEa = ea;
						bestEb = eb;
					}
				}
			}
			
			if (bestMergeVal > 0)
			{
				// Found best, merge.
				unsigned short* pa = &polys[bestPa*nvp];
				unsigned short* pb = &polys[bestPb*nvp];
				mergePolys(pa, pb, bestEa, bestEb, nvp);
				memcpy(pb, &polys[(npolys-1)*nvp], sizeof(unsigned short)*nvp);
				pareas[bestPb] = pareas[npolys-1];
				npolys--;
			}
			else
			{
				// Could not merge any polygons, stop.
				break;
			}
		}
	}
	
	// Store polygons.
	for (int i = 0; i < npolys; ++i)
	{
		if (mesh.npolys >= maxTris) break;
		unsigned short* p = &mesh.polys[mesh.npolys*nvp*2];
		memset(p,0xff,sizeof(unsigned short)*nvp*2);
		for (int j = 0; j < nvp; ++j)
			p[j] = polys[i*nvp+j];
		mesh.areas[mesh.npolys] = pareas[i];
		mesh.npolys++;
		if (mesh.npolys > maxTris)
		{
			ctx->log(RC_LOG_ERROR, "removeVertex: Too many polygons %d (max:%d).", mesh.npolys, maxTris);
			return false;
		}
	}
	
	return true;
}


bool rcBuildLayerPolyMesh(rcContext* ctx,
						  rcLayerContourSet& lcset,
						  const int maxVertsPerPoly,
						  rcLayerPolyMesh& mesh)
{
	rcAssert(ctx);
	
	const int nvp = rcMin(maxVertsPerPoly, MAX_VERTS_PER_POLY);
	
//	ctx->startTimer(RC_TIMER_BUILD_POLYMESH);
	
	rcVcopy(mesh.bmin, lcset.bmin);
	rcVcopy(mesh.bmax, lcset.bmax);
	mesh.cs = lcset.cs;
	mesh.ch = lcset.ch;
	
	int maxVertices = 0;
	int maxTris = 0;
	int maxVertsPerCont = 0;
	for (int i = 0; i < lcset.nconts; ++i)
	{
		// Skip null contours.
		if (lcset.conts[i].nverts < 3) continue;
		maxVertices += lcset.conts[i].nverts;
		maxTris += lcset.conts[i].nverts - 2;
		maxVertsPerCont = rcMax(maxVertsPerCont, lcset.conts[i].nverts);
	}
	
	if (maxVertices >= 0xfffe)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Too many vertices %d.", maxVertices);
		return false;
	}
	
	rcScopedDelete<unsigned char> vflags = (unsigned char*)rcAlloc(sizeof(unsigned char)*maxVertices, RC_ALLOC_TEMP);
	if (!vflags)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Out of memory 'vflags' (%d).", maxVertices);
		return false;
	}
	memset(vflags, 0, maxVertices);
	
	mesh.verts = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxVertices*3, RC_ALLOC_PERM);
	if (!mesh.verts)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Out of memory 'mesh.verts' (%d).", maxVertices);
		return false;
	}
	mesh.polys = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxTris*nvp*2*2, RC_ALLOC_PERM);
	if (!mesh.polys)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Out of memory 'mesh.polys' (%d).", maxTris*nvp*2);
		return false;
	}
	mesh.areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*maxTris, RC_ALLOC_PERM);
	if (!mesh.areas)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Out of memory 'mesh.areas' (%d).", maxTris);
		return false;
	}
	mesh.flags = (unsigned short*)rcAlloc(sizeof(unsigned short)*mesh.npolys, RC_ALLOC_PERM);
	if (!mesh.flags)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Out of memory 'mesh.flags' (%d).", mesh.npolys);
		return false;
	}
	// Just allocate and clean the mesh flags array. The user is resposible for filling it.
	memset(mesh.flags, 0, sizeof(unsigned short) * mesh.npolys);
	
	
	mesh.nverts = 0;
	mesh.npolys = 0;
	mesh.nvp = nvp;
	mesh.maxpolys = maxTris;
	
	memset(mesh.verts, 0, sizeof(unsigned short)*maxVertices*3);
	memset(mesh.polys, 0xff, sizeof(unsigned short)*maxTris*nvp*2);
	memset(mesh.areas, 0, sizeof(unsigned char)*maxTris);

	unsigned short firstVert[VERTEX_BUCKET_COUNT2];
	for (int i = 0; i < VERTEX_BUCKET_COUNT2; ++i)
		firstVert[i] = RC_MESH_NULL_IDX;
	
	rcScopedDelete<unsigned short> nextVert = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxVertices, RC_ALLOC_TEMP);
	if (!nextVert)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Out of memory 'nextVert' (%d).", maxVertices);
		return false;
	}
	memset(nextVert, 0, sizeof(unsigned short)*maxVertices);
	
	rcScopedDelete<unsigned short> indices = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxVertsPerCont, RC_ALLOC_TEMP);
	if (!indices)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Out of memory 'indices' (%d).", maxVertsPerCont);
		return false;
	}
	rcScopedDelete<unsigned short> tris = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxVertsPerCont*3, RC_ALLOC_TEMP);
	if (!tris)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Out of memory 'tris' (%d).", maxVertsPerCont*3);
		return false;
	}
	rcScopedDelete<unsigned short> polys = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxVertsPerCont*nvp, RC_ALLOC_TEMP);
	if (!polys)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Out of memory 'polys' (%d).", maxVertsPerCont*nvp);
		return false;
	}
	
	for (int i = 0; i < lcset.nconts; ++i)
	{
		rcLayerContour& cont = lcset.conts[i];
		
		// Skip null contours.
		if (cont.nverts < 3)
			continue;
		
		// Triangulate contour
		for (int j = 0; j < cont.nverts; ++j)
			indices[j] = (unsigned short)j;
		
		int ntris = triangulate(cont.nverts, cont.verts, &indices[0], &tris[0]);
		if (ntris <= 0)
		{
			ctx->log(RC_LOG_WARNING, "rcBuildLayerPolyMesh: Bad triangulation Contour %d.", i);
			ntris = -ntris;
		}
		
		// Add and merge vertices.
		for (int j = 0; j < cont.nverts; ++j)
		{
			const unsigned char* v = &cont.verts[j*4];
			indices[j] = addVertex((unsigned short)v[0], (unsigned short)v[1], (unsigned short)v[2],
								   mesh.verts, firstVert, nextVert, mesh.nverts);
			if (v[3] & 0x80)
			{
				// This vertex should be removed.
				vflags[indices[j]] = 1;
			}
		}
		
		// Build initial polygons.
		int npolys = 0;
		memset(polys, 0xff, sizeof(unsigned short) * maxVertsPerCont * nvp);
		for (int j = 0; j < ntris; ++j)
		{
			const unsigned short* t = &tris[j*3];
			if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2])
			{
				polys[npolys*nvp+0] = indices[t[0]];
				polys[npolys*nvp+1] = indices[t[1]];
				polys[npolys*nvp+2] = indices[t[2]];
				npolys++;
			}
		}
		if (!npolys)
			continue;
		
		// Merge polygons.
		if (nvp > 3)
		{
			for(;;)
			{
				// Find best polygons to merge.
				int bestMergeVal = 0;
				int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;
				
				for (int j = 0; j < npolys-1; ++j)
				{
					unsigned short* pj = &polys[j*nvp];
					for (int k = j+1; k < npolys; ++k)
					{
						unsigned short* pk = &polys[k*nvp];
						int ea, eb;
						int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb, nvp);
						if (v > bestMergeVal)
						{
							bestMergeVal = v;
							bestPa = j;
							bestPb = k;
							bestEa = ea;
							bestEb = eb;
						}
					}
				}
				
				if (bestMergeVal > 0)
				{
					// Found best, merge.
					unsigned short* pa = &polys[bestPa*nvp];
					unsigned short* pb = &polys[bestPb*nvp];
					mergePolys(pa, pb, bestEa, bestEb, nvp);
					memcpy(pb, &polys[(npolys-1)*nvp], sizeof(unsigned short)*nvp);
					npolys--;
				}
				else
				{
					// Could not merge any polygons, stop.
					break;
				}
			}
		}
		
		// Store polygons.
		for (int j = 0; j < npolys; ++j)
		{
			unsigned short* p = &mesh.polys[mesh.npolys*nvp*2];
			unsigned short* q = &polys[j*nvp];
			for (int k = 0; k < nvp; ++k)
				p[k] = q[k];
			mesh.areas[mesh.npolys] = cont.area;
			mesh.npolys++;
			if (mesh.npolys > maxTris)
			{
				ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Too many polygons %d (max:%d).", mesh.npolys, maxTris);
				return false;
			}
		}
	}
	
	
	// Remove edge vertices.
	for (int i = 0; i < mesh.nverts; ++i)
	{
		if (vflags[i])
		{
			if (!canRemoveVertex(ctx, mesh, (unsigned short)i))
				continue;
			if (!removeVertex(ctx, mesh, (unsigned short)i, maxTris))
			{
				// Failed to remove vertex
				ctx->log(RC_LOG_ERROR, "rcBuildPolyMesh: Failed to remove edge vertex %d.", i);
				return false;
			}
			// Remove vertex
			// Note: mesh.nverts is already decremented inside removeVertex()!
			for (int j = i; j < mesh.nverts; ++j)
				vflags[j] = vflags[j+1];
			--i;
		}
	}
	
	// Calculate adjacency.
	if (!buildMeshAdjacency(mesh.polys, mesh.npolys, mesh.verts, mesh.nverts, nvp, lcset))
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: Adjacency failed.");
		return false;
	}
		
	if (mesh.nverts > 0xffff)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: The resulting mesh has too many vertices %d (max %d). Data can be corrupted.", mesh.nverts, 0xffff);
	}
	if (mesh.npolys > 0xffff)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerPolyMesh: The resulting mesh has too many polygons %d (max %d). Data can be corrupted.", mesh.npolys, 0xffff);
	}
	
//	ctx->stopTimer(RC_TIMER_BUILD_POLYMESH);
	
	return true;
}
