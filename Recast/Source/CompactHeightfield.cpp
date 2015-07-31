
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

namespace {
	void insertSort(unsigned char* a, const int n)
	{
		int i, j;
		for (i = 1; i < n; i++)
		{
			const unsigned char value = a[i];
			for (j = i - 1; j >= 0 && a[j] > value; j--)
			{
				a[j + 1] = a[j];
			}
			a[j + 1] = value;
		}
	}
}

/// @par
///
/// This is just the beginning of the process of fully building a compact heightfield.
/// Various filters may be applied, then the distance field and regions built.
/// E.g: #rcBuildDistanceField and #rcBuildRegions
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocCompactHeightfield, rcHeightfield, rcCompactHeightfield, rcConfig
bool rcBuildCompactHeightfield(rcContext* ctx, const int walkableHeight, const int walkableClimb, rcHeightfield& hf, rcCompactHeightfield& chf)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

	const int w = hf.width;
	const int h = hf.height;
	const int spanCount = rcGetHeightFieldSpanCount(ctx, hf);

	// Fill in header.
	chf.width = w;
	chf.height = h;
	chf.spanCount = spanCount;
	chf.walkableHeight = walkableHeight;
	chf.walkableClimb = walkableClimb;
	chf.maxRegions = 0;
	rcVcopy(chf.bmin, hf.bmin);
	rcVcopy(chf.bmax, hf.bmax);
	chf.bmax[1] += walkableHeight*hf.ch;
	chf.cs = hf.cs;
	chf.ch = hf.ch;
	chf.cells = (rcCompactCell*)rcAlloc(sizeof(rcCompactCell)*w*h, RC_ALLOC_PERM);
	if (!chf.cells)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.cells' (%d)", w*h);
		return false;
	}
	memset(chf.cells, 0, sizeof(rcCompactCell)*w*h);
	chf.spans = (rcCompactSpan*)rcAlloc(sizeof(rcCompactSpan)*spanCount, RC_ALLOC_PERM);
	if (!chf.spans)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.spans' (%d)", spanCount);
		return false;
	}
	memset(chf.spans, 0, sizeof(rcCompactSpan)*spanCount);
	chf.areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*spanCount, RC_ALLOC_PERM);
	if (!chf.areas)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.areas' (%d)", spanCount);
		return false;
	}
	memset(chf.areas, RC_NULL_AREA, sizeof(unsigned char)*spanCount);

	const int MAX_HEIGHT = 0xffff;

	// Fill in cells and spans.
	int idx = 0;
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcSpan* s = hf.spans[x + y*w];
			// If there are no spans at this cell, just leave the data to index=0, count=0.
			if (!s) continue;
			rcCompactCell& c = chf.cells[x + y*w];
			c.index = idx;
			c.count = 0;
			while (s)
			{
				if (s->area != RC_NULL_AREA)
				{
					const int bot = (int)s->smax;
					const int top = s->next ? (int)s->next->smin : MAX_HEIGHT;
					chf.spans[idx].minY = (unsigned short)rcClamp(bot, 0, 0xffff);
					chf.spans[idx].height = (unsigned char)rcClamp(top - bot, 0, 0xff);
					chf.areas[idx] = s->area;
					idx++;
					c.count++;
				}
				s = s->next;
			}
		}
	}

	// Find neighbour connections.
	const int MAX_LAYERS = RC_NOT_CONNECTED - 1;
	int tooHighNeighbour = 0;
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y*w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];

				for (int dir = 0; dir < 4; ++dir)
				{
					rcSetCon(s, dir, RC_NOT_CONNECTED);
					const int nx = x + rcGetDirOffsetX(dir);
					const int ny = y + rcGetDirOffsetY(dir);
					// First check that the neighbour cell is in bounds.
					if (nx < 0 || ny < 0 || nx >= w || ny >= h)
						continue;

					// Iterate over all neighbour spans and check if any of the is
					// accessible from current cell.
					const rcCompactCell& nc = chf.cells[nx + ny*w];
					for (int k = (int)nc.index, nk = (int)(nc.index + nc.count); k < nk; ++k)
					{
						const rcCompactSpan& ns = chf.spans[k];
						const int bot = rcMax(s.minY, ns.minY);
						const int top = rcMin(s.minY + s.height, ns.minY + ns.height);

						// Check that the gap between the spans is walkable,
						// and that the climb height between the gaps is not too high.
						if ((top - bot) >= walkableHeight && rcAbs((int)ns.minY - (int)s.minY) <= walkableClimb)
						{
							// Mark direction as walkable.
							const int lidx = k - (int)nc.index;
							if (lidx < 0 || lidx > MAX_LAYERS)
							{
								tooHighNeighbour = rcMax(tooHighNeighbour, lidx);
								continue;
							}
							rcSetCon(s, dir, lidx);
							break;
						}
					}

				}
			}
		}
	}

	if (tooHighNeighbour > MAX_LAYERS)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)", tooHighNeighbour, MAX_LAYERS);
	}

	ctx->stopTimer(RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

	return true;
}

/// @par 
/// 
/// Basically, any spans that are closer to a boundary or obstruction than the specified radius 
/// are marked as unwalkable.
///
/// This method is usually called immediately after the heightfield has been built.
///
/// @see rcCompactHeightfield, rcBuildCompactHeightfield, rcConfig::walkableRadius
bool rcErodeWalkableArea(rcContext* ctx, int radius, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(ctx);

	const int w = compactHeightfield.width;
	const int h = compactHeightfield.height;

	ctx->startTimer(RC_TIMER_ERODE_AREA);

	unsigned char* dist = (unsigned char*)rcAlloc(sizeof(unsigned char) * compactHeightfield.spanCount, RC_ALLOC_TEMP);
	if (!dist)
	{
		ctx->log(RC_LOG_ERROR, "erodeWalkableArea: Out of memory 'dist' (%d).", compactHeightfield.spanCount);
		return false;
	}

	// Init distance.
	memset(dist, 0xff, sizeof(unsigned char) * compactHeightfield.spanCount);

	// Mark boundary cells.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = compactHeightfield.cells[x + y*w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				if (compactHeightfield.areas[i] == RC_NULL_AREA)
				{
					dist[i] = 0;
				}
				else
				{
					const rcCompactSpan& s = compactHeightfield.spans[i];
					int nc = 0;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int nx = x + rcGetDirOffsetX(dir);
							const int ny = y + rcGetDirOffsetY(dir);
							const int nidx = (int)compactHeightfield.cells[nx + ny*w].index + rcGetCon(s, dir);
							if (compactHeightfield.areas[nidx] != RC_NULL_AREA)
							{
								nc++;
							}
						}
					}
					// At least one missing neighbour.
					if (nc != 4)
						dist[i] = 0;
				}
			}
		}
	}

	unsigned char nd;

	// Pass 1
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = compactHeightfield.cells[x + y*w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = compactHeightfield.spans[i];

				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					// (-1,0)
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)compactHeightfield.cells[ax + ay * w].index + rcGetCon(s, 0);
					const rcCompactSpan& as = compactHeightfield.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai] + 2, 255);
					if (nd < dist[i])
						dist[i] = nd;

					// (-1,-1)
					if (rcGetCon(as, 3) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(3);
						const int aay = ay + rcGetDirOffsetY(3);
						const int aai = (int)compactHeightfield.cells[aax + aay * w].index + rcGetCon(as, 3);
						nd = (unsigned char)rcMin((int)dist[aai] + 3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
				if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
				{
					// (0,-1)
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)compactHeightfield.cells[ax + ay * w].index + rcGetCon(s, 3);
					const rcCompactSpan& as = compactHeightfield.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai] + 2, 255);
					if (nd < dist[i])
						dist[i] = nd;

					// (1,-1)
					if (rcGetCon(as, 2) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(2);
						const int aay = ay + rcGetDirOffsetY(2);
						const int aai = (int)compactHeightfield.cells[aax + aay * w].index + rcGetCon(as, 2);
						nd = (unsigned char)rcMin((int)dist[aai] + 3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
			}
		}
	}

	// Pass 2
	for (int y = h - 1; y >= 0; --y)
	{
		for (int x = w - 1; x >= 0; --x)
		{
			const rcCompactCell& c = compactHeightfield.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = compactHeightfield.spans[i];

				if (rcGetCon(s, 2) != RC_NOT_CONNECTED)
				{
					// (1,0)
					const int ax = x + rcGetDirOffsetX(2);
					const int ay = y + rcGetDirOffsetY(2);
					const int ai = (int)compactHeightfield.cells[ax + ay * w].index + rcGetCon(s, 2);
					const rcCompactSpan& as = compactHeightfield.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai] + 2, 255);
					if (nd < dist[i])
						dist[i] = nd;

					// (1,1)
					if (rcGetCon(as, 1) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(1);
						const int aay = ay + rcGetDirOffsetY(1);
						const int aai = (int)compactHeightfield.cells[aax + aay * w].index + rcGetCon(as, 1);
						nd = (unsigned char)rcMin((int)dist[aai] + 3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
				if (rcGetCon(s, 1) != RC_NOT_CONNECTED)
				{
					// (0,1)
					const int ax = x + rcGetDirOffsetX(1);
					const int ay = y + rcGetDirOffsetY(1);
					const int ai = (int)compactHeightfield.cells[ax + ay * w].index + rcGetCon(s, 1);
					const rcCompactSpan& as = compactHeightfield.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai] + 2, 255);
					if (nd < dist[i])
						dist[i] = nd;

					// (-1,1)
					if (rcGetCon(as, 0) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(0);
						const int aay = ay + rcGetDirOffsetY(0);
						const int aai = (int)compactHeightfield.cells[aax + aay * w].index + rcGetCon(as, 0);
						nd = (unsigned char)rcMin((int)dist[aai] + 3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
			}
		}
	}

	for (int i = 0; i < compactHeightfield.spanCount; ++i)
	{
		if (dist[i] < (unsigned char)(radius * 2))
		{
			compactHeightfield.areas[i] = RC_NULL_AREA;
		}
	}

	rcFree(dist);

	ctx->stopTimer(RC_TIMER_ERODE_AREA);

	return true;
}

/// @par
///
/// This filter is usually applied after applying area id's using functions
/// such as #rcMarkBoxArea, #rcMarkConvexPolyArea, and #rcMarkCylinderArea.
/// 
/// @see rcCompactHeightfield
bool rcMedianFilterWalkableArea(rcContext* ctx, rcCompactHeightfield& chf)
{
	rcAssert(ctx);

	const int w = chf.width;
	const int h = chf.height;

	ctx->startTimer(RC_TIMER_MEDIAN_AREA);

	unsigned char* areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_TEMP);
	if (!areas)
	{
		ctx->log(RC_LOG_ERROR, "medianFilterWalkableArea: Out of memory 'areas' (%d).", chf.spanCount);
		return false;
	}

	// Init distance.
	memset(areas, 0xff, sizeof(unsigned char) * chf.spanCount);

	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA)
				{
					areas[i] = chf.areas[i];
					continue;
				}

				unsigned char nei[9];
				for (int j = 0; j < 9; ++j)
					nei[j] = chf.areas[i];

				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
						if (chf.areas[ai] != RC_NULL_AREA)
							nei[dir * 2 + 0] = chf.areas[ai];

						const rcCompactSpan& as = chf.spans[ai];
						const int dir2 = (dir + 1) & 0x3;
						if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
						{
							const int ax2 = ax + rcGetDirOffsetX(dir2);
							const int ay2 = ay + rcGetDirOffsetY(dir2);
							const int ai2 = (int)chf.cells[ax2 + ay2 * w].index + rcGetCon(as, dir2);
							if (chf.areas[ai2] != RC_NULL_AREA)
								nei[dir * 2 + 1] = chf.areas[ai2];
						}
					}
				}
				insertSort(nei, 9);
				areas[i] = nei[4];
			}
		}
	}

	memcpy(chf.areas, areas, sizeof(unsigned char) * chf.spanCount);

	rcFree(areas);

	ctx->stopTimer(RC_TIMER_MEDIAN_AREA);

	return true;
}