
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

	int pointInPoly(int nvert, const float* verts, const float* p)
	{
		int i, j, c = 0;
		for (i = 0, j = nvert - 1; i < nvert; j = i++)
		{
			const float* vi = &verts[i * 3];
			const float* vj = &verts[j * 3];
			if (((vi[2] > p[2]) != (vj[2] > p[2])) &&
				(p[0] < (vj[0] - vi[0]) * (p[2] - vi[2]) / (vj[2] - vi[2]) + vi[0]))
				c = !c;
		}
		return c;
	}

	unsigned short* boxBlur(rcCompactHeightfield& chf, int thr, unsigned short* src, unsigned short* dst)
	{
		const int w = chf.width;
		const int h = chf.height;

		thr *= 2;

		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.cells[x + y*w];
				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					const rcCompactSpan& s = chf.spans[i];
					const unsigned short cd = src[i];
					if (cd <= thr)
					{
						dst[i] = cd;
						continue;
					}

					int d = (int)cd;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int ax = x + rcGetDirOffsetX(dir);
							const int ay = y + rcGetDirOffsetY(dir);
							const int ai = (int)chf.cells[ax + ay*w].index + rcGetCon(s, dir);
							d += (int)src[ai];

							const rcCompactSpan& as = chf.spans[ai];
							const int dir2 = (dir + 1) & 0x3;
							if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
							{
								const int ax2 = ax + rcGetDirOffsetX(dir2);
								const int ay2 = ay + rcGetDirOffsetY(dir2);
								const int ai2 = (int)chf.cells[ax2 + ay2*w].index + rcGetCon(as, dir2);
								d += (int)src[ai2];
							}
							else
							{
								d += cd;
							}
						}
						else
						{
							d += cd * 2;
						}
					}
					dst[i] = (unsigned short)((d + 5) / 9);
				}
			}
		}
		return dst;
	}

	void calculateDistanceField(rcCompactHeightfield& chf, unsigned short* src, unsigned short& maxDist)
	{
		const int w = chf.width;
		const int h = chf.height;

		// Init distance and points.
		for (int i = 0; i < chf.spanCount; ++i)
			src[i] = 0xffff;

		// Mark boundary cells.
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.cells[x + y*w];
				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					const rcCompactSpan& s = chf.spans[i];
					const unsigned char area = chf.areas[i];

					int nc = 0;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int ax = x + rcGetDirOffsetX(dir);
							const int ay = y + rcGetDirOffsetY(dir);
							const int ai = (int)chf.cells[ax + ay*w].index + rcGetCon(s, dir);
							if (area == chf.areas[ai])
								nc++;
						}
					}
					if (nc != 4)
						src[i] = 0;
				}
			}
		}


		// Pass 1
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.cells[x + y*w];
				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					const rcCompactSpan& s = chf.spans[i];

					if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
					{
						// (-1,0)
						const int ax = x + rcGetDirOffsetX(0);
						const int ay = y + rcGetDirOffsetY(0);
						const int ai = (int)chf.cells[ax + ay*w].index + rcGetCon(s, 0);
						const rcCompactSpan& as = chf.spans[ai];
						if (src[ai] + 2 < src[i])
							src[i] = src[ai] + 2;

						// (-1,-1)
						if (rcGetCon(as, 3) != RC_NOT_CONNECTED)
						{
							const int aax = ax + rcGetDirOffsetX(3);
							const int aay = ay + rcGetDirOffsetY(3);
							const int aai = (int)chf.cells[aax + aay*w].index + rcGetCon(as, 3);
							if (src[aai] + 3 < src[i])
								src[i] = src[aai] + 3;
						}
					}
					if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
					{
						// (0,-1)
						const int ax = x + rcGetDirOffsetX(3);
						const int ay = y + rcGetDirOffsetY(3);
						const int ai = (int)chf.cells[ax + ay*w].index + rcGetCon(s, 3);
						const rcCompactSpan& as = chf.spans[ai];
						if (src[ai] + 2 < src[i])
							src[i] = src[ai] + 2;

						// (1,-1)
						if (rcGetCon(as, 2) != RC_NOT_CONNECTED)
						{
							const int aax = ax + rcGetDirOffsetX(2);
							const int aay = ay + rcGetDirOffsetY(2);
							const int aai = (int)chf.cells[aax + aay*w].index + rcGetCon(as, 2);
							if (src[aai] + 3 < src[i])
								src[i] = src[aai] + 3;
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
				const rcCompactCell& c = chf.cells[x + y*w];
				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					const rcCompactSpan& s = chf.spans[i];

					if (rcGetCon(s, 2) != RC_NOT_CONNECTED)
					{
						// (1,0)
						const int ax = x + rcGetDirOffsetX(2);
						const int ay = y + rcGetDirOffsetY(2);
						const int ai = (int)chf.cells[ax + ay*w].index + rcGetCon(s, 2);
						const rcCompactSpan& as = chf.spans[ai];
						if (src[ai] + 2 < src[i])
							src[i] = src[ai] + 2;

						// (1,1)
						if (rcGetCon(as, 1) != RC_NOT_CONNECTED)
						{
							const int aax = ax + rcGetDirOffsetX(1);
							const int aay = ay + rcGetDirOffsetY(1);
							const int aai = (int)chf.cells[aax + aay*w].index + rcGetCon(as, 1);
							if (src[aai] + 3 < src[i])
								src[i] = src[aai] + 3;
						}
					}
					if (rcGetCon(s, 1) != RC_NOT_CONNECTED)
					{
						// (0,1)
						const int ax = x + rcGetDirOffsetX(1);
						const int ay = y + rcGetDirOffsetY(1);
						const int ai = (int)chf.cells[ax + ay*w].index + rcGetCon(s, 1);
						const rcCompactSpan& as = chf.spans[ai];
						if (src[ai] + 2 < src[i])
							src[i] = src[ai] + 2;

						// (-1,1)
						if (rcGetCon(as, 0) != RC_NOT_CONNECTED)
						{
							const int aax = ax + rcGetDirOffsetX(0);
							const int aay = ay + rcGetDirOffsetY(0);
							const int aai = (int)chf.cells[aax + aay*w].index + rcGetCon(as, 0);
							if (src[aai] + 3 < src[i])
								src[i] = src[aai] + 3;
						}
					}
				}
			}
		}

		maxDist = 0;
		for (int i = 0; i < chf.spanCount; ++i)
			maxDist = rcMax(src[i], maxDist);

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

/// @par
///
/// The value of spacial parameters are in world units.
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
void rcMarkBoxArea(rcContext* ctx, const float* bmin, const float* bmax, unsigned char areaId, rcCompactHeightfield& chf)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_MARK_BOX_AREA);

	int minx = (int)((bmin[0] - chf.bmin[0]) / chf.cs);
	int miny = (int)((bmin[1] - chf.bmin[1]) / chf.ch);
	int minz = (int)((bmin[2] - chf.bmin[2]) / chf.cs);
	int maxx = (int)((bmax[0] - chf.bmin[0]) / chf.cs);
	int maxy = (int)((bmax[1] - chf.bmin[1]) / chf.ch);
	int maxz = (int)((bmax[2] - chf.bmin[2]) / chf.cs);

	if (maxx < 0) return;
	if (minx >= chf.width) return;
	if (maxz < 0) return;
	if (minz >= chf.height) return;

	if (minx < 0) minx = 0;
	if (maxx >= chf.width) maxx = chf.width - 1;
	if (minz < 0) minz = 0;
	if (maxz >= chf.height) maxz = chf.height - 1;

	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x + z*chf.width];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];
				if ((int)s.minY >= miny && (int)s.minY <= maxy)
				{
					if (chf.areas[i] != RC_NULL_AREA)
						chf.areas[i] = areaId;
				}
			}
		}
	}

	ctx->stopTimer(RC_TIMER_MARK_BOX_AREA);
}


/// @par
///
/// The value of spacial parameters are in world units.
/// 
/// The y-values of the polygon vertices are ignored. So the polygon is effectively 
/// projected onto the xz-plane at @p hmin, then extruded to @p hmax.
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
void rcMarkConvexPolyArea(rcContext* ctx, const float* verts, const int nverts, const float hmin, const float hmax, unsigned char areaId, rcCompactHeightfield& chf)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_MARK_CONVEXPOLY_AREA);

	float bmin[3], bmax[3];
	rcVcopy(bmin, verts);
	rcVcopy(bmax, verts);
	for (int i = 1; i < nverts; ++i)
	{
		rcVmin(bmin, &verts[i * 3]);
		rcVmax(bmax, &verts[i * 3]);
	}
	bmin[1] = hmin;
	bmax[1] = hmax;

	int minx = (int)((bmin[0] - chf.bmin[0]) / chf.cs);
	int miny = (int)((bmin[1] - chf.bmin[1]) / chf.ch);
	int minz = (int)((bmin[2] - chf.bmin[2]) / chf.cs);
	int maxx = (int)((bmax[0] - chf.bmin[0]) / chf.cs);
	int maxy = (int)((bmax[1] - chf.bmin[1]) / chf.ch);
	int maxz = (int)((bmax[2] - chf.bmin[2]) / chf.cs);

	if (maxx < 0) return;
	if (minx >= chf.width) return;
	if (maxz < 0) return;
	if (minz >= chf.height) return;

	if (minx < 0) minx = 0;
	if (maxx >= chf.width) maxx = chf.width - 1;
	if (minz < 0) minz = 0;
	if (maxz >= chf.height) maxz = chf.height - 1;


	// TODO: Optimize.
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x + z*chf.width];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA)
					continue;
				if ((int)s.minY >= miny && (int)s.minY <= maxy)
				{
					float p[3];
					p[0] = chf.bmin[0] + (x + 0.5f)*chf.cs;
					p[1] = 0;
					p[2] = chf.bmin[2] + (z + 0.5f)*chf.cs;

					if (pointInPoly(nverts, verts, p))
					{
						chf.areas[i] = areaId;
					}
				}
			}
		}
	}

	ctx->stopTimer(RC_TIMER_MARK_CONVEXPOLY_AREA);
}


int rcOffsetPoly(const float* verts, const int nverts, const float offset, float* outVerts, const int maxOutVerts)
{
	const float	MITER_LIMIT = 1.20f;

	int n = 0;

	for (int i = 0; i < nverts; i++)
	{
		const int a = (i + nverts - 1) % nverts;
		const int b = i;
		const int c = (i + 1) % nverts;
		const float* va = &verts[a * 3];
		const float* vb = &verts[b * 3];
		const float* vc = &verts[c * 3];
		float dx0 = vb[0] - va[0];
		float dy0 = vb[2] - va[2];
		float d0 = dx0*dx0 + dy0*dy0;
		if (d0 > 1e-6f)
		{
			d0 = 1.0f / rcSqrt(d0);
			dx0 *= d0;
			dy0 *= d0;
		}
		float dx1 = vc[0] - vb[0];
		float dy1 = vc[2] - vb[2];
		float d1 = dx1*dx1 + dy1*dy1;
		if (d1 > 1e-6f)
		{
			d1 = 1.0f / rcSqrt(d1);
			dx1 *= d1;
			dy1 *= d1;
		}
		const float dlx0 = -dy0;
		const float dly0 = dx0;
		const float dlx1 = -dy1;
		const float dly1 = dx1;
		float cross = dx1*dy0 - dx0*dy1;
		float dmx = (dlx0 + dlx1) * 0.5f;
		float dmy = (dly0 + dly1) * 0.5f;
		float dmr2 = dmx*dmx + dmy*dmy;
		bool bevel = dmr2 * MITER_LIMIT*MITER_LIMIT < 1.0f;
		if (dmr2 > 1e-6f)
		{
			const float scale = 1.0f / dmr2;
			dmx *= scale;
			dmy *= scale;
		}

		if (bevel && cross < 0.0f)
		{
			if (n + 2 >= maxOutVerts)
				return 0;
			float d = (1.0f - (dx0*dx1 + dy0*dy1))*0.5f;
			outVerts[n * 3 + 0] = vb[0] + (-dlx0 + dx0*d)*offset;
			outVerts[n * 3 + 1] = vb[1];
			outVerts[n * 3 + 2] = vb[2] + (-dly0 + dy0*d)*offset;
			n++;
			outVerts[n * 3 + 0] = vb[0] + (-dlx1 - dx1*d)*offset;
			outVerts[n * 3 + 1] = vb[1];
			outVerts[n * 3 + 2] = vb[2] + (-dly1 - dy1*d)*offset;
			n++;
		}
		else
		{
			if (n + 1 >= maxOutVerts)
				return 0;
			outVerts[n * 3 + 0] = vb[0] - dmx*offset;
			outVerts[n * 3 + 1] = vb[1];
			outVerts[n * 3 + 2] = vb[2] - dmy*offset;
			n++;
		}
	}

	return n;
}


/// @par
///
/// The value of spacial parameters are in world units.
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
void rcMarkCylinderArea(rcContext* ctx, const float* pos, const float r, const float h, unsigned char areaId, rcCompactHeightfield& chf)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_MARK_CYLINDER_AREA);

	float bmin[3], bmax[3];
	bmin[0] = pos[0] - r;
	bmin[1] = pos[1];
	bmin[2] = pos[2] - r;
	bmax[0] = pos[0] + r;
	bmax[1] = pos[1] + h;
	bmax[2] = pos[2] + r;
	const float r2 = r*r;

	int minx = (int)((bmin[0] - chf.bmin[0]) / chf.cs);
	int miny = (int)((bmin[1] - chf.bmin[1]) / chf.ch);
	int minz = (int)((bmin[2] - chf.bmin[2]) / chf.cs);
	int maxx = (int)((bmax[0] - chf.bmin[0]) / chf.cs);
	int maxy = (int)((bmax[1] - chf.bmin[1]) / chf.ch);
	int maxz = (int)((bmax[2] - chf.bmin[2]) / chf.cs);

	if (maxx < 0) return;
	if (minx >= chf.width) return;
	if (maxz < 0) return;
	if (minz >= chf.height) return;

	if (minx < 0) minx = 0;
	if (maxx >= chf.width) maxx = chf.width - 1;
	if (minz < 0) minz = 0;
	if (maxz >= chf.height) maxz = chf.height - 1;


	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x + z*chf.width];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];

				if (chf.areas[i] == RC_NULL_AREA)
					continue;

				if ((int)s.minY >= miny && (int)s.minY <= maxy)
				{
					const float sx = chf.bmin[0] + (x + 0.5f)*chf.cs;
					const float sz = chf.bmin[2] + (z + 0.5f)*chf.cs;
					const float dx = sx - pos[0];
					const float dz = sz - pos[2];

					if (dx*dx + dz*dz < r2)
					{
						chf.areas[i] = areaId;
					}
				}
			}
		}
	}

	ctx->stopTimer(RC_TIMER_MARK_CYLINDER_AREA);
}

/// @par
/// 
/// This is usually the second to the last step in creating a fully built
/// compact heightfield.  This step is required before regions are built
/// using #rcBuildRegions or #rcBuildRegionsMonotone.
/// 
/// After this step, the distance data is available via the rcCompactHeightfield::maxDistance
/// and rcCompactHeightfield::dist fields.
///
/// @see rcCompactHeightfield, rcBuildRegions, rcBuildRegionsMonotone
bool rcBuildDistanceField(rcContext* ctx, rcCompactHeightfield& chf)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_BUILD_DISTANCEFIELD);

	if (chf.dist)
	{
		rcFree(chf.dist);
		chf.dist = 0;
	}

	unsigned short* src = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
	if (!src)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	unsigned short* dst = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
	if (!dst)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'dst' (%d).", chf.spanCount);
		rcFree(src);
		return false;
	}

	unsigned short maxDist = 0;

	ctx->startTimer(RC_TIMER_BUILD_DISTANCEFIELD_DIST);

	calculateDistanceField(chf, src, maxDist);
	chf.maxDistance = maxDist;

	ctx->stopTimer(RC_TIMER_BUILD_DISTANCEFIELD_DIST);

	ctx->startTimer(RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

	// Blur
	if (boxBlur(chf, 1, src, dst) != src)
		rcSwap(src, dst);

	// Store distance.
	chf.dist = src;

	ctx->stopTimer(RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

	ctx->stopTimer(RC_TIMER_BUILD_DISTANCEFIELD);

	rcFree(dst);

	return true;
}