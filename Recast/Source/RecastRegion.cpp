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
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

namespace
{
struct LevelStackEntry
{
	LevelStackEntry(int x_, int y_, int index_) : x(x_), y(y_), index(index_) {}
	int x;
	int y;
	int index; // cell 中的 index
};
}  // namespace

static void calculateDistanceField(rcCompactHeightfield& chf, unsigned short* src, unsigned short& maxDist)
{
	const int w = chf.width;
	const int h = chf.height;

	// Init distance and points.
	for (int i = 0; i < chf.spanCount; ++i)
		src[i] = 0xffff;

	// 1.默认：计算的是所有 span 距离可行走与不可行走发生变化的边界处的最小距离。
	// 2.如果自定义了area，则是计算的是所有 span 距离 area 发生变化的边界处的最小距离

	// Mark boundary cells.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned char area = chf.areas[i];

				int nc = 0;
				for (int dir = neighbor_dir_left; dir < neighbor_dir_max; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
						if (area == chf.areas[ai])
							nc++;
					}
				}

				// 有邻居的area不一样，存在boundary
				if (nc != 4)
					src[i] = 0;
			}
		}
	}

	// Pass 1
	int dir = neighbor_dir_left;
	int dir2 = (dir + 3) & 0x3;
	int dir3 = (dir + 2) & 0x3;
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];

				if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
				{
					// (-1,0)
					const int ax = x + rcGetDirOffsetX(dir);
					const int ay = y + rcGetDirOffsetY(dir);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai] + 2 < src[i])
						src[i] = src[ai] + 2;

					// (-1,-1)
					if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(dir2);
						const int aay = ay + rcGetDirOffsetY(dir2);
						const int aai = (int)chf.cells[aax + aay * w].index + rcGetCon(as, dir2);
						if (src[aai] + 3 < src[i])
							src[i] = src[aai] + 3;
					}
				}
				if (rcGetCon(s, dir2) != RC_NOT_CONNECTED)
				{
					// (0,-1)
					const int ax = x + rcGetDirOffsetX(dir2);
					const int ay = y + rcGetDirOffsetY(dir2);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir2);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai] + 2 < src[i])
						src[i] = src[ai] + 2;

					// (1,-1)
					if (rcGetCon(as, dir3) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(dir3);
						const int aay = ay + rcGetDirOffsetY(dir3);
						const int aai = (int)chf.cells[aax + aay * w].index + rcGetCon(as, dir3);
						if (src[aai] + 3 < src[i])
							src[i] = src[aai] + 3;
					}
				}
			}
		}
	}

	// Pass 2
	dir = neighbor_dir_right;
	dir2 = (dir + 3) & 0x3;
	dir3 = (dir + 2) & 0x3;
	for (int y = h - 1; y >= 0; --y)
	{
		for (int x = w - 1; x >= 0; --x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];

				if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
				{
					// (1,0)
					const int ax = x + rcGetDirOffsetX(dir);
					const int ay = y + rcGetDirOffsetY(dir);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai] + 2 < src[i])
						src[i] = src[ai] + 2;

					// (1,1)
					if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(dir2);
						const int aay = ay + rcGetDirOffsetY(dir2);
						const int aai = (int)chf.cells[aax + aay * w].index + rcGetCon(as, dir2);
						if (src[aai] + 3 < src[i])
							src[i] = src[aai] + 3;
					}
				}
				if (rcGetCon(s, dir2) != RC_NOT_CONNECTED)
				{
					// (0,1)
					const int ax = x + rcGetDirOffsetX(dir2);
					const int ay = y + rcGetDirOffsetY(dir2);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir2);
					const rcCompactSpan& as = chf.spans[ai];
					if (src[ai] + 2 < src[i])
						src[i] = src[ai] + 2;

					// (-1,1)
					if (rcGetCon(as, dir3) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(dir3);
						const int aay = ay + rcGetDirOffsetY(dir3);
						const int aai = (int)chf.cells[aax + aay * w].index + rcGetCon(as, dir3);
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

static unsigned short* boxBlur(rcCompactHeightfield& chf, int thr,
	unsigned short* src, unsigned short* dst)
{
	const int w = chf.width;
	const int h = chf.height;

	// 边缘 span 的距离是 0，横 竖、斜向的距离增量分别是 2、3
	thr *= 2;

	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned short cd = src[i];
				if (cd <= thr)
				{
					dst[i] = cd;
					continue;
				}

				// 从左开始，顺时钟八方向遍历
				int d = (int)cd;
				for (int dir = neighbor_dir_left; dir < neighbor_dir_max; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
						d += (int)src[ai];

						const rcCompactSpan& as = chf.spans[ai];
						const int dir2 = (dir + 1) & 0x3;
						if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
						{
							const int ax2 = ax + rcGetDirOffsetX(dir2);
							const int ay2 = ay + rcGetDirOffsetY(dir2);
							const int ai2 = (int)chf.cells[ax2 + ay2 * w].index + rcGetCon(as, dir2);
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

				// 自身加8方向共9个，+5 为了向上取整
				dst[i] = (unsigned short)((d + 5) / 9);
			}
		}
	}
	return dst;
}


static bool floodRegion(int x, int y, int i,
	unsigned short level, unsigned short r,
	rcCompactHeightfield& chf,
	unsigned short* srcReg, unsigned short* srcDist,
	rcTempVector<LevelStackEntry>& stack)
{
	const int w = chf.width;

	const unsigned char area = chf.areas[i];

	// Flood fill mark region.
	stack.clear();
	stack.push_back(LevelStackEntry(x, y, i));
	srcReg[i] = r;
	srcDist[i] = 0;

	const unsigned short lev = level >= 2 ? level - 2 : 0;
	int count = 0;

	while (stack.size() > 0)
	{
		LevelStackEntry& back = stack.back();
		int cx = back.x;
		int cy = back.y;
		int ci = back.index;
		stack.pop_back();

		const rcCompactSpan& cs = chf.spans[ci];

		// Check if any of the neighbours already have a valid region set.
		// 检测8方向的邻居是否已经存在一个有效的 region set
		bool has_valid_reg = false;
		for (int dir = neighbor_dir_left; dir < neighbor_dir_max; ++dir)
		{
			// 8 connected
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(cs, dir);
				if (chf.areas[ai] != area)
					continue;
				unsigned short nr = srcReg[ai];
				if (nr & RC_BORDER_REG) // Do not take borders into account.| 不考虑边界。
					continue;

				// 邻居已分配 reg 并且和 我的不一样
				if (nr != 0 && nr != r)
				{
					has_valid_reg = true;
					break;
				}

				const rcCompactSpan& as = chf.spans[ai];

				const int dir2 = (dir + 1) & 0x3;
				if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
				{
					const int ax2 = ax + rcGetDirOffsetX(dir2);
					const int ay2 = ay + rcGetDirOffsetY(dir2);
					const int ai2 = (int)chf.cells[ax2 + ay2 * w].index + rcGetCon(as, dir2);
					if (chf.areas[ai2] != area)
						continue;
					unsigned short nr2 = srcReg[ai2];
					// 邻居已分配 reg 并且和 我的不一样
					if (nr2 != 0 && nr2 != r)
					{
						has_valid_reg = true;
						break;
					}
				}
			}
		}
		if (has_valid_reg)
		{
			// 8方向的邻居已经存在一个有效的 region set
			srcReg[ci] = 0;
			continue;
		}

		count++;

		// Expand neighbours.
		for (int dir = neighbor_dir_left; dir < neighbor_dir_max; ++dir)
		{
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(cs, dir);
				if (chf.areas[ai] != area)
					continue;

				// 邻居的 dist >= 当前值 并且 邻居没有分配 reg, 则把它合并到当前区域(即，reg一样）
				// 然后把它放进队列，继续检测
				if (chf.dist[ai] >= lev && srcReg[ai] == 0)
				{
					srcReg[ai] = r;
					srcDist[ai] = 0;
					stack.push_back(LevelStackEntry(ax, ay, ai));
				}
			}
		}
	}

	return count > 0;
}

// Struct to keep track of entries in the region table that have been changed.
struct DirtyEntry
{
	DirtyEntry(int index_, unsigned short region_, unsigned short distance2_)
		: index(index_), region(region_), distance2(distance2_) {}
	int index;
	unsigned short region;
	unsigned short distance2;
};
static void expandRegions(int maxIter, unsigned short level,
					      rcCompactHeightfield& chf,
					      unsigned short* srcReg, unsigned short* srcDist,
					      rcTempVector<LevelStackEntry>& stack,
					      bool fillStack)
{
	const int w = chf.width;
	const int h = chf.height;

	if (fillStack)
	{
		// Find cells revealed by the raised level.
		stack.clear();
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.cells[x+y*w];
				for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
				{
					if (chf.dist[i] >= level && srcReg[i] == 0 && chf.areas[i] != RC_NULL_AREA)
					{
						stack.push_back(LevelStackEntry(x, y, i));
					}
				}
			}
		}
	}
	else // use cells in the input stack
	{
		// mark all cells which already have a region
		for (int j=0; j<stack.size(); j++)
		{
			int i = stack[j].index;
			if (srcReg[i] != 0)
				stack[j].index = -1;
		}
	}

	rcTempVector<DirtyEntry> dirtyEntries;
	int iter = 0;
	while (stack.size() > 0)
	{
		int failed = 0;
		dirtyEntries.clear();
		
		for (int j = 0; j < stack.size(); j++)
		{
			int x = stack[j].x;
			int y = stack[j].y;
			int i = stack[j].index;
			if (i < 0)
			{
				// 已分配了reg，跳过
				failed++;
				continue;
			}
			
			// 找到四个邻居中与本身area相同的最小距离和 reg
			unsigned short r = srcReg[i]; // r 值一定为0，非0表示已分配，上面已经跳过
			unsigned short d2 = 0xffff;
			const unsigned char area = chf.areas[i];
			const rcCompactSpan& s = chf.spans[i];
			for (int dir = neighbor_dir_left; dir < neighbor_dir_max; ++dir)
			{
				// 不连通，跳过
				if (rcGetCon(s, dir) == RC_NOT_CONNECTED) continue;
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
				// area 不相同，到边缘了，跳过
				if (chf.areas[ai] != area) continue;

				// 邻居的 reg 存在且 reg 小于 RC_BORDER_REG
				if (srcReg[ai] > 0 && (srcReg[ai] & RC_BORDER_REG) == 0)
				{
					if ((int)srcDist[ai]+2 < (int)d2)
					{
						r = srcReg[ai];
						d2 = srcDist[ai]+2;
					}
				}
			}
			if (r)
			{
				stack[j].index = -1; // mark as used
				dirtyEntries.push_back(DirtyEntry(i, r, d2));
			}
			else
			{
				failed++;
			}
		}
		
		// Copy entries that differ between src and dst to keep them in sync.
		for (int i = 0; i < dirtyEntries.size(); i++)
		{
			int idx = dirtyEntries[i].index;
			srcReg[idx] = dirtyEntries[i].region;
			srcDist[idx] = dirtyEntries[i].distance2;
		}
		
		if (failed == stack.size())
			break;
		
		if (level > 0)
		{
			++iter;
			if (iter >= maxIter)
				break;
		}
	}
}



static void sortCellsByLevel(unsigned short startLevel,
	rcCompactHeightfield& chf,
	const unsigned short* srcReg,
	unsigned int nbStacks, rcTempVector<LevelStackEntry>* stacks,
	unsigned short loglevelsPerStack) // the levels per stack (2 in our case) as a bit shift|每个堆栈的级别（在我们的例子中是 2）作为位移
{
	const int w = chf.width;
	const int h = chf.height;
	startLevel = startLevel >> loglevelsPerStack;

	for (unsigned int j = 0; j < nbStacks; ++j)
		stacks[j].clear();

	// put all cells in the level range into the appropriate stacks
	// 将级别范围内的所有单元格放入适当的堆栈中
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				if (chf.areas[i] == RC_NULL_AREA || srcReg[i] != 0)
					continue;

				// 每两个level为一层
				int level = chf.dist[i] >> loglevelsPerStack;
				int sId = startLevel - level; // 层级差值|每层两个距离单位
				// nbStacks = 8| 最多只有0-7，大于这个值说明距离相差太远了
				if (sId >= (int)nbStacks)
					continue;
				if (sId < 0)
					sId = 0;

				stacks[sId].push_back(LevelStackEntry(x, y, i));
			}
		}
	}
}

static void appendStacks(const rcTempVector<LevelStackEntry>& srcStack,
						 rcTempVector<LevelStackEntry>& dstStack,
						 const unsigned short* srcReg)
{
	// 把没有分配reg的点放到dstStack中
	for (int j=0; j<srcStack.size(); j++)
	{
		int i = srcStack[j].index;
		if ((i < 0) || (srcReg[i] != 0))
			continue;
		dstStack.push_back(srcStack[j]);
	}
}

struct rcRegion
{
	inline rcRegion(unsigned short i) :
		spanCount(0),
		id(i),
		areaType(0),
		remap(false),
		visited(false),
		overlap(false),
		connectsToBorder(false),
		ymin(0xffff),
		ymax(0)
	{}
	
	int spanCount;					// Number of spans belonging to this region
	unsigned short id;				// ID of the region
	unsigned char areaType;			// Are type.
	bool remap;
	bool visited;
	bool overlap;
	bool connectsToBorder;
	unsigned short ymin, ymax;
	rcTempVector<int> connections; // 边缘连接|轮廓 reg id
	rcTempVector<int> floors; // 垂直方向所有的 reg id
};

static void removeAdjacentNeighbours(rcRegion& reg)
{
	// Remove adjacent duplicates.
	// 删除相邻的重复项。
	for (int i = 0; i < reg.connections.size() && reg.connections.size() > 1; )
	{
		int ni = (i+1) % reg.connections.size();
		if (reg.connections[i] == reg.connections[ni])
		{
			// Remove duplicate
			for (int j = i; j < reg.connections.size()-1; ++j)
				reg.connections[j] = reg.connections[j+1];
			reg.connections.pop_back();
		}
		else
			++i;
	}
}

static void replaceNeighbour(rcRegion& reg, unsigned short oldId, unsigned short newId)
{
	bool neiChanged = false;
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == oldId)
		{
			reg.connections[i] = newId;
			neiChanged = true;
		}
	}
	for (int i = 0; i < reg.floors.size(); ++i)
	{
		if (reg.floors[i] == oldId)
			reg.floors[i] = newId;
	}
	if (neiChanged)
		removeAdjacentNeighbours(reg);
}

static bool canMergeWithRegion(const rcRegion& rega, const rcRegion& regb)
{
	// areaType 不相同, 说明是边界
	if (rega.areaType != regb.areaType)
		return false;

	int n = 0;
	for (int i = 0; i < rega.connections.size(); ++i)
	{
		if (rega.connections[i] == regb.id)
			n++;
	}
	//两个区域之间只有一段连续的边相交；
	if (n > 1)
	{
		return false;
	}

	//两个区域不存在垂直方向上的重叠部分
	for (int i = 0; i < rega.floors.size(); ++i)
	{
		if (rega.floors[i] == regb.id)
			return false;
	}
	return true;
}

static void addUniqueFloorRegion(rcRegion& reg, int n)
{
	for (int i = 0; i < reg.floors.size(); ++i)
		if (reg.floors[i] == n)
			return;
	reg.floors.push_back(n);
}

static bool mergeRegions(rcRegion& rega, rcRegion& regb)
{
	unsigned short aid = rega.id;
	unsigned short bid = regb.id;
	
	// Duplicate current neighbourhood.
	rcTempVector<int> acon;
	acon.resize(rega.connections.size());
	for (int i = 0; i < rega.connections.size(); ++i)
		acon[i] = rega.connections[i];
	rcTempVector<int>& bcon = regb.connections;
	
	// Find insertion point on A.
	// 在 A 上查找插入点。
	int insa = -1;
	for (int i = 0; i < acon.size(); ++i)
	{
		if (acon[i] == bid)
		{
			insa = i;
			break;
		}
	}
	if (insa == -1)
		return false;
	
	// Find insertion point on B.
	int insb = -1;
	for (int i = 0; i < bcon.size(); ++i)
	{
		if (bcon[i] == aid)
		{
			insb = i;
			break;
		}
	}
	if (insb == -1)
		return false;
	
	// Merge neighbours.
	rega.connections.clear();
	for (int i = 0, ni = static_cast<int>(acon.size()); i < ni-1; ++i)
	{
		rega.connections.push_back(acon[(insa+1+i) % ni]);
	}
		
	for (int i = 0, ni = static_cast<int>(bcon.size()); i < ni-1; ++i)
	{
		rega.connections.push_back(bcon[(insb+1+i) % ni]);
	}
	
	removeAdjacentNeighbours(rega);
	
	for (int j = 0; j < regb.floors.size(); ++j)
		addUniqueFloorRegion(rega, regb.floors[j]);
	rega.spanCount += regb.spanCount;
	regb.spanCount = 0;
	regb.connections.resize(0);

	return true;
}

static bool isRegionConnectedToBorder(const rcRegion& reg)
{
	// Region is connected to border if
	// one of the neighbours is null id.
	// 如果有一个邻居的连接ID没有，表示当前区域到边界了
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == 0)
			return true;
	}
	return false;
}

static bool isSolidEdge(rcCompactHeightfield& chf, const unsigned short* srcReg,
						int x, int y, int i, int dir)
{
	const rcCompactSpan& s = chf.spans[i];
	unsigned short r = 0;
	if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
		r = srcReg[ai];
	}
	if (r == srcReg[i])
		return false;
	return true;
}

static void walkContour(int x, int y, int i, int dir,
	rcCompactHeightfield& chf,
	const unsigned short* srcReg,
	rcTempVector<int>& cont)
{
	int startDir = dir;
	int starti = i;

	const rcCompactSpan& ss = chf.spans[i];
	unsigned short curReg = 0;
	if (rcGetCon(ss, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(ss, dir);
		curReg = srcReg[ai];
	}
	cont.push_back(curReg);

	int iter = 0;
	while (++iter < 40000)
	{
		const rcCompactSpan& s = chf.spans[i];

		if (isSolidEdge(chf, srcReg, x, y, i, dir))
		{
			// Choose the edge corner
			// 选择边缘角
			unsigned short r = 0;
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(s, dir);
				r = srcReg[ai];
			}
			if (r != curReg)
			{
				curReg = r;
				cont.push_back(curReg);
			}

			dir = (dir + 1) & 0x3;  // Rotate CW | 顺时针旋转 | Clockwise rotation
		}
		else
		{
			int ni = -1;
			const int nx = x + rcGetDirOffsetX(dir);
			const int ny = y + rcGetDirOffsetY(dir);
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const rcCompactCell& nc = chf.cells[nx + ny * chf.width];
				ni = (int)nc.index + rcGetCon(s, dir);
			}
			if (ni == -1)
			{
				// Should not happen.
				return;
			}
			x = nx;
			y = ny;
			i = ni;
			dir = (dir + 3) & 0x3;	// Rotate CCW | Counterclockwise rotation
		}

		if (starti == i && startDir == dir)
		{
			break;
		}
	}

	// Remove adjacent duplicates.
	// 删除相邻的重复项。
	if (cont.size() > 1)
	{
		for (int j = 0; j < cont.size(); )
		{
			int nj = (j + 1) % cont.size();
			if (cont[j] == cont[nj])
			{
				for (int k = j; k < cont.size() - 1; ++k)
					cont[k] = cont[k + 1];
				cont.pop_back();
			}
			else
				++j;
		}
	}
}


static bool mergeAndFilterRegions(rcContext* ctx, int minRegionArea, int mergeRegionSize,
								  unsigned short& maxRegionId,
								  rcCompactHeightfield& chf,
								  unsigned short* srcReg, rcTempVector<int>& overlaps)
{
	const int w = chf.width;
	const int h = chf.height;
	
	const int nreg = maxRegionId+1;
	rcTempVector<rcRegion> regions;
	if (!regions.reserve(nreg)) {
		ctx->log(RC_LOG_ERROR, "mergeAndFilterRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}

	// Construct regions
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short) i));
	
	// Find edge of a region and find connections around the contour.
	// 找到区域的边缘并找到轮廓周围的连接。
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				unsigned short r = srcReg[i];
				// 跳过没有分配的和 >= 最大区域的 span
				if (r == 0 || r >= nreg)
					continue;
				
				rcRegion& reg = regions[r];
				reg.spanCount++;
				
				// Update floors.
				for (int j = (int)c.index; j < ni; ++j)
				{
					// 跳过自身
					if (i == j) continue;

					unsigned short floorId = srcReg[j];
					// 跳过没有分配的和 >= 最大区域的 span
					if (floorId == 0 || floorId >= nreg)
						continue;

					// 垂直方向存在重叠
					if (floorId == r)
						reg.overlap = true;

					// 添加到区域的当前层列表中
					addUniqueFloorRegion(reg, floorId);
				}
				
				// Have found contour|轮廓
				// 已经处理过了，不再处理
				if (reg.connections.size() > 0)
					continue;
				
				reg.areaType = chf.areas[i];
				
				// Check if this cell is next to a border.
				// 检查此单元格是否位于边框旁边。
				int ndir = -1;
				for (int dir = neighbor_dir_left; dir < neighbor_dir_max; ++dir)
				{
					// 是否是实体边缘
					if (isSolidEdge(chf, srcReg, x, y, i, dir))
					{
						ndir = dir;
						break;
					}
				}
				
				if (ndir != -1)
				{
					// The cell is at border.
					// Walk around the contour to find all the neighbours.
					// 当前 cell 在边缘了
					walkContour(x, y, i, ndir, chf, srcReg, reg.connections);
				}
			}
		}
	}

	// Remove too small regions.
	rcTempVector<int> stack(32);
	rcTempVector<int> trace(32);
	for (int i = 0; i < nreg; ++i)
	{
		rcRegion& reg = regions[i];

		// 没有分配ID或区域边界
		if (reg.id == 0 || (reg.id & RC_BORDER_REG))
			continue;                       

		// span 数据为0 跳过
		if (reg.spanCount == 0)
			continue;

		// 已处理过
		if (reg.visited)
			continue;
		
		// Count the total size of all the connected regions.
		// Also keep track of the regions connects to a tile border.
		// 计算所有连接区域的总大小。
		// 还要跟踪连接到图块边框的区域。
		bool connectsToBorder = false;
		int spanCount = 0;
		stack.clear();
		trace.clear();

		reg.visited = true;
		stack.push_back(i);
		
		while (stack.size())
		{
			// Pop
			int ri = stack.back(); stack.pop_back();
			
			rcRegion& creg = regions[ri];

			spanCount += creg.spanCount;
			trace.push_back(ri);

			for (int j = 0; j < creg.connections.size(); ++j)
			{
				if (creg.connections[j] & RC_BORDER_REG)
				{
					// 存在区域边界
					connectsToBorder = true;
					continue;
				}
				
				// 轮廓
				rcRegion& neireg = regions[creg.connections[j]];

				// 已处理
				if (neireg.visited)
					continue;

				// reg id 没有分配
				if (neireg.id == 0 || (neireg.id & RC_BORDER_REG))
					continue;

				// Visit
				stack.push_back(neireg.id);
				neireg.visited = true;
			}
		}
		
		// If the accumulated regions size is too small, remove it.
		// Do not remove areas which connect to tile borders
		// as their size cannot be estimated correctly and removing them
		// can potentially remove necessary areas.
		// 如果累积区域大小过小，则将其移除。
		// 不要移除与图块边界相连的区域
		// 因为无法正确估计它们的大小，并且移除它们
		// 可能会删除必要的区域。
		if (spanCount < minRegionArea && !connectsToBorder)
		{
			// Kill all visited regions.
			for (int j = 0; j < trace.size(); ++j)
			{
				regions[trace[j]].spanCount = 0;
				regions[trace[j]].id = 0;
			}
		}
	}
	
	// Merge too small regions to neighbour regions.
	// 将太小的区域合并到邻近区域。
	int mergeCount = 0 ;
	do
	{
		mergeCount = 0;
		for (int i = 0; i < nreg; ++i)
		{
			rcRegion& reg = regions[i];

			// 跳过没有分配的区域|边界区域
			if (reg.id == 0 || (reg.id & RC_BORDER_REG))
				continue;
			
			// 跳过相交的区域
			if (reg.overlap)
				continue;

			// 跳过span数据为0的
			if (reg.spanCount == 0)
				continue;
			
			// Check to see if the region should be merged.
			// 检测当前区域是否应该合并
			if (reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg))
				continue;
			
			// Small region with more than 1 connection.
			// Or region which is not connected to a border at all.
			// Find smallest neighbour region that connects to this one.
			// 具有多于 1 个连接的小区域。
			// 或者根本没有连接到边界的区域。
			// 查找与此区域相连的最小邻近区域。
			int smallest = 0xfffffff;
			unsigned short mergeId = reg.id;
			for (int j = 0; j < reg.connections.size(); ++j)
			{
				// 边界
				if (reg.connections[j] & RC_BORDER_REG) continue;

				rcRegion& mreg = regions[reg.connections[j]];
				// 没有分配的|边界|重叠
				if (mreg.id == 0 || (mreg.id & RC_BORDER_REG) || mreg.overlap) continue;

				if (mreg.spanCount < smallest &&
					canMergeWithRegion(reg, mreg) &&
					canMergeWithRegion(mreg, reg))
				{
					smallest = mreg.spanCount;
					mergeId = mreg.id;
				}
			}
			// Found new id.
			if (mergeId != reg.id)
			{
				unsigned short oldId = reg.id;
				rcRegion& target = regions[mergeId];
				
				// Merge neighbours.
				if (mergeRegions(target, reg))
				{
					// Fixup regions pointing to current region.
					// 修复指向当前区域的区域。
					for (int j = 0; j < nreg; ++j)
					{
						if (regions[j].id == 0 || (regions[j].id & RC_BORDER_REG)) continue;
						// If another region was already merged into current region
						// change the nid of the previous region too.
						// 如果另一个区域已经合并到当前区域
						// 也更改前一个区域的 nid。
						if (regions[j].id == oldId)
							regions[j].id = mergeId;
						// Replace the current region with the new one if the
						// current regions is neighbour.
						// 如果当前区域是相邻区域，则用新区域替换当前区域。
						replaceNeighbour(regions[j], oldId, mergeId);
					}
					mergeCount++;
				}
			}
		}
	}
	while (mergeCount > 0);
	
	// Compress region Ids.
	for (int i = 0; i < nreg; ++i)
	{
		regions[i].remap = false;
		if (regions[i].id == 0) continue;       // Skip nil regions.
		if (regions[i].id & RC_BORDER_REG) continue;    // Skip external regions.
		regions[i].remap = true;
	}
	
	unsigned short regIdGen = 0;
	for (int i = 0; i < nreg; ++i)
	{
		if (!regions[i].remap)
			continue;
		unsigned short oldId = regions[i].id;
		unsigned short newId = ++regIdGen;
		for (int j = i; j < nreg; ++j)
		{
			if (regions[j].id == oldId)
			{
				regions[j].id = newId;
				regions[j].remap = false;
			}
		}
	}
	maxRegionId = regIdGen;
	
	// Remap regions.
	for (int i = 0; i < chf.spanCount; ++i)
	{
		if ((srcReg[i] & RC_BORDER_REG) == 0)
			srcReg[i] = regions[srcReg[i]].id;
	}

	// Return regions that we found to be overlapping.
	for (int i = 0; i < nreg; ++i)
		if (regions[i].overlap)
			overlaps.push_back(regions[i].id);

	return true;
}


static void addUniqueConnection(rcRegion& reg, int n)
{
	for (int i = 0; i < reg.connections.size(); ++i)
		if (reg.connections[i] == n)
			return;
	reg.connections.push_back(n);
}

static bool mergeAndFilterLayerRegions(rcContext* ctx, int minRegionArea,
									   unsigned short& maxRegionId,
									   rcCompactHeightfield& chf,
									   unsigned short* srcReg)
{
	const int w = chf.width;
	const int h = chf.height;
	
	const int nreg = maxRegionId+1;
	rcTempVector<rcRegion> regions;
	
	// Construct regions
	if (!regions.reserve(nreg)) {
		ctx->log(RC_LOG_ERROR, "mergeAndFilterLayerRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short) i));
	
	// Find region neighbours and overlapping regions.
	rcTempVector<int> lregs(32);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];

			lregs.clear();
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned char area = chf.areas[i];
				const unsigned short ri = srcReg[i];
				if (ri == 0 || ri >= nreg) continue;
				rcRegion& reg = regions[ri];
				
				reg.spanCount++;
				reg.areaType = area;

				reg.ymin = rcMin(reg.ymin, s.y);
				reg.ymax = rcMax(reg.ymax, s.y);
				
				// Collect all region layers.
				lregs.push_back(ri);
				
				// Update neighbours
				for (int dir = neighbor_dir_left; dir < neighbor_dir_max; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						const unsigned short rai = srcReg[ai];
						if (rai > 0 && rai < nreg && rai != ri)
							addUniqueConnection(reg, rai);
						if (rai & RC_BORDER_REG)
							reg.connectsToBorder = true;
					}
				}
				
			}
			
			// Update overlapping regions.
			for (int i = 0; i < lregs.size()-1; ++i)
			{
				for (int j = i+1; j < lregs.size(); ++j)
				{
					if (lregs[i] != lregs[j])
					{
						rcRegion& ri = regions[lregs[i]];
						rcRegion& rj = regions[lregs[j]];
						addUniqueFloorRegion(ri, lregs[j]);
						addUniqueFloorRegion(rj, lregs[i]);
					}
				}
			}
			
		}
	}

	// Create 2D layers from regions.
	unsigned short layerId = 1;

	for (int i = 0; i < nreg; ++i)
		regions[i].id = 0;

	// Merge montone regions to create non-overlapping areas.
	rcTempVector<int> stack(32);
	for (int i = 1; i < nreg; ++i)
	{
		rcRegion& root = regions[i];
		// Skip already visited.
		if (root.id != 0)
			continue;
		
		// Start search.
		root.id = layerId;

		stack.clear();
		stack.push_back(i);
		
		while (stack.size() > 0)
		{
			// Pop front
			rcRegion& reg = regions[stack[0]];
			for (int j = 0; j < stack.size()-1; ++j)
				stack[j] = stack[j+1];
			stack.resize(stack.size()-1);
			
			const int ncons = (int)reg.connections.size();
			for (int j = 0; j < ncons; ++j)
			{
				const int nei = reg.connections[j];
				rcRegion& regn = regions[nei];
				// Skip already visited.
				if (regn.id != 0)
					continue;
				// Skip if different area type, do not connect regions with different area type.
				if (reg.areaType != regn.areaType)
					continue;
				// Skip if the neighbour is overlapping root region.
				//如果邻居与根区域重叠，则跳过。
				bool overlap = false;
				for (int k = 0; k < root.floors.size(); k++)
				{
					if (root.floors[k] == nei)
					{
						overlap = true;
						break;
					}
				}
				if (overlap)
					continue;
					
				// Deepen
				stack.push_back(nei);
					
				// Mark layer id
				regn.id = layerId;
				// Merge current layers to root.
				for (int k = 0; k < regn.floors.size(); ++k)
					addUniqueFloorRegion(root, regn.floors[k]);
				root.ymin = rcMin(root.ymin, regn.ymin);
				root.ymax = rcMax(root.ymax, regn.ymax);
				root.spanCount += regn.spanCount;
				regn.spanCount = 0;
				root.connectsToBorder = root.connectsToBorder || regn.connectsToBorder;
			}
		}
		
		layerId++;
	}
	
	// Remove small regions
	for (int i = 0; i < nreg; ++i)
	{
		if (regions[i].spanCount > 0 && regions[i].spanCount < minRegionArea && !regions[i].connectsToBorder)
		{
			unsigned short reg = regions[i].id;
			for (int j = 0; j < nreg; ++j)
				if (regions[j].id == reg)
					regions[j].id = 0;
		}
	}
	
	// Compress region Ids.
	for (int i = 0; i < nreg; ++i)
	{
		regions[i].remap = false;
		if (regions[i].id == 0) continue;				// Skip nil regions.
		if (regions[i].id & RC_BORDER_REG) continue;    // Skip external regions.
		regions[i].remap = true;
	}
	
	unsigned short regIdGen = 0;
	for (int i = 0; i < nreg; ++i)
	{
		if (!regions[i].remap)
			continue;
		unsigned short oldId = regions[i].id;
		unsigned short newId = ++regIdGen;
		for (int j = i; j < nreg; ++j)
		{
			if (regions[j].id == oldId)
			{
				regions[j].id = newId;
				regions[j].remap = false;
			}
		}
	}
	maxRegionId = regIdGen;
	
	// Remap regions.
	for (int i = 0; i < chf.spanCount; ++i)
	{
		if ((srcReg[i] & RC_BORDER_REG) == 0)
			srcReg[i] = regions[srcReg[i]].id;
	}
	
	return true;
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
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_DISTANCEFIELD);
	
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

	{
		rcScopedTimer timerDist(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST);

		calculateDistanceField(chf, src, maxDist);
		chf.maxDistance = maxDist;
	}

	{
		rcScopedTimer timerBlur(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

		// Blur 模糊
		if (boxBlur(chf, 1, src, dst) != src)
			rcSwap(src, dst);

		// Store distance.
		chf.dist = src;
	}
	
	rcFree(dst);
	
	return true;
}

static void paintRectRegion(int minx, int maxx, int miny, int maxy, unsigned short regId,
							rcCompactHeightfield& chf, unsigned short* srcReg)
{
	const int w = chf.width;	
	for (int y = miny; y < maxy; ++y)
	{
		for (int x = minx; x < maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (chf.areas[i] != RC_NULL_AREA)
					srcReg[i] = regId;
			}
		}
	}
}


static const unsigned short RC_NULL_NEI = 0xffff;

struct rcSweepSpan
{
	unsigned short rid;	// row id
	unsigned short id;	// region id
	unsigned short ns;	// number samples
	unsigned short nei;	// neighbour region id
};

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Partitioning can result in smaller than necessary regions. @p mergeRegionArea helps 
/// reduce unnecessarily small regions.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
/// /// @par
/// 
/// 非空区域将由相互连接、互不重叠的可行走跨度组成，这些跨度将形成单个轮廓。
/// 轮廓将形成简单的多边形。
/// 
/// 如果多个区域形成的面积小于 @p minRegionArea，则所有跨度将
/// 重新分配给零（空）区域。
/// 
/// 分区可能会导致区域小于必要的大小。@p mergeRegionArea 有助于
/// 减少不必要的小区域。
/// 
/// 有关配置参数的更多信息，请参阅 #rcConfig 文档。
/// 
/// 区域数据将通过 rcCompactHeightfield::maxRegions
/// 和 rcCompactSpan::reg 字段获取。
/// 
/// @warning 在尝试构建区域之前，必须使用 #rcBuildDistanceField 创建距离字段。
/// 
/// @see rcCompactHeightfield、rcCompactSpan、rcBuildDistanceField、rcBuildRegionsMonotone、rcConfig
bool rcBuildRegionsMonotone(rcContext* ctx, rcCompactHeightfield& chf,
							const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;
	
	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);

	const int nsweeps = rcMax(chf.width,chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// Mark border regions.
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		// Paint regions
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;
	
	rcTempVector<int> prev(256);

	// Sweep one line at a time.
	for (int y = borderSize; y < h-borderSize; ++y)
	{
		// Collect spans from this row.
		prev.resize(id+1);
		memset(&prev[0],0,sizeof(int)*id);
		unsigned short rid = 1;
		
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;
				
				// -x
				unsigned short previd = 0;
				if (rcGetCon(s, neighbor_dir_left) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(neighbor_dir_left);
					const int ay = y + rcGetDirOffsetY(neighbor_dir_left);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, neighbor_dir_left);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}
				
				// 只要与左边的reg id不相同，则说明多了一个reg id
				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}

				// 沿着Y扫描，遍历X时，Y是一样的，所以以Y的邻居来判断？
				// -y
				if (rcGetCon(s, neighbor_dir_down) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(neighbor_dir_down);
					const int ay = y + rcGetDirOffsetY(neighbor_dir_down);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, neighbor_dir_down);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short neighbourRegId = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == neighbourRegId)
						{
							sweeps[previd].nei = neighbourRegId;
							sweeps[previd].ns++;
							prev[neighbourRegId]++;
						}
						else
						{
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}

				srcReg[i] = previd;
			}
		}
		
		// Create unique ID.
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}
		
		// Remap IDs
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}


	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out small regions.
		rcTempVector<int> overlaps;
		chf.maxRegions = id;
		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// Monotone partitioning does not generate overlapping regions.
		// 单调分割不会产生重叠区域。
	}
	
	// Store the result out.
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];

	return true;
}

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Watershed partitioning can result in smaller than necessary regions, especially in diagonal corridors. 
/// @p mergeRegionArea helps reduce unnecessarily small regions.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the rcCompactHeightfield::maxRegions
/// and rcCompactSpan::reg fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
/*
rcBuildRegions 这个函数稍微绕一点儿，因为用了辅助结构 lvlStacks 来进行遍历。

现在已经有了代表 span 与边界距离的距离场数据，离边界越远距离越大，离边界越近距离越小。

使用分水岭算法划分区域的时候，认为距离最大的 span 为最低点，从这些点开始进行处理。

首先第一步，将高度场边界 borderSize 范围内可行走的 span 的 reg id 标记为头四个 reg id（从 1 开始）。对于 SoloMesh 而言，borderSize 是 0。

函数内定义了一个水平线的变量 unsigned short level = (chf.maxDistance+1) & ~1;，它的初始值是距离场内的最大距离。先加 1 再与 ~1 按位与，是为了向上取整到偶数，方便后面减 2 的处理。因为后面每次循环会将水平线上升 2 个单位的距离。这个 level 比较关键。

sortCellsByLevel 会将高度场内距离大于 level 的、未被分配 reg id 的可行走 span 加入到 lvlStacks 里对应的 stack 中。lvlStacks 一共有 NB_STACKS(8) 个元素，每个 stack 存储 2 个单位距离的 span，
其中 stack 在 lvlStacks 中的序号越小，其中存储的 span 距离越大；序号越大，其中存储的 span 距离越小。但是一次最多只添加比 level 的距离小 14 个单位的 span，如果距离小到对应的 sId 大于 lvlStacks 的大小了，
则直接略过，等待后续处理。而如果遇到了距离比 level 更大的 span，则通通加入到 lvlStacks[0] 中，因为这部分是之前遍历未能处理的，需要带入到当前循环里继续处理。

while (level > 0) 这个循环会在 lvlStacks 为空时（sId == 0），调用 sortCellsByLevel 进行填充，同时每次遍历将水平线 level 上升两个单位，并且从代表距离最远的 lvlStacks[0] 开始进行处理，
逐步往代表着距离更近的 stack 靠近。当 sId != 0 时，appendStacks 会将前一次遍历中未被处理完的元素添加到当前的 stack 中。

while 循环内的主要逻辑有两块，一个是 expandRegions，它会检查 stack 中所有未被分配 reg id 的 span，如果其四方向邻接 span 里存在分配过 reg id、不是 border 的 span，
那么找到其中距离最小的一个，将自己的 reg id 赋为该邻接 span 的 reg id。换句话说，也就是从已分配 reg id 的外层区域，尝试向外再扩展一圈。

然后是 floodRegion，这就是分水岭算法中的泛洪填充了。rcBuildRegions 会遍历 stack 中剩下的所有未被分配 reg id 的 span，依次调用 floodRegion 为其分配一个新的区域 id，
并尝试使用 bfs 算法将这个区域进行扩展。扩展的范围由 unsigned short lev = level >= 2 ? level-2 : 0; 和 if (chf.dist[ai] >= lev && srcReg[ai] == 0) 确定，也就是比当前水平线 level 最多小 2 个单位距离范围内的未被分配 reg id 的 span。

需要注意，如果 floodRegion 在填充时发现一个 span 其八方向里已经有被分配了不同 reg id 的邻接 span，那么跳过对这个 span 得到 reg id 填充，等待后续处理（下次 expandRegions 时会将其 reg id 赋值为其四方向邻接里最近的 reg id）。

当水平线 level 减到 0 后，再调用 expandRegions 对所有未处理的 span 的 reg id 赋值，整个分水岭算法就结束了。
*/
bool rcBuildRegions(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);

	const int w = chf.width;
	const int h = chf.height;

	rcScopedDelete<unsigned short> buf((unsigned short*)rcAlloc(sizeof(unsigned short) * chf.spanCount * 2, RC_ALLOC_TEMP));
	if (!buf)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegions: Out of memory 'tmp' (%d).", chf.spanCount * 4);
		return false;
	}

	ctx->startTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

	const int LOG_NB_STACKS = 3;
	const int NB_STACKS = 1 << LOG_NB_STACKS;
	rcTempVector<LevelStackEntry> lvlStacks[NB_STACKS];
	for (int i = 0; i < NB_STACKS; ++i)
		lvlStacks[i].reserve(256);

	rcTempVector<LevelStackEntry> stack;
	stack.reserve(256);

	unsigned short* srcReg = buf;
	unsigned short* srcDist = buf + chf.spanCount;

	memset(srcReg, 0, sizeof(unsigned short) * chf.spanCount);
	memset(srcDist, 0, sizeof(unsigned short) * chf.spanCount);

	unsigned short regionId = 1;
	unsigned short level = (chf.maxDistance + 1) & ~1;

	// TODO: Figure better formula, expandIters defines how much the 
	// watershed "overflows" and simplifies the regions. Tying it to
	// agent radius was usually good indication how greedy it could be.
	// const int expandIters = 4 + walkableRadius * 2;
	// TODO：设计更合理的公式，expandIters 定义流域“溢出”的程度，并简化区域。将其与代理半径绑定通常可以很好地表明其贪婪程度。
	const int expandIters = 8;

	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);

		// Paint regions
		paintRectRegion(0, bw, 0, h, regionId | RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(w - bw, w, 0, h, regionId | RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, 0, bh, regionId | RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, h - bh, h, regionId | RC_BORDER_REG, chf, srcReg); regionId++;
	}

	chf.borderSize = borderSize;

	int sId = -1;
	while (level > 0)
	{
		level = level >= 2 ? level - 2 : 0;
		sId = (sId + 1) & (NB_STACKS - 1);

		//		ctx->startTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		if (sId == 0)
		{
			sortCellsByLevel(level, chf, srcReg, NB_STACKS, lvlStacks, 1);
		}
		else
		{
			// 把上一层中没有分配区域的复制到当前层中
			appendStacks(lvlStacks[sId - 1], lvlStacks[sId], srcReg); // copy left overs from last level
		}

		//		ctx->stopTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		{
			rcScopedTimer timerExpand(ctx, RC_TIMER_BUILD_REGIONS_EXPAND);

			// Expand current regions until no empty connected cells found.
			// 扩展当前区域，直到找不到空的连接单元。
			expandRegions(expandIters, level, chf, srcReg, srcDist, lvlStacks[sId], false);
		}

		{
			rcScopedTimer timerFloor(ctx, RC_TIMER_BUILD_REGIONS_FLOOD);

			// Mark new regions with IDs.
			for (int j = 0; j < lvlStacks[sId].size(); j++)
			{
				LevelStackEntry current = lvlStacks[sId][j];
				int x = current.x;
				int y = current.y;
				int i = current.index;
				// 没有分配 reg
				if (i >= 0 && srcReg[i] == 0)
				{
					if (floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, stack))
					{
						if (regionId == 0xFFFF)
						{
							ctx->log(RC_LOG_ERROR, "rcBuildRegions: Region ID overflow");
							return false;
						}

						regionId++;
					}
				}
			}
		}
	}

	// Expand current regions until no empty connected cells found.
	expandRegions(expandIters * 8, 0, chf, srcReg, srcDist, stack, true);

	ctx->stopTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out small regions.
		rcTempVector<int> overlaps;
		chf.maxRegions = regionId;
		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// If overlapping regions were found during merging, split those regions.
		if (overlaps.size() > 0)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildRegions: %d overlapping regions.", overlaps.size());
		}
	}

	// Write the result out.
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];

	return true;
}


bool rcBuildLayerRegions(rcContext* ctx, rcCompactHeightfield& chf,
						 const int borderSize, const int minRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;
	
	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);
	
	const int nsweeps = rcMax(chf.width,chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// Mark border regions.
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);
		// Paint regions
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;
	
	rcTempVector<int> prev(256);
	
	// Sweep one line at a time.
	for (int y = borderSize; y < h-borderSize; ++y)
	{
		// Collect spans from this row.
		prev.resize(id+1);
		memset(&prev[0],0,sizeof(int)*id);
		unsigned short rid = 1;
		
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;
				
				// -x
				unsigned short previd = 0;
				if (rcGetCon(s, neighbor_dir_left) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(neighbor_dir_left);
					const int ay = y + rcGetDirOffsetY(neighbor_dir_left);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, neighbor_dir_left);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}
				
				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rid = previd; // row id
					sweeps[previd].ns = 0; // numbers
					sweeps[previd].nei = 0; // neighbourId
				}
				
				// -y
				if (rcGetCon(s, neighbor_dir_down) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(neighbor_dir_down);
					const int ay = y + rcGetDirOffsetY(neighbor_dir_down);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, neighbor_dir_down);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short neighbourId = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == neighbourId)
						{
							sweeps[previd].nei = neighbourId;
							sweeps[previd].ns++;
							prev[neighbourId]++;
						}
						else
						{
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}
				
				srcReg[i] = previd;
			}
		}
		
		// Create unique ID.
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}
		
		// Remap IDs
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}
	
	
	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge monotone regions to layers and remove small regions.
		chf.maxRegions = id;
		if (!mergeAndFilterLayerRegions(ctx, minRegionArea, chf.maxRegions, chf, srcReg))
			return false;
	}
	
	
	// Store the result out.
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];
	
	return true;
}
