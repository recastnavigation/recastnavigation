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

#include "Recast.h"
#include "RecastAssert.h"

#include <stdlib.h>

namespace
{
	const int MAX_HEIGHTFIELD_HEIGHT = 0xffff; // TODO (graham): Move this to a more visible constant and update usages.
}

void rcFilterLowHangingWalkableObstacles(rcContext* context, const int walkableClimb, rcHeightfield& heightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_LOW_OBSTACLES);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			rcSpan* previousSpan = NULL;
			bool previousWasWalkable = false;
			unsigned char previousAreaID = RC_NULL_AREA;

			// For each span in the column...
			for (rcSpan* span = heightfield.spans[x + z * xSize]; span != NULL; previousSpan = span, span = span->next)
			{
				const bool walkable = span->area != RC_NULL_AREA;

				// If current span is not walkable, but there is walkable span just below it and the height difference
				// is small enough for the agent to walk over, mark the current span as walkable too.
				// 如果当前跨度不可行走，但其下方有一个可行走跨度，且高度差
				// 足够小，代理可以越过，则将当前跨度也标记为可行走。
				if (!walkable && previousWasWalkable && (int)span->smax - (int)previousSpan->smax <= walkableClimb)
				{
					span->area = previousAreaID;
				}

				// Copy the original walkable value regardless of whether we changed it.
				// This prevents multiple consecutive non-walkable spans from being erroneously marked as walkable.
				// 无论我们是否更改了原始可行走值，都会复制它。
				// 这可以防止多个连续的不可行走跨度被错误地标记为可行走。
				previousWasWalkable = walkable;
				previousAreaID = span->area;
			}
		}
	}
}

void rcFilterLedgeSpans(rcContext* context, const int walkableHeight, const int walkableClimb, rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	
	// Mark spans that are adjacent to a ledge as unwalkable..
	// 将与壁架相邻的跨度标记为不可行走。
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z * xSize]; span; span = span->next)
			{
				// Skip non-walkable spans.
				if (span->area == RC_NULL_AREA)
				{
					continue;
				}

				const int floor = (int)(span->smax);
				const int ceiling = span->next ? (int)(span->next->smin) : MAX_HEIGHTFIELD_HEIGHT;

				// The difference between this walkable area and the lowest neighbor walkable area.
				// This is the difference between the current span and all neighbor spans that have
				// enough space for an agent to move between, but not accounting at all for surface slope.
				// 此可行走区域与最低邻近可行走区域之间的差异。
				// 这是当前跨度与所有邻近跨度之间的差异，这些跨度具有
				// 足够的空间供代理在其间移动，但完全不考虑表面坡度。
				int lowestNeighborFloorDifference = MAX_HEIGHTFIELD_HEIGHT;

				// Min and max height of accessible neighbours.
				int lowestTraversableNeighborFloor = span->smax;
				int highestTraversableNeighborFloor = span->smax;

				for (int direction = 0; direction < 4; ++direction)
				{
					const int neighborX = x + rcGetDirOffsetX(direction);
					const int neighborZ = z + rcGetDirOffsetY(direction);

					// Skip neighbours which are out of bounds.
					if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize)
					{
						lowestNeighborFloorDifference = -walkableClimb - 1;
						break;
					}

					const rcSpan* neighborSpan = heightfield.spans[neighborX + neighborZ * xSize];

					// The most we can step down to the neighbor is the walkableClimb distance.
					// Start with the area under the neighbor span
					// 我们最多可以向下移动到邻居的 walkableClimb 距离。
					// 从邻居跨度下的区域开始
					int neighborCeiling = neighborSpan ? (int)neighborSpan->smin : MAX_HEIGHTFIELD_HEIGHT;

					// Skip neighbour if the gap between the spans is too big.
					// 如果跨度之间的间隙太大，则跳过邻居。
					if (rcMin(ceiling, neighborCeiling) - floor >= walkableHeight)
					{
						lowestNeighborFloorDifference = (-walkableClimb - 1);
						break;
					}

					// For each span in the neighboring column...
					for (; neighborSpan != NULL; neighborSpan = neighborSpan->next)
					{
						const int neighborFloor = (int)neighborSpan->smax;
						neighborCeiling = neighborSpan->next ? (int)neighborSpan->next->smin : MAX_HEIGHTFIELD_HEIGHT;

						// Only consider neighboring areas that have enough overlap to be potentially traversable.
						// 仅考虑具有足够重叠以便可能可穿越的邻近区域。
						if (rcMin(ceiling, neighborCeiling) - rcMax(floor, neighborFloor) < walkableHeight)
						{
							// No space to traverse between them.
							continue;
						}

						const int neighborFloorDifference = neighborFloor - floor;
						lowestNeighborFloorDifference = rcMin(lowestNeighborFloorDifference, neighborFloorDifference);

						// Find min/max accessible neighbor height.
						// Only consider neighbors that are at most walkableClimb away.
						// 查找最小/最大可访问邻居高度。
						// 仅考虑距离最多为 walkableClimb 的邻居。
						if (rcAbs(neighborFloorDifference) <= walkableClimb)
						{
							// There is space to move to the neighbor cell and the slope isn't too much.
							// 有空间可以移动到相邻单元格并且坡度不是太大。
							lowestTraversableNeighborFloor = rcMin(lowestTraversableNeighborFloor, neighborFloor);
							highestTraversableNeighborFloor = rcMax(highestTraversableNeighborFloor, neighborFloor);
						}
						else if (neighborFloorDifference < -walkableClimb)
						{
							// We already know this will be considered a ledge span so we can early-out
							// 我们已经知道这将被视为一个边缘跨度，因此我们可以提前退出
							break;
						}
					}
				}

				// The current span is close to a ledge if the magnitude of the drop to any neighbour span is greater than the walkableClimb distance.
				// That is, there is a gap that is large enough to let an agent move between them, but the drop (surface slope) is too large to allow it.
				// (If this is the case, then biggestNeighborStepDown will be negative, so compare against the negative walkableClimb as a means of checking
				// the magnitude of the delta)
				// 如果当前跨度与任何相邻跨度之间的落差大于 walkableClimb 距离，则表示当前跨度接近边缘。
				// 也就是说，存在足够大的间隙，可以让代理在它们之间移动，但落差（表面坡度）太大，无法允许代理移动。
				// （如果是这种情况，则 biggestNeighborStepDown 将为负数，因此需要与负的 walkableClimb 进行比较，以检查
				// 增量的大小）
				if (lowestNeighborFloorDifference < -walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
				// If the difference between all neighbor floors is too large, this is a steep slope, so mark the span as an unwalkable ledge.
				// 如果所有相邻楼层之间的差异太大，这是一个陡坡，因此将跨度标记为不可行走的边缘。
				else if (highestTraversableNeighborFloor - lowestTraversableNeighborFloor > walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}

void rcFilterWalkableLowHeightSpans(rcContext* context, const int walkableHeight, rcHeightfield& heightfield)
{
	rcAssert(context);
	rcScopedTimer timer(context, RC_TIMER_FILTER_WALKABLE);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z*xSize]; span; span = span->next)
			{
				const int floor = (int)(span->smax);
				const int ceiling = span->next ? (int)(span->next->smin) : MAX_HEIGHTFIELD_HEIGHT;
				if (ceiling - floor < walkableHeight)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}
