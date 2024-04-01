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
				if (!walkable && previousWasWalkable && (int)span->smax - (int)previousSpan->smax <= walkableClimb)
				{
					span->area = previousAreaID;
				}

				// Copy the original walkable value regardless of whether we changed it.
				// This prevents multiple consecutive non-walkable spans from being erroneously marked as walkable.
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
					int neighborCeiling = neighborSpan ? (int)neighborSpan->smin : MAX_HEIGHTFIELD_HEIGHT;

					// Skip neighbour if the gap between the spans is too small.
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
						if (rcMin(ceiling, neighborCeiling) - rcMax(floor, neighborFloor) < walkableHeight)
						{
							// No space to traverse between them.
							continue;
						}

						const int neighborFloorDifference = neighborFloor - floor;
						lowestNeighborFloorDifference = rcMin(lowestNeighborFloorDifference, neighborFloorDifference);

						// Find min/max accessible neighbor height.
						// Only consider neighbors that are at most walkableClimb away.
						if (rcAbs(neighborFloorDifference) <= walkableClimb)
						{
							// There is space to move to the neighbor cell and the slope isn't too much.
							lowestTraversableNeighborFloor = rcMin(lowestTraversableNeighborFloor, neighborFloor);
							highestTraversableNeighborFloor = rcMax(highestTraversableNeighborFloor, neighborFloor);
						}
						else if (neighborFloorDifference < -walkableClimb)
						{
							// We already know this will be considered a ledge span so we can early-out
							break;
						}
					}
				}

				// The current span is close to a ledge if the magnitude of the drop to any neighbour span is greater than the walkableClimb distance.
				// That is, there is a gap that is large enough to let an agent move between them, but the drop (surface slope) is too large to allow it.
				// (If this is the case, then biggestNeighborStepDown will be negative, so compare against the negative walkableClimb as a means of checking
				// the magnitude of the delta)
				if (lowestNeighborFloorDifference < -walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
				// If the difference between all neighbor floors is too large, this is a steep slope, so mark the span as an unwalkable ledge.
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
