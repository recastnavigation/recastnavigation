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

#include <cstdlib>

void rcFilterLowHangingWalkableObstacles(rcContext* context, const int walkableClimb, const rcHeightfield& heightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_LOW_OBSTACLES);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcSpan* previousSpan = nullptr;
			bool previousWasWalkable = false;
			unsigned char previousArea = RC_NULL_AREA;

			for (rcSpan* span = heightfield.spans[x + z * xSize]; span != nullptr; previousSpan = span, span = span->next)
			{
				const bool walkable = span->area != RC_NULL_AREA;
				// If current span is not walkable, but there is walkable
				// span just below it, mark the span above it walkable too.
				if (!walkable && previousWasWalkable)
				{
					if (rcAbs(static_cast<int>(span->smax) - static_cast<int>(previousSpan->smax)) <= walkableClimb)
					{
						span->area = previousArea;
					}
				}
				// Copy walkable flag so that it cannot propagate
				// past multiple non-walkable objects.
				previousWasWalkable = walkable;
				previousArea = span->area;
			}
		}
	}
}

void rcFilterLedgeSpans(rcContext* context, const int walkableHeight, const int walkableClimb,
                        const rcHeightfield& heightfield)
{
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	// Mark border spans.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			for (rcSpan* span = heightfield.spans[x + z * xSize]; span; span = span->next)
			{
				const int MAX_HEIGHT = 0xffff;
				// Skip non walkable spans.
				if (span->area == RC_NULL_AREA)
				{
					continue;
				}

				const int bot = static_cast<int>(span->smax);
				const int top = span->next ? static_cast<int>(span->next->smin) : MAX_HEIGHT;

				// Find neighbours minimum height.
				int minNeighborHeight = MAX_HEIGHT;

				// Min and max height of accessible neighbours.
				int accessibleNeighborMinHeight = span->smax;
				int accessibleNeighborMaxHeight = span->smax;

				for (int direction = 0; direction < 4; ++direction)
				{
					const int dx = x + rcGetDirOffsetX(direction);
					const int dy = z + rcGetDirOffsetY(direction);
					// Skip neighbours which are out of bounds.
					if (dx < 0 || dy < 0 || dx >= xSize || dy >= zSize)
					{
						minNeighborHeight = rcMin(minNeighborHeight, -walkableClimb - bot);
						continue;
					}

					// From minus infinity to the first span.
					const rcSpan* neighborSpan = heightfield.spans[dx + dy * xSize];
					int neighborBot = -walkableClimb;
					int neighborTop = neighborSpan ? static_cast<int>(neighborSpan->smin) : MAX_HEIGHT;
					
					// Skip neighbour if the gap between the spans is too small.
					if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
					{
						minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);
					}

					// Rest of the spans.
					for (neighborSpan = heightfield.spans[dx + dy * xSize]; neighborSpan; neighborSpan = neighborSpan->next)
					{
						neighborBot = static_cast<int>(neighborSpan->smax);
						neighborTop = neighborSpan->next ? static_cast<int>(neighborSpan->next->smin) : MAX_HEIGHT;
						
						// Skip neighbour if the gap between the spans is too small.
						if (rcMin(top, neighborTop) - rcMax(bot, neighborBot) > walkableHeight)
						{
							minNeighborHeight = rcMin(minNeighborHeight, neighborBot - bot);

							// Find min/max accessible neighbour height. 
							if (rcAbs(neighborBot - bot) <= walkableClimb)
							{
								if (neighborBot < accessibleNeighborMinHeight) accessibleNeighborMinHeight = neighborBot;
								if (neighborBot > accessibleNeighborMaxHeight) accessibleNeighborMaxHeight = neighborBot;
							}

						}
					}
				}

				// The current span is close to a ledge if the drop to any
				// neighbour span is less than the walkableClimb.
				// we are at steep slope, mark the span as ledge.
				if (minNeighborHeight < -walkableClimb || accessibleNeighborMaxHeight - accessibleNeighborMinHeight > walkableClimb)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}

void rcFilterWalkableLowHeightSpans(rcContext* context, const int walkableHeight, const rcHeightfield& heightfield)
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
				const int MAX_HEIGHT = 0xffff;
				const int bot = static_cast<int>(span->smax);
				const int top = span->next ? static_cast<int>(span->next->smin) : MAX_HEIGHT;
				if (top - bot < walkableHeight)
				{
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}
