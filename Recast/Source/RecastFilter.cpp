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

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastLog.h"
#include "RecastTimer.h"


// TODO: Missuses ledge flag, must be called before rcFilterLedgeSpans!
void rcFilterLowHangingWalkableObstacles(const int walkableClimb, rcHeightfield& solid)
{
	const int w = solid.width;
	const int h = solid.height;
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			rcSpan* ps = 0;
			for (rcSpan* s = solid.spans[x + y*w]; s; ps = s, s = s->next)
			{
				const bool walkable = (s->flags & RC_WALKABLE) != 0;
				const bool previousWalkable = ps && (ps->flags & RC_WALKABLE) != 0;
				// If current span is not walkable, but there is walkable
				// span just below it, mark the span above it walkable too.
				// Missuse the edge flag so that walkable flag cannot propagate
				// past multiple non-walkable objects.
				if (!walkable && previousWalkable)
				{
					if (rcAbs((int)s->smax - (int)ps->smax) <= walkableClimb)
						s->flags |= RC_LEDGE;
				}
			}
			// Transfer "fake ledges" to walkables.
			for (rcSpan* s = solid.spans[x + y*w]; s; ps = s, s = s->next)
			{
				if (s->flags & RC_LEDGE)
					s->flags |= RC_WALKABLE;
				s->flags &= ~RC_LEDGE;
			}
		}
	}
}
	
void rcFilterLedgeSpans(const int walkableHeight,
						const int walkableClimb,
						rcHeightfield& solid)
{
	rcTimeVal startTime = rcGetPerformanceTimer();

	const int w = solid.width;
	const int h = solid.height;
	const int MAX_HEIGHT = 0xffff;
	
	// Mark border spans.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			for (rcSpan* s = solid.spans[x + y*w]; s; s = s->next)
			{
				// Skip non walkable spans.
				if ((s->flags & RC_WALKABLE) == 0)
					continue;
				
				const int bot = (int)(s->smax);
				const int top = s->next ? (int)(s->next->smin) : MAX_HEIGHT;
				
				// Find neighbours minimum height.
				int minh = MAX_HEIGHT;

				// Min and max height of accessible neighbours.
				int asmin = s->smax;
				int asmax = s->smax;

				for (int dir = 0; dir < 4; ++dir)
				{
					int dx = x + rcGetDirOffsetX(dir);
					int dy = y + rcGetDirOffsetY(dir);
					// Skip neighbours which are out of bounds.
					if (dx < 0 || dy < 0 || dx >= w || dy >= h)
					{
						minh = rcMin(minh, -walkableClimb - bot);
						continue;
					}

					// From minus infinity to the first span.
					rcSpan* ns = solid.spans[dx + dy*w];
					int nbot = -walkableClimb;
					int ntop = ns ? (int)ns->smin : MAX_HEIGHT;
					// Skip neightbour if the gap between the spans is too small.
					if (rcMin(top,ntop) - rcMax(bot,nbot) > walkableHeight)
						minh = rcMin(minh, nbot - bot);
					
					// Rest of the spans.
					for (ns = solid.spans[dx + dy*w]; ns; ns = ns->next)
					{
						nbot = (int)ns->smax;
						ntop = ns->next ? (int)ns->next->smin : MAX_HEIGHT;
						// Skip neightbour if the gap between the spans is too small.
						if (rcMin(top,ntop) - rcMax(bot,nbot) > walkableHeight)
						{
							minh = rcMin(minh, nbot - bot);
						
							// Find min/max accessible neighbour height. 
							if (rcAbs(nbot - bot) <= walkableClimb)
							{
								if (nbot < asmin) asmin = nbot;
								if (nbot > asmax) asmax = nbot;
							}
							
						}
					}
				}
				
				// The current span is close to a ledge if the drop to any
				// neighbour span is less than the walkableClimb.
				if (minh < -walkableClimb)
					s->flags |= RC_LEDGE;
					
				// If the difference between all neighbours is too large,
				// we are at steep slope, mark the span as ledge.
				if ((asmax - asmin) > walkableClimb)
				{
					s->flags |= RC_LEDGE;
				}
			}
		}
	}
	
	rcTimeVal endTime = rcGetPerformanceTimer();
//	if (rcGetLog())
//		rcGetLog()->log(RC_LOG_PROGRESS, "Filter border: %.3f ms", rcGetDeltaTimeUsec(startTime, endTime)/1000.0f);
	if (rcGetBuildTimes())
		rcGetBuildTimes()->filterBorder += rcGetDeltaTimeUsec(startTime, endTime);
}	

void rcFilterWalkableLowHeightSpans(int walkableHeight,
									rcHeightfield& solid)
{
	rcTimeVal startTime = rcGetPerformanceTimer();
	
	const int w = solid.width;
	const int h = solid.height;
	const int MAX_HEIGHT = 0xffff;
	
	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			for (rcSpan* s = solid.spans[x + y*w]; s; s = s->next)
			{
				const int bot = (int)(s->smax);
				const int top = s->next ? (int)(s->next->smin) : MAX_HEIGHT;
				if ((top - bot) <= walkableHeight)
					s->flags &= ~RC_WALKABLE;
			}
		}
	}
	
	rcTimeVal endTime = rcGetPerformanceTimer();

//	if (rcGetLog())
//		rcGetLog()->log(RC_LOG_PROGRESS, "Filter walkable: %.3f ms", rcGetDeltaTimeUsec(startTime, endTime)/1000.0f);
	if (rcGetBuildTimes())
		rcGetBuildTimes()->filterWalkable += rcGetDeltaTimeUsec(startTime, endTime);
}
