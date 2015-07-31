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
	
	int minx = (int)((bmin[0]-chf.bmin[0])/chf.cs);
	int miny = (int)((bmin[1]-chf.bmin[1])/chf.ch);
	int minz = (int)((bmin[2]-chf.bmin[2])/chf.cs);
	int maxx = (int)((bmax[0]-chf.bmin[0])/chf.cs);
	int maxy = (int)((bmax[1]-chf.bmin[1])/chf.ch);
	int maxz = (int)((bmax[2]-chf.bmin[2])/chf.cs);
	
	if (maxx < 0) return;
	if (minx >= chf.width) return;
	if (maxz < 0) return;
	if (minz >= chf.height) return;
	
	if (minx < 0) minx = 0;
	if (maxx >= chf.width) maxx = chf.width-1;
	if (minz < 0) minz = 0;
	if (maxz >= chf.height) maxz = chf.height-1;	
	
	
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x+z*chf.width];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];
				
				if (chf.areas[i] == RC_NULL_AREA)
					continue;
				
				if ((int)s.minY >= miny && (int)s.minY <= maxy)
				{
					const float sx = chf.bmin[0] + (x+0.5f)*chf.cs; 
					const float sz = chf.bmin[2] + (z+0.5f)*chf.cs; 
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
