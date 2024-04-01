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
#include "RecastAlloc.h"
#include "RecastAssert.h"

#include <string.h> // for memcpy and memset

/// Sorts the given data in-place using insertion sort.
///
/// @param	data		The data to sort
/// @param	dataLength	The number of elements in @p data
static void insertSort(unsigned char* data, const int dataLength)
{
	for (int valueIndex = 1; valueIndex < dataLength; valueIndex++)
	{
		const unsigned char value = data[valueIndex];
		int insertionIndex;
		for (insertionIndex = valueIndex - 1; insertionIndex >= 0 && data[insertionIndex] > value; insertionIndex--)
		{
			// Shift over values
			data[insertionIndex + 1] = data[insertionIndex];
		}
		
		// Insert the value in sorted order.
		data[insertionIndex + 1] = value;
	}
}

// TODO (graham): This is duplicated in the ConvexVolumeTool in RecastDemo
/// Checks if a point is contained within a polygon
///
/// @param[in]	numVerts	Number of vertices in the polygon
/// @param[in]	verts		The polygon vertices
/// @param[in]	point		The point to check
/// @returns true if the point lies within the polygon, false otherwise.
static bool pointInPoly(int numVerts, const float* verts, const float* point)
{
	bool inPoly = false;
	for (int i = 0, j = numVerts - 1; i < numVerts; j = i++)
	{
		const float* vi = &verts[i * 3];
		const float* vj = &verts[j * 3];

		if ((vi[2] > point[2]) == (vj[2] > point[2]))
		{
			continue;
		}

		if (point[0] >= (vj[0] - vi[0]) * (point[2] - vi[2]) / (vj[2] - vi[2]) + vi[0])
		{
			continue;
		}
		inPoly = !inPoly;
	}
	return inPoly;
}

bool rcErodeWalkableArea(rcContext* context, const int erosionRadius, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context != NULL);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int& zStride = xSize; // For readability

	rcScopedTimer timer(context, RC_TIMER_ERODE_AREA);

	unsigned char* distanceToBoundary = (unsigned char*)rcAlloc(sizeof(unsigned char) * compactHeightfield.spanCount,
	                                                            RC_ALLOC_TEMP);
	if (!distanceToBoundary)
	{
		context->log(RC_LOG_ERROR, "erodeWalkableArea: Out of memory 'dist' (%d).", compactHeightfield.spanCount);
		return false;
	}
	memset(distanceToBoundary, 0xff, sizeof(unsigned char) * compactHeightfield.spanCount);
	
	// Mark boundary cells.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			for (int spanIndex = (int)cell.index, maxSpanIndex = (int)(cell.index + cell.count); spanIndex < maxSpanIndex; ++spanIndex)
			{
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					distanceToBoundary[spanIndex] = 0;
					continue;
				}
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Check that there is a non-null adjacent span in each of the 4 cardinal directions.
				int neighborCount = 0;
				for (int direction = 0; direction < 4; ++direction)
				{
					const int neighborConnection = rcGetCon(span, direction);
					if (neighborConnection == RC_NOT_CONNECTED)
					{
						break;
					}
					
					const int neighborX = x + rcGetDirOffsetX(direction);
					const int neighborZ = z + rcGetDirOffsetY(direction);
					const int neighborSpanIndex = (int)compactHeightfield.cells[neighborX + neighborZ * zStride].index + neighborConnection;
					
					if (compactHeightfield.areas[neighborSpanIndex] == RC_NULL_AREA)
					{
						break;
					}
					neighborCount++;
				}
				
				// At least one missing neighbour, so this is a boundary cell.
				if (neighborCount != 4)
				{
					distanceToBoundary[spanIndex] = 0;
				}
			}
		}
	}
	
	unsigned char newDistance;
	
	// Pass 1
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				if (rcGetCon(span, 0) != RC_NOT_CONNECTED)
				{
					// (-1,0)
					const int aX = x + rcGetDirOffsetX(0);
					const int aY = z + rcGetDirOffsetY(0);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 0);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (unsigned char)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (-1,-1)
					if (rcGetCon(aSpan, 3) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(3);
						const int bY = aY + rcGetDirOffsetY(3);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 3);
						newDistance = (unsigned char)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
				if (rcGetCon(span, 3) != RC_NOT_CONNECTED)
				{
					// (0,-1)
					const int aX = x + rcGetDirOffsetX(3);
					const int aY = z + rcGetDirOffsetY(3);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 3);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (unsigned char)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (1,-1)
					if (rcGetCon(aSpan, 2) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(2);
						const int bY = aY + rcGetDirOffsetY(2);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 2);
						newDistance = (unsigned char)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
			}
		}
	}

	// Pass 2
	for (int z = zSize - 1; z >= 0; --z)
	{
		for (int x = xSize - 1; x >= 0; --x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				if (rcGetCon(span, 2) != RC_NOT_CONNECTED)
				{
					// (1,0)
					const int aX = x + rcGetDirOffsetX(2);
					const int aY = z + rcGetDirOffsetY(2);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 2);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (unsigned char)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (1,1)
					if (rcGetCon(aSpan, 1) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(1);
						const int bY = aY + rcGetDirOffsetY(1);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 1);
						newDistance = (unsigned char)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
				if (rcGetCon(span, 1) != RC_NOT_CONNECTED)
				{
					// (0,1)
					const int aX = x + rcGetDirOffsetX(1);
					const int aY = z + rcGetDirOffsetY(1);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 1);
					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (unsigned char)rcMin((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (-1,1)
					if (rcGetCon(aSpan, 0) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(0);
						const int bY = aY + rcGetDirOffsetY(0);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 0);
						newDistance = (unsigned char)rcMin((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
			}
		}
	}

	const unsigned char minBoundaryDistance = (unsigned char)(erosionRadius * 2);
	for (int spanIndex = 0; spanIndex < compactHeightfield.spanCount; ++spanIndex)
	{
		if (distanceToBoundary[spanIndex] < minBoundaryDistance)
		{
			compactHeightfield.areas[spanIndex] = RC_NULL_AREA;
		}
	}

	rcFree(distanceToBoundary);
	
	return true;
}

bool rcMedianFilterWalkableArea(rcContext* context, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);
	
	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	rcScopedTimer timer(context, RC_TIMER_MEDIAN_AREA);

	unsigned char* areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * compactHeightfield.spanCount, RC_ALLOC_TEMP);
	if (!areas)
	{
		context->log(RC_LOG_ERROR, "medianFilterWalkableArea: Out of memory 'areas' (%d).",
		             compactHeightfield.spanCount);
		return false;
	}
	memset(areas, 0xff, sizeof(unsigned char) * compactHeightfield.spanCount);

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const rcCompactSpan& span = compactHeightfield.spans[spanIndex];
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					areas[spanIndex] = compactHeightfield.areas[spanIndex];
					continue;
				}

				unsigned char neighborAreas[9];
				for (int neighborIndex = 0; neighborIndex < 9; ++neighborIndex)
				{
					neighborAreas[neighborIndex] = compactHeightfield.areas[spanIndex];
				}

				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(span, dir) == RC_NOT_CONNECTED)
					{
						continue;
					}
					
					const int aX = x + rcGetDirOffsetX(dir);
					const int aZ = z + rcGetDirOffsetY(dir);
					const int aIndex = (int)compactHeightfield.cells[aX + aZ * zStride].index + rcGetCon(span, dir);
					if (compactHeightfield.areas[aIndex] != RC_NULL_AREA)
					{
						neighborAreas[dir * 2 + 0] = compactHeightfield.areas[aIndex];
					}

					const rcCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					const int dir2 = (dir + 1) & 0x3;
					const int neighborConnection2 = rcGetCon(aSpan, dir2);
					if (neighborConnection2 != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(dir2);
						const int bZ = aZ + rcGetDirOffsetY(dir2);
						const int bIndex = (int)compactHeightfield.cells[bX + bZ * zStride].index + neighborConnection2;
						if (compactHeightfield.areas[bIndex] != RC_NULL_AREA)
						{
							neighborAreas[dir * 2 + 1] = compactHeightfield.areas[bIndex];
						}
					}
				}
				insertSort(neighborAreas, 9);
				areas[spanIndex] = neighborAreas[4];
			}
		}
	}

	memcpy(compactHeightfield.areas, areas, sizeof(unsigned char) * compactHeightfield.spanCount);

	rcFree(areas);

	return true;
}

void rcMarkBoxArea(rcContext* context, const float* boxMinBounds, const float* boxMaxBounds, unsigned char areaId,
                   rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_BOX_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Find the footprint of the box area in grid cell coordinates. 
	int minX = (int)((boxMinBounds[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int minY = (int)((boxMinBounds[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int minZ = (int)((boxMinBounds[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);
	int maxX = (int)((boxMaxBounds[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int maxY = (int)((boxMaxBounds[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int maxZ = (int)((boxMaxBounds[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);

	// Early-out if the box is outside the bounds of the grid.
	if (maxX < 0) { return; }
	if (minX >= xSize) { return; }
	if (maxZ < 0) { return; }
	if (minZ >= zSize) { return; }

	// Clamp relevant bound coordinates to the grid.
	if (minX < 0) { minX = 0; }
	if (maxX >= xSize) { maxX = xSize - 1; }
	if (minZ < 0) { minZ = 0; }
	if (maxZ >= zSize) { maxZ = zSize - 1; }

	// Mark relevant cells.
	for (int z = minZ; z <= maxZ; ++z)
	{
		for (int x = minX; x <= maxX; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if the span is outside the box extents.
				if ((int)span.y < minY || (int)span.y > maxY)
				{
					continue;
				}

				// Skip if the span has been removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					continue;
				}

				// Mark the span.
				compactHeightfield.areas[spanIndex] = areaId;
			}
		}
	}
}

void rcMarkConvexPolyArea(rcContext* context, const float* verts, const int numVerts,
						  const float minY, const float maxY, unsigned char areaId,
						  rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_CONVEXPOLY_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Compute the bounding box of the polygon
	float bmin[3];
	float bmax[3];
	rcVcopy(bmin, verts);
	rcVcopy(bmax, verts);
	for (int i = 1; i < numVerts; ++i)
	{
		rcVmin(bmin, &verts[i * 3]);
		rcVmax(bmax, &verts[i * 3]);
	}
	bmin[1] = minY;
	bmax[1] = maxY;

	// Compute the grid footprint of the polygon 
	int minx = (int)((bmin[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int miny = (int)((bmin[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int minz = (int)((bmin[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);
	int maxx = (int)((bmax[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int maxy = (int)((bmax[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int maxz = (int)((bmax[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);

	// Early-out if the polygon lies entirely outside the grid.
	if (maxx < 0) { return; }
    if (minx >= xSize) { return; }
    if (maxz < 0) { return; }
    if (minz >= zSize) { return; }

	// Clamp the polygon footprint to the grid
    if (minx < 0) { minx = 0; }
    if (maxx >= xSize) { maxx = xSize - 1; }
    if (minz < 0) { minz = 0; }
    if (maxz >= zSize) { maxz = zSize - 1; }

	// TODO: Optimize.
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if span is removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					continue;
				}

				// Skip if y extents don't overlap.
				if ((int)span.y < miny || (int)span.y > maxy)
				{
					continue;
				}

				const float point[] = {
					compactHeightfield.bmin[0] + ((float)x + 0.5f) * compactHeightfield.cs,
					0,
					compactHeightfield.bmin[2] + ((float)z + 0.5f) * compactHeightfield.cs
				};
				
				if (pointInPoly(numVerts, verts, point))
				{
					compactHeightfield.areas[spanIndex] = areaId;
				}
			}
		}
	}
}

static const float EPSILON = 1e-6f;

/// Normalizes the vector if the length is greater than zero.
/// If the magnitude is zero, the vector is unchanged.
/// @param[in,out]	v	The vector to normalize. [(x, y, z)]
static void rcVsafeNormalize(float* v)
{
	const float sqMag = rcSqr(v[0]) + rcSqr(v[1]) + rcSqr(v[2]);
	if (sqMag > EPSILON)
	{
		const float inverseMag = 1.0f / rcSqrt(sqMag);
		v[0] *= inverseMag;
		v[1] *= inverseMag;
		v[2] *= inverseMag;
	}
}

int rcOffsetPoly(const float* verts, const int numVerts, const float offset, float* outVerts, const int maxOutVerts)
{
	// Defines the limit at which a miter becomes a bevel.
	// Similar in behavior to https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/stroke-miterlimit
	const float MITER_LIMIT = 1.20f;

	int numOutVerts = 0;

	for (int vertIndex = 0; vertIndex < numVerts; vertIndex++)
	{
        // Grab three vertices of the polygon.
		const int vertIndexA = (vertIndex + numVerts - 1) % numVerts;
		const int vertIndexB = vertIndex;
		const int vertIndexC = (vertIndex + 1) % numVerts;
		const float* vertA = &verts[vertIndexA * 3];
		const float* vertB = &verts[vertIndexB * 3];
		const float* vertC = &verts[vertIndexC * 3];

        // From A to B on the x/z plane
		float prevSegmentDir[3];
		rcVsub(prevSegmentDir, vertB, vertA);
		prevSegmentDir[1] = 0; // Squash onto x/z plane
		rcVsafeNormalize(prevSegmentDir);
		
        // From B to C on the x/z plane
		float currSegmentDir[3];
		rcVsub(currSegmentDir, vertC, vertB);
		currSegmentDir[1] = 0; // Squash onto x/z plane
		rcVsafeNormalize(currSegmentDir);

        // The y component of the cross product of the two normalized segment directions.
        // The X and Z components of the cross product are both zero because the two
        // segment direction vectors fall within the x/z plane.
        float cross = currSegmentDir[0] * prevSegmentDir[2] - prevSegmentDir[0] * currSegmentDir[2];

        // CCW perpendicular vector to AB.  The segment normal.
		const float prevSegmentNormX = -prevSegmentDir[2];
		const float prevSegmentNormZ = prevSegmentDir[0];

        // CCW perpendicular vector to BC.  The segment normal.
		const float currSegmentNormX = -currSegmentDir[2];
		const float currSegmentNormZ = currSegmentDir[0];

        // Average the two segment normals to get the proportional miter offset for B.
        // This isn't normalized because it's defining the distance and direction the corner will need to be
        // adjusted proportionally to the edge offsets to properly miter the adjoining edges.
		float cornerMiterX = (prevSegmentNormX + currSegmentNormX) * 0.5f;
		float cornerMiterZ = (prevSegmentNormZ + currSegmentNormZ) * 0.5f;
        const float cornerMiterSqMag = rcSqr(cornerMiterX) + rcSqr(cornerMiterZ);

        // If the magnitude of the segment normal average is less than about .69444,
        // the corner is an acute enough angle that the result should be beveled.
        const bool bevel = cornerMiterSqMag * MITER_LIMIT * MITER_LIMIT < 1.0f;

        // Scale the corner miter so it's proportional to how much the corner should be offset compared to the edges.
		if (cornerMiterSqMag > EPSILON)
		{
			const float scale = 1.0f / cornerMiterSqMag;
            cornerMiterX *= scale;
            cornerMiterZ *= scale;
		}

		if (bevel && cross < 0.0f) // If the corner is convex and an acute enough angle, generate a bevel.
		{
			if (numOutVerts + 2 > maxOutVerts)
			{
				return 0;
			}

            // Generate two bevel vertices at a distances from B proportional to the angle between the two segments.
            // Move each bevel vertex out proportional to the given offset.
			float d = (1.0f - (prevSegmentDir[0] * currSegmentDir[0] + prevSegmentDir[2] * currSegmentDir[2])) * 0.5f;

			outVerts[numOutVerts * 3 + 0] = vertB[0] + (-prevSegmentNormX + prevSegmentDir[0] * d) * offset;
			outVerts[numOutVerts * 3 + 1] = vertB[1];
			outVerts[numOutVerts * 3 + 2] = vertB[2] + (-prevSegmentNormZ + prevSegmentDir[2] * d) * offset;
			numOutVerts++;

			outVerts[numOutVerts * 3 + 0] = vertB[0] + (-currSegmentNormX - currSegmentDir[0] * d) * offset;
			outVerts[numOutVerts * 3 + 1] = vertB[1];
			outVerts[numOutVerts * 3 + 2] = vertB[2] + (-currSegmentNormZ - currSegmentDir[2] * d) * offset;
			numOutVerts++;
		}
		else
		{
			if (numOutVerts + 1 > maxOutVerts)
			{
				return 0;
			}

            // Move B along the miter direction by the specified offset.
			outVerts[numOutVerts * 3 + 0] = vertB[0] - cornerMiterX * offset;
			outVerts[numOutVerts * 3 + 1] = vertB[1];
			outVerts[numOutVerts * 3 + 2] = vertB[2] - cornerMiterZ * offset;
			numOutVerts++;
		}
	}

	return numOutVerts;
}

void rcMarkCylinderArea(rcContext* context, const float* position, const float radius, const float height,
                        unsigned char areaId, rcCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_CYLINDER_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Compute the bounding box of the cylinder
	const float cylinderBBMin[] =
	{
		position[0] - radius,
		position[1],
		position[2] - radius
	};
	const float cylinderBBMax[] =
	{
		position[0] + radius,
		position[1] + height,
		position[2] + radius
	};

	// Compute the grid footprint of the cylinder
	int minx = (int)((cylinderBBMin[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int miny = (int)((cylinderBBMin[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int minz = (int)((cylinderBBMin[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);
	int maxx = (int)((cylinderBBMax[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int maxy = (int)((cylinderBBMax[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int maxz = (int)((cylinderBBMax[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);

	// Early-out if the cylinder is completely outside the grid bounds.
    if (maxx < 0) { return; }
    if (minx >= xSize) { return; }
    if (maxz < 0) { return; }
    if (minz >= zSize) { return; }

	// Clamp the cylinder bounds to the grid.
    if (minx < 0) { minx = 0; }
    if (maxx >= xSize) { maxx = xSize - 1; }
    if (minz < 0) { minz = 0; }
    if (maxz >= zSize) { maxz = zSize - 1; }

	const float radiusSq = radius * radius;

	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);

			const float cellX = compactHeightfield.bmin[0] + ((float)x + 0.5f) * compactHeightfield.cs;
			const float cellZ = compactHeightfield.bmin[2] + ((float)z + 0.5f) * compactHeightfield.cs;
			const float deltaX = cellX - position[0];
            const float deltaZ = cellZ - position[2];

			// Skip this column if it's too far from the center point of the cylinder.
            if (rcSqr(deltaX) + rcSqr(deltaZ) >= radiusSq)
            {
	            continue;
            }

			// Mark all overlapping spans
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				rcCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if span is removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					continue;
				}

				// Mark if y extents overlap.
				if ((int)span.y >= miny && (int)span.y <= maxy)
				{
					compactHeightfield.areas[spanIndex] = areaId;
				}
			}
		}
	}
}
