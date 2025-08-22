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

#include "Tool_ConvexVolume.h"

#include "InputGeom.h"
#include "Recast.h"
#include "Sample.h"
#include "imguiHelpers.h"

#include <imgui.h>

#include <cfloat>

#ifdef WIN32
#	define snprintf _snprintf
#endif

// Quick and dirty convex hull.

namespace {
/// Returns true if 'c' is left of line 'a'-'b'.
inline bool left(const float* a, const float* b, const float* c)
{
	const float u1 = b[0] - a[0];
	const float v1 = b[2] - a[2];
	const float u2 = c[0] - a[0];
	const float v2 = c[2] - a[2];
	return u1 * v2 - v1 * u2 < 0;
}

/// Returns true if 'a' is more lower-left than 'b'.
inline bool comparePoints(const float* a, const float* b)
{
	if (a[0] < b[0])
	{
		return true;
	}
	if (a[0] > b[0])
	{
		return false;
	}
	if (a[2] < b[2])
	{
		return true;
	}
	if (a[2] > b[2])
	{
		return false;
	}
	return false;
}

/// Calculates convex hull on xz-plane of points on 'pts',
/// stores the indices of the resulting hull in 'out' and
/// returns number of points on hull.
int convexhull(const float* pts, int npts, int* out)
{
	// Find lower-leftmost point.
	int hull = 0;
	for (int i = 1; i < npts; ++i)
	{
		if (comparePoints(&pts[i * 3], &pts[hull * 3]))
		{
			hull = i;
		}
	}
	// Gift wrap hull.
	int endpt = 0;
	int i = 0;
	do
	{
		out[i++] = hull;
		endpt = 0;
		for (int j = 1; j < npts; ++j)
		{
			if (hull == endpt || left(&pts[hull * 3], &pts[endpt * 3], &pts[j * 3]))
			{
				endpt = j;
			}
		}
		hull = endpt;
	} while (endpt != out[0]);

	return i;
}

bool pointInPoly(int nvert, const float* verts, const float* p)
{
	bool result = false;
	for (int i = 0, j = nvert - 1; i < nvert; j = i++)
	{
		const float* vi = &verts[i * 3];
		const float* vj = &verts[j * 3];
		if (((vi[2] > p[2]) != (vj[2] > p[2])) && (p[0] < (vj[0] - vi[0]) * (p[2] - vi[2]) / (vj[2] - vi[2]) + vi[0]))
		{
			result = !result;
		}
	}
	return result;
}
}

void ConvexVolumeTool::drawMenuUI()
{
	ImGui::SliderFloat("##Shape Height", &boxHeight, 0.1f, 20.0f, "Shape Height = %f");
	ImGui::SliderFloat("##Shape Descent", &boxDescent, 0.1f, 20.0f, "Shape Descent = %f");
	ImGui::SliderFloat("##Poly Offset", &polyOffset, 0.0f, 10.0f, "Poly Offset = %f");

	ImGui::Text("Area Type");

	ImGui::Indent();
	if (ImGui::RadioButton("Ground", areaType == SAMPLE_POLYAREA_GROUND))
	{
		areaType = SAMPLE_POLYAREA_GROUND;
	}
	if (ImGui::RadioButton("Water", areaType == SAMPLE_POLYAREA_WATER))
	{
		areaType = SAMPLE_POLYAREA_WATER;
	}
	if (ImGui::RadioButton("Road", areaType == SAMPLE_POLYAREA_ROAD))
	{
		areaType = SAMPLE_POLYAREA_ROAD;
	}
	if (ImGui::RadioButton("Door", areaType == SAMPLE_POLYAREA_DOOR))
	{
		areaType = SAMPLE_POLYAREA_DOOR;
	}
	if (ImGui::RadioButton("Grass", areaType == SAMPLE_POLYAREA_GRASS))
	{
		areaType = SAMPLE_POLYAREA_GRASS;
	}
	if (ImGui::RadioButton("Jump", areaType == SAMPLE_POLYAREA_JUMP))
	{
		areaType = SAMPLE_POLYAREA_JUMP;
	}
	ImGui::Unindent();

	ImGui::Separator();

	if (ImGui::Button("Clear Shape"))
	{
		numPoints = 0;
		numHull = 0;
	}
}

void ConvexVolumeTool::onClick(const float* /*s*/, const float* p, bool shift)
{
	if (!sample)
	{
		return;
	}
	InputGeom* geom = sample->inputGeometry;
	if (!geom)
	{
		return;
	}

	if (shift)
	{
		// Delete
		int nearestIndex = -1;
		const ConvexVolume* vols = geom->convexVolumes;
		for (int i = 0; i < geom->convexVolumeCount; ++i)
		{
			if (pointInPoly(vols[i].nverts, vols[i].verts, p) && p[1] >= vols[i].hmin && p[1] <= vols[i].hmax)
			{
				nearestIndex = i;
			}
		}
		// If end point close enough, delete it.
		if (nearestIndex != -1)
		{
			geom->deleteConvexVolume(nearestIndex);
		}
	}
	else
	{
		// Create

		// If clicked on that last pt, create the shape.
		if (numPoints && rcVdistSqr(p, &points[(numPoints - 1) * 3]) < rcSqr(0.2f))
		{
			if (numHull > 2)
			{
				// Create shape.
				float verts[MAX_PTS * 3];
				for (int i = 0; i < numHull; ++i)
				{
					rcVcopy(&verts[i * 3], &points[hull[i] * 3]);
				}

				float minh = FLT_MAX, maxh = 0;
				for (int i = 0; i < numHull; ++i)
				{
					minh = rcMin(minh, verts[i * 3 + 1]);
				}
				minh -= boxDescent;
				maxh = minh + boxHeight;

				if (polyOffset > 0.01f)
				{
					float offset[MAX_PTS * 2 * 3];
					int noffset = rcOffsetPoly(verts, numHull, polyOffset, offset, MAX_PTS * 2);
					if (noffset > 0)
					{
						geom->addConvexVolume(offset, noffset, minh, maxh, (unsigned char)areaType);
					}
				}
				else
				{
					geom->addConvexVolume(verts, numHull, minh, maxh, (unsigned char)areaType);
				}
			}

			numPoints = 0;
			numHull = 0;
		}
		else
		{
			// Add new point
			if (numPoints < MAX_PTS)
			{
				rcVcopy(&points[numPoints * 3], p);
				numPoints++;
				// Update hull.
				if (numPoints > 1)
				{
					numHull = convexhull(points, numPoints, hull);
				}
				else
				{
					numHull = 0;
				}
			}
		}
	}
}

void ConvexVolumeTool::render()
{
	duDebugDraw& dd = sample->debugDraw;

	// Find height extent of the shape.
	float minh = FLT_MAX, maxh = 0;
	for (int i = 0; i < numPoints; ++i)
	{
		minh = rcMin(minh, points[i * 3 + 1]);
	}
	minh -= boxDescent;
	maxh = minh + boxHeight;

	dd.begin(DU_DRAW_POINTS, 4.0f);
	for (int i = 0; i < numPoints; ++i)
	{
		unsigned int col = duRGBA(255, 255, 255, 255);
		if (i == numPoints - 1)
		{
			col = duRGBA(240, 32, 16, 255);
		}
		dd.vertex(points[i * 3 + 0], points[i * 3 + 1] + 0.1f, points[i * 3 + 2], col);
	}
	dd.end();

	dd.begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0, j = numHull - 1; i < numHull; j = i++)
	{
		const float* vi = &points[hull[j] * 3];
		const float* vj = &points[hull[i] * 3];
		dd.vertex(vj[0], minh, vj[2], duRGBA(255, 255, 255, 64));
		dd.vertex(vi[0], minh, vi[2], duRGBA(255, 255, 255, 64));
		dd.vertex(vj[0], maxh, vj[2], duRGBA(255, 255, 255, 64));
		dd.vertex(vi[0], maxh, vi[2], duRGBA(255, 255, 255, 64));
		dd.vertex(vj[0], minh, vj[2], duRGBA(255, 255, 255, 64));
		dd.vertex(vj[0], maxh, vj[2], duRGBA(255, 255, 255, 64));
	}
	dd.end();
}

void ConvexVolumeTool::drawOverlayUI(double* /*proj*/, double* /*model*/, int* /*view*/)
{
	// Tool help
	if (!numPoints)
	{
		DrawScreenspaceText(
			280.0f,
			40.0f,
			IM_COL32(255, 255, 255, 192),
			"LMB: Create new shape.  SHIFT+LMB: Delete existing shape (click inside a shape).");
	}
	else
	{
		DrawScreenspaceText(
			280.0f,
			40.0f,
			IM_COL32(255, 255, 255, 192),
			"Click LMB to add new points. Click on the red point to finish the shape.");
		DrawScreenspaceText(
			280.0f,
			60.0f,
			IM_COL32(255, 255, 255, 192),
			"The shape will be convex hull of all added points.");
	}
}
