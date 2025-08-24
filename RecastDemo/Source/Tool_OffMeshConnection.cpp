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

#include "Tool_OffMeshConnection.h"

#include "DetourDebugDraw.h"
#include "InputGeom.h"
#include "Recast.h"
#include "SDL_opengl.h"
#include "Sample.h"
#include "imguiHelpers.h"

#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif

#include <imgui.h>

#include <cfloat>
#include <cmath>

void OffMeshConnectionTool::init(Sample* newSample)
{
	if (sample == newSample)
	{
		return;
	}

	sample = newSample;
	oldFlags = sample->navMeshDrawFlags;
	sample->navMeshDrawFlags &= ~DU_DRAWNAVMESH_OFFMESHCONS;
}

void OffMeshConnectionTool::reset()
{
	hitPosSet = false;
}

void OffMeshConnectionTool::drawMenuUI()
{
	ImGui::Checkbox("Bidirectional", &bidirectional);
}

void OffMeshConnectionTool::onClick(const float* /*rayStartPos*/, const float* rayHitPos, bool shift)
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
		// Find nearest link end-point
		float nearestDist = FLT_MAX;
		int nearestIndex = -1;
		const float* verts = geom->offMeshConVerts;
		for (int i = 0; i < geom->offMeshConCount * 2; ++i)
		{
			const float distance = rcVdistSqr(rayHitPos, &verts[i * 3]);
			if (distance < nearestDist)
			{
				nearestDist = distance;
				nearestIndex = i / 2;  // Each link has two vertices.
			}
		}

		// If end point close enough, delete it.
		if (nearestIndex != -1 && sqrtf(nearestDist) < sample->agentRadius)
		{
			geom->deleteOffMeshConnection(nearestIndex);
		}
	}
	else
	{
		// Create
		if (!hitPosSet)
		{
			rcVcopy(hitPos, rayHitPos);
			hitPosSet = true;
		}
		else
		{
			geom->addOffMeshConnection(
				hitPos,
				rayHitPos,
				sample->agentRadius,
				bidirectional ? 1 : 0,
				SAMPLE_POLYAREA_JUMP,
				SAMPLE_POLYFLAGS_JUMP);
			hitPosSet = false;
		}
	}
}

void OffMeshConnectionTool::render()
{
	duDebugDraw& dd = sample->debugDraw;

	if (hitPosSet)
	{
		duDebugDrawCross(&dd, hitPos[0], hitPos[1] + 0.1f, hitPos[2], sample->agentRadius, duRGBA(0, 0, 0, 128), 2.0f);
	}

	if (InputGeom* geom = sample->inputGeometry)
	{
		geom->drawOffMeshConnections(&dd, true);
	}
}

void OffMeshConnectionTool::drawOverlayUI()
{
	if (hitPosSet)
	{
		DrawWorldspaceText(hitPos[0], hitPos[1], hitPos[2], IM_COL32(0, 0, 0, 220), "Start", true, 25);
	}

	// Tool help
	const char* setMessage = "LMB: Set connection end point and finish.";
	const char* connectMessage = "LMB: Create new connection. SHIFT+LMB: Delete existing connection, click near start or end point.";
	DrawScreenspaceText(280, 40, IM_COL32(255, 255, 255, 192), hitPosSet ? setMessage : connectMessage);
}
