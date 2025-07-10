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

#include "SDL_opengl.h"

#include <float.h>
#include <cmath>

#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif
#include "DetourDebugDraw.h"
#include "InputGeom.h"
#include "OffMeshConnectionTool.h"
#include "Recast.h"
#include "Sample.h"
#include "imgui.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

void OffMeshConnectionTool::init(Sample* newSample)
{
	if (sample != newSample)
	{
		sample = newSample;
		oldFlags = sample->getNavMeshDrawFlags();
		sample->setNavMeshDrawFlags(oldFlags & ~DU_DRAWNAVMESH_OFFMESHCONS);
	}
}

void OffMeshConnectionTool::reset()
{
	hitPosSet = false;
}

void OffMeshConnectionTool::handleMenu()
{
#if 0
	if (imguiCheck("One Way", !bidir))
	{
		bidir = false;
	}
	if (imguiCheck("Bidirectional", bidir))
	{
		bidir = true;
	}
#endif
}

void OffMeshConnectionTool::handleClick(const float* /*rayStartPos*/, const float* rayHitPos, bool shift)
{
	if (!sample)
	{
		return;
	}

	InputGeom* geom = sample->getInputGeom();
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
		const float* verts = geom->getOffMeshConnectionVerts();
		for (int i = 0; i < geom->getOffMeshConnectionCount() * 2; ++i)
		{
			float dist = rcVdistSqr(rayHitPos, &verts[i * 3]);
			if (dist < nearestDist)
			{
				nearestDist = dist;
				nearestIndex = i / 2;  // Each link has two vertices.
			}
		}

		// If end point close enough, delete it.
		if (nearestIndex != -1 && sqrtf(nearestDist) < sample->getAgentRadius())
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
				sample->getAgentRadius(),
				bidir ? 1 : 0,
				SAMPLE_POLYAREA_JUMP,
				SAMPLE_POLYFLAGS_JUMP);
			hitPosSet = false;
		}
	}
}

void OffMeshConnectionTool::handleToggle() {}

void OffMeshConnectionTool::handleStep() {}

void OffMeshConnectionTool::handleUpdate(const float /*dt*/) {}

void OffMeshConnectionTool::handleRender()
{
	duDebugDraw& dd = sample->getDebugDraw();

	if (hitPosSet)
	{
		duDebugDrawCross(&dd, hitPos[0], hitPos[1] + 0.1f, hitPos[2], sample->getAgentRadius(), duRGBA(0, 0, 0, 128), 2.0f);
	}

	if (InputGeom* geom = sample->getInputGeom())
	{
		geom->drawOffMeshConnections(&dd, true);
	}
}

void OffMeshConnectionTool::handleRenderOverlay(double* proj, double* model, int* view)
{
#if 0
	GLdouble x, y, z;

	// Draw start and end point labels
	if (hitPosSet && gluProject((GLdouble)hitPos[0], (GLdouble)hitPos[1], (GLdouble)hitPos[2], model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y - 25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0, 0, 0, 220));
	}

	// Tool help
	const int h = view[3];
	if (!hitPosSet)
	{
		imguiDrawText(
			280,
			h - 40,
			IMGUI_ALIGN_LEFT,
			"LMB: Create new connection.  SHIFT+LMB: Delete existing connection, click close to start or end point.",
			imguiRGBA(255, 255, 255, 192));
	}
	else
	{
		imguiDrawText(
			280,
			h - 40,
			IMGUI_ALIGN_LEFT,
			"LMB: Set connection end point and finish.",
			imguiRGBA(255, 255, 255, 192));
	}
#endif
}
