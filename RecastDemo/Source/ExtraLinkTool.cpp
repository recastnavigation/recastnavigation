//
// Copyright (c) 2009 Mikko Mononen memon@inside.org
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
#include <string.h>
#include <float.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "ExtraLinkTool.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

static void drawCross(const float* p, const float s, const float dy)
{
	glBegin(GL_LINES);
	glVertex3f(p[0]-s,p[1]+dy,p[2]);
	glVertex3f(p[0]+s,p[1]+dy,p[2]);
	glVertex3f(p[0],p[1]-s+dy,p[2]);
	glVertex3f(p[0],p[1]+s+dy,p[2]);
	glVertex3f(p[0],p[1]+dy,p[2]-s);
	glVertex3f(p[0],p[1]+dy,p[2]+s);
	glEnd();
} 

ExtraLinkTool::ExtraLinkTool() :
	m_sample(0),
	m_hitPosSet(0)
{
}

ExtraLinkTool::~ExtraLinkTool()
{
}

void ExtraLinkTool::init(Sample* sample)
{
	m_sample = sample;
}

void ExtraLinkTool::reset()
{
	m_hitPosSet = false;
}

void ExtraLinkTool::handleMenu()
{
	if (m_hitPosSet)
	{
		imguiValue("Click to set link start.");
	}
	else
	{
		imguiValue("Click to set link end.");
	}
}

void ExtraLinkTool::handleClick(const float* p, bool shift)
{
	if (!m_sample) return;
	InputGeom* geom = m_sample->getInputGeom();
	if (!geom) return;

	if (shift)
	{
		// Delete
		float nearestDist = FLT_MAX;
		int nearestIndex = -1;
		for (int i = 0; i < geom->getExtraLinkCount(); ++i)
		{
			ExtraLink* link = geom->getExtraLink(i);
			float d;
			d = vdistSqr(p, link->spos);
			if (d < nearestDist)
			{
				nearestDist = d;
				nearestIndex = i;
			}
			d = vdistSqr(p, link->epos);
			if (d < nearestDist)
			{
				nearestDist = d;
				nearestIndex = i;
			}
		}
		
		if (nearestIndex != -1 &&
			sqrtf(nearestDist) < m_sample->getAgentRadius())
		{
			geom->deleteExtraLink(nearestIndex);
		}
	}
	else
	{
		// Create	
		if (!m_hitPosSet)
		{
			vcopy(m_hitPos, p);
			m_hitPosSet = true;
		}
		else
		{
			geom->addExtraLink(m_hitPos, p);
			m_hitPosSet = false;
		}
	}
	
}

void ExtraLinkTool::handleRender()
{
	if (m_hitPosSet)
	{
		const float s = m_sample->getAgentRadius();
		glColor4ub(0,0,0,128);
		glLineWidth(2.0f);
		drawCross(m_hitPos, s, 0.1f);
		glLineWidth(1.0f);
	}

	const float s = m_sample->getAgentRadius();

	InputGeom* geom = m_sample->getInputGeom();
	if (!geom) return;
	
	DebugDrawGL dd;
	const float linkCol[4] = {1,1,1,0.75f};
	for (int i = 0; i < geom->getExtraLinkCount(); ++i)
	{
		ExtraLink* link = geom->getExtraLink(i);

		duDebugDrawArc(&dd, link->spos, link->epos, linkCol, 2.0f);

		glLineWidth(2.0f);
		glColor4ub(0,0,0,255);
		drawCross(link->spos, s, 0.1f);
		drawCross(link->epos, s, 0.1f);
		glLineWidth(1.0f);		
	}
}

void ExtraLinkTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_hitPosSet && gluProject((GLdouble)m_hitPos[0], (GLdouble)m_hitPos[1], (GLdouble)m_hitPos[2],
								model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0,0,0,220));
	}
}
