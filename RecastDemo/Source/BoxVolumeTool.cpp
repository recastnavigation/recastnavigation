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
#include "BoxVolumeTool.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

BoxVolumeTool::BoxVolumeTool() :
	m_sample(0),
	m_hitPosSet(0),
	m_areaType(1),
	m_boxHeight(5.0f)
{
}

BoxVolumeTool::~BoxVolumeTool()
{
}

void BoxVolumeTool::init(Sample* sample)
{
	m_sample = sample;
}

void BoxVolumeTool::reset()
{
	m_hitPosSet = false;
}

void BoxVolumeTool::handleMenu()
{
	imguiSlider("Box Height", &m_boxHeight, 0.1f, 20.0f, 0.1f);
	imguiSlider("Area Type", &m_areaType, 1.0f, 20.0f, 1.0f);

	if (!m_hitPosSet)
	{
		imguiValue("Click to set connection start.");
	}
	else
	{
		imguiValue("Click to set connection end.");
	}
}

void BoxVolumeTool::handleClick(const float* p, bool shift)
{
	if (!m_sample) return;
	InputGeom* geom = m_sample->getInputGeom();
	if (!geom) return;
	
	if (shift)
	{
		// Delete
		// Find nearest link end-point
		int nearestIndex = -1;
		const float* verts = geom->getBoxVolumeVerts();
		for (int i = 0; i < geom->getBoxVolumeCount(); ++i)
		{
			const float* bmin = &verts[(i*2+0)*3];
			const float* bmax = &verts[(i*2+1)*3];
			if (p[0] >= bmin[0] && p[0] <= bmax[0] &&
				p[1] >= bmin[1] && p[1] <= bmax[1] &&
				p[2] >= bmin[2] && p[2] <= bmax[2])
			{
				nearestIndex = i; // Each link has two vertices.
			}
		}
		// If end point close enough, delete it.
		if (nearestIndex != -1)
		{
			geom->deleteBoxVolume(nearestIndex);
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
			float bmin[3], bmax[3];
			vcopy(bmin, m_hitPos);
			vcopy(bmax, m_hitPos);
			vmin(bmin, p); 
			vmax(bmax, p);
			bmin[1] -= m_boxHeight/4.0f;
			bmax[1] = bmin[1]+m_boxHeight;
			geom->addBoxVolume(bmin, bmax, (unsigned char)m_areaType);
			
			m_hitPosSet = false;
		}
	}
	
}

void BoxVolumeTool::handleRender()
{
	DebugDrawGL dd;
	const float s = m_sample->getAgentRadius();
	
	if (m_hitPosSet)
		duDebugDrawCross(&dd, m_hitPos[0],m_hitPos[1]+0.1f,m_hitPos[2], s, duRGBA(0,0,0,128), 2.0f);
	
	InputGeom* geom = m_sample->getInputGeom();
	if (geom)
		geom->drawBoxVolumes(&dd, true);
}

void BoxVolumeTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_hitPosSet && gluProject((GLdouble)m_hitPos[0], (GLdouble)m_hitPos[1], (GLdouble)m_hitPos[2],
								  model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0,0,0,220));
	}
}
