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
#include "Sample_Debug.h"
#include "InputGeom.h"
#include "Recast.h"
#include "DetourNavMesh.h"
#include "RecastLog.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "RecastDump.h"
#include "imgui.h"
#include "SDL.h"
#include "SDL_opengl.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


Sample_Debug::Sample_Debug() :
	m_chf(0)
{
	resetCommonSettings();

	// Test
	m_chf = new rcCompactHeightfield;
	if (!duReadCompactHeightfield(*m_chf, "test.chf"))
	{
		delete m_chf;
		m_chf = 0;
	}
}

Sample_Debug::~Sample_Debug()
{
	delete m_chf;
}

void Sample_Debug::handleSettings()
{
}

void Sample_Debug::handleTools()
{
}

void Sample_Debug::handleDebugMode()
{
}

void Sample_Debug::handleRender()
{
	DebugDrawGL dd;
	
	if (m_chf)
		duDebugDrawCompactHeightfieldRegions(&dd, *m_chf);
}

void Sample_Debug::handleRenderOverlay(double* proj, double* model, int* view)
{
}

void Sample_Debug::handleMeshChanged(InputGeom* geom)
{
	m_geom = geom;
}

const float* Sample_Debug::getBoundsMin()
{
	if (!m_chf) return 0;
	return m_chf->bmin;
}

const float* Sample_Debug::getBoundsMax()
{
	if (!m_chf) return 0;
	return m_chf->bmax;
}

void Sample_Debug::handleClick(const float* p, bool shift)
{
	if (m_tool)
		m_tool->handleClick(p, shift);
}

void Sample_Debug::handleStep()
{
	if (m_tool)
		m_tool->handleStep();
}

bool Sample_Debug::handleBuild()
{
	return true;
}
