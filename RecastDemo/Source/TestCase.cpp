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

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include "TestCase.h"
#include "DetourNavMesh.h"
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "Recast.h"

#ifdef WIN32
#define snprintf _snprintf
#endif

TestCase::TestCase() :
	m_tests(0)
{
}

TestCase::~TestCase()
{
	Test* iter = m_tests;
	while (iter)
	{
		Test* next = iter->next;
		delete iter;
		iter = next;
	}
}


static char* parseRow(char* buf, char* bufEnd, char* row, int len)
{
	bool start = true;
	bool done = false;
	int n = 0;
	while (!done && buf < bufEnd)
	{
		char c = *buf;
		buf++;
		// multirow
		switch (c)
		{
			case '\n':
				if (start) break;
				done = true;
				break;
			case '\r':
				break;
			case '\t':
			case ' ':
				if (start) break;
			default:
				start = false;
				row[n++] = c;
				if (n >= len-1)
					done = true;
				break;
		}
	}
	row[n] = '\0';
	return buf;
}

static void copyName(char* dst, const char* src)
{
	// Skip white spaces
	while (*src && isspace(*src))
		src++;
	strcpy(dst, src);
}

bool TestCase::load(const char* filePath)
{
	char* buf = 0;
	FILE* fp = fopen(filePath, "rb");
	if (!fp)
		return false;
	fseek(fp, 0, SEEK_END);
	int bufSize = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	buf = new char[bufSize];
	if (!buf)
	{
		fclose(fp);
		return false;
	}
	fread(buf, bufSize, 1, fp);
	fclose(fp);

	char* src = buf;
	char* srcEnd = buf + bufSize;
	char row[512];
	while (src < srcEnd)
	{
		// Parse one row
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
		if (row[0] == 's')
		{
			// Sample name.
			copyName(m_sampleName, row+1);
		}
		else if (row[0] == 'f')
		{
			// File name.
			copyName(m_geomFileName, row+1);
		}
		else if (row[0] == 'p' && row[1] == 'f')
		{
			// Pathfind test.
			Test* test = new Test;
			memset(test, 0, sizeof(Test));
			test->type = TEST_PATHFIND;
			test->expand = false;
			test->next = m_tests;
			m_tests = test;
			sscanf(row+2, "%f %f %f %f %f %f %x %x",
				   &test->spos[0], &test->spos[1], &test->spos[2],
				   &test->epos[0], &test->epos[1], &test->epos[2],
				   &test->includeFlags, &test->excludeFlags);
		}
	}
	
	delete [] buf;

	return true;
}
		
void TestCase::resetTimes()
{
	for (Test* iter = m_tests; iter; iter = iter->next)
	{
		iter->findNearestPolyTime = 0;
		iter->findPathTime = 0;
		iter->findStraightPathTime = 0;
	}
}

void TestCase::doTests(dtNavMesh* navmesh)
{
	if (!navmesh)
		return;
	
	resetTimes();
	
	static const int MAX_POLYS = 256;
	dtPolyRef polys[MAX_POLYS];
	float straight[MAX_POLYS*3];
	const float polyPickExt[3] = {2,4,2};
	
	for (Test* iter = m_tests; iter; iter = iter->next)
	{
		delete [] iter->polys;
		iter->polys = 0;
		iter->npolys = 0;
		delete [] iter->straight;
		iter->straight = 0;
		iter->nstraight = 0;
		
		dtQueryFilter filter;
		filter.includeFlags = (unsigned short)iter->includeFlags;
		filter.excludeFlags = (unsigned short)iter->excludeFlags;
	
		// Find start points
		rcTimeVal findNearestPolyStart = rcGetPerformanceTimer();
		
		dtPolyRef startRef = navmesh->findNearestPoly(iter->spos, polyPickExt, &filter, 0);
		dtPolyRef endRef = navmesh->findNearestPoly(iter->epos, polyPickExt, &filter, 0);

		rcTimeVal findNearestPolyEnd = rcGetPerformanceTimer();
		iter->findNearestPolyTime += rcGetDeltaTimeUsec(findNearestPolyStart, findNearestPolyEnd);

		if (!startRef || ! endRef)
			continue;
	
		// Find path
		rcTimeVal findPathStart = rcGetPerformanceTimer();

		iter->npolys = navmesh->findPath(startRef, endRef, iter->spos, iter->epos, &filter, polys, MAX_POLYS);
		
		rcTimeVal findPathEnd = rcGetPerformanceTimer();
		iter->findPathTime += rcGetDeltaTimeUsec(findPathStart, findPathEnd);
		
		// Find straight path
		if (iter->npolys)
		{
			rcTimeVal findStraightPathStart = rcGetPerformanceTimer();
			
			iter->nstraight = navmesh->findStraightPath(iter->spos, iter->epos, polys, iter->npolys,
														  straight, 0, 0, MAX_POLYS);
			rcTimeVal findStraightPathEnd = rcGetPerformanceTimer();
			iter->findStraightPathTime += rcGetDeltaTimeUsec(findStraightPathStart, findStraightPathEnd);
		}
		
		// Copy results
		if (iter->npolys)
		{
			iter->polys = new dtPolyRef[iter->npolys];
			memcpy(iter->polys, polys, sizeof(dtPolyRef)*iter->npolys);
		}
		if (iter->nstraight)
		{
			iter->straight = new float[iter->nstraight*3];
			memcpy(iter->straight, straight, sizeof(float)*3*iter->nstraight);
		}
		
	}
}

void TestCase::handleRender()
{
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	for (Test* iter = m_tests; iter; iter = iter->next)
	{
		float dir[3];
		rcVsub(dir, iter->epos, iter->spos);
		rcVnormalize(dir);
		glColor4ub(128,25,0,192);
		glVertex3f(iter->spos[0],iter->spos[1]-0.3f,iter->spos[2]);
		glVertex3f(iter->spos[0],iter->spos[1]+0.3f,iter->spos[2]);
		glVertex3f(iter->spos[0],iter->spos[1]+0.3f,iter->spos[2]);
		glVertex3f(iter->spos[0]+dir[0]*0.3f,iter->spos[1]+0.3f+dir[1]*0.3f,iter->spos[2]+dir[2]*0.3f);
		glColor4ub(51,102,0,129);
		glVertex3f(iter->epos[0],iter->epos[1]-0.3f,iter->epos[2]);
		glVertex3f(iter->epos[0],iter->epos[1]+0.3f,iter->epos[2]);
		
		if (iter->expand)
			glColor4ub(255,192,0,255);
		else
			glColor4ub(0,0,0,64);
			
		for (int i = 0; i < iter->nstraight-1; ++i)
		{
			glVertex3f(iter->straight[i*3+0],iter->straight[i*3+1]+0.3f,iter->straight[i*3+2]);
			glVertex3f(iter->straight[(i+1)*3+0],iter->straight[(i+1)*3+1]+0.3f,iter->straight[(i+1)*3+2]);
		}
	}
	glEnd();
	glLineWidth(1.0f);
}

bool TestCase::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	char text[64];
	int n = 0;

	static const float LABEL_DIST = 1.0f;

	for (Test* iter = m_tests; iter; iter = iter->next)
	{
		float pt[3], dir[3];
		if (iter->nstraight)
		{
			rcVcopy(pt, &iter->straight[3]);
			if (rcVdist(pt, iter->spos) > LABEL_DIST)
			{
				rcVsub(dir, pt, iter->spos);
				rcVnormalize(dir);
				rcVmad(pt, iter->spos, dir, LABEL_DIST);
			}
			pt[1]+=0.5f;
		}
		else
		{
			rcVsub(dir, iter->epos, iter->spos);
			rcVnormalize(dir);
			rcVmad(pt, iter->spos, dir, LABEL_DIST);
			pt[1]+=0.5f;
		}
		
		if (gluProject((GLdouble)pt[0], (GLdouble)pt[1], (GLdouble)pt[2],
					   model, proj, view, &x, &y, &z))
		{
			snprintf(text, 64, "Path %d\n", n);
			unsigned int col = imguiRGBA(0,0,0,128);
			if (iter->expand)
				col = imguiRGBA(255,192,0,220);
			imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, text, col);
		}
		n++;
	}
	
	static int resScroll = 0;
	bool mouseOverMenu = imguiBeginScrollArea("Test Results", 10, view[3] - 10 - 350, 200, 350, &resScroll);
//		mouseOverMenu = true;
		
	n = 0;
	for (Test* iter = m_tests; iter; iter = iter->next)
	{
		snprintf(text, 64, "Path %d\n", n);
		
		if (imguiCollapse(text, iter->expand))
			iter->expand = !iter->expand;
		if (iter->expand)
		{
			snprintf(text, 64, "Poly: %.4f ms\n", (float)iter->findNearestPolyTime/1000.0f);
			imguiValue(text);

			snprintf(text, 64, "Path: %.4f ms\n", (float)iter->findPathTime/1000.0f);
			imguiValue(text);

			snprintf(text, 64, "Straight: %.4f ms\n", (float)iter->findStraightPathTime/1000.0f);
			imguiValue(text);
		}
		rcTimeVal total = iter->findNearestPolyTime + iter->findPathTime + iter->findStraightPathTime;
		snprintf(text, 64, "Total: %.4f ms\n", (float)total/1000.0f);
		imguiValue(text);
		
		
//		imguiDrawText(10, 700-n*20, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255,220));
		n++;
	}

	imguiEndScrollArea();
	
	return mouseOverMenu;
}
