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

#include <iostream>
#include <iomanip>
#include <sstream>

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include "TestCase.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "PerfTimer.h"

#ifdef WIN32
#define snprintf _snprintf
#endif

TestCase::TestCase()
{
}

TestCase::~TestCase()
{
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
				if (!start)
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

static void copyName(string& dst, const char* src)
{
	// Skip white spaces
	while (*src && isspace(*src))
		src++;
	dst = src;
}

bool TestCase::load(string filePath)
{
	std::ifstream file(filePath, std::ios::binary);
	file.seekg(0, std::ios::end);
	std::streamsize size = file.tellg();
	file.seekg(0, std::ios::beg);

	char* buf = new char[static_cast<unsigned int>(size)];
	if (!file.read(buf, size))
		return false;

	char* src = buf;
	char* srcEnd = buf + size;
	char row[512];
	while (src < srcEnd)
	{
		// Parse one row
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, sizeof(row) / sizeof(char));
		if (row[0] == 's')
		{
			// Sample name.
			copyName(m_sampleName, row + 1);
		}
		else if (row[0] == 'f')
		{
			// File name.
			copyName(m_geomFileName, row + 1);
		}
		else if (row[0] == 'p' && row[1] == 'f')
		{
			// Pathfind test.
			Test test(TEST_PATHFIND, false);
			std::istringstream ss(row + 2);
			ss >> test.spos[0] >> test.spos[1] >> test.spos[2] 
			   >> test.epos[0] >> test.epos[1] >> test.epos[2]
			   >> std::hex
			   >> test.includeFlags >> test.excludeFlags;

			m_tests.push_back(std::move(test));
		}
		else if (row[0] == 'r' && row[1] == 'c')
		{
			// Raycast test.
			Test test(TEST_RAYCAST, false);
			std::istringstream ss(row + 2);
			ss >> test.spos[0] >> test.spos[1] >> test.spos[2]
				>> test.epos[0] >> test.epos[1] >> test.epos[2]
				>> std::hex
				>> test.includeFlags >> test.excludeFlags;
			m_tests.push_back(std::move(test));
		}
	}
	
	delete[] buf;

	return true;
}
		
void TestCase::resetTimes()
{
	for (Test& test : m_tests)
	{
		test.findNearestPolyTime = 0;
		test.findPathTime = 0;
		test.findStraightPathTime = 0;
	}
}

void TestCase::doTests(dtNavMesh* navmesh, dtNavMeshQuery* navquery)
{
	if (!navmesh || !navquery)
		return;
	
	resetTimes();
	
	static const int MAX_POLYS = 256;
	dtPolyRef polys[MAX_POLYS];
	float straight[MAX_POLYS * 3];
	const float polyPickExt[3] = {2, 4, 2};
	
	for (Test& test : m_tests) {
        test.polys.clear();
		test.npolys = 0;
        test.straight.clear();
		test.nstraight = 0;
		
		dtQueryFilter filter;
		filter.setIncludeFlags((unsigned short)test.includeFlags);
		filter.setExcludeFlags((unsigned short)test.excludeFlags);
	
		// Find start points
		TimeVal findNearestPolyStart = getPerfTime();
		
		dtPolyRef startRef;
		navquery->findNearestPoly(test.spos, polyPickExt, &filter, &startRef, test.nspos);

		dtPolyRef endRef;
		navquery->findNearestPoly(test.epos, polyPickExt, &filter, &endRef, test.nepos);

		TimeVal findNearestPolyEnd = getPerfTime();
		test.findNearestPolyTime += getPerfDeltaTimeUsec(findNearestPolyStart, findNearestPolyEnd);

		if (!startRef || ! endRef)
			continue;
	
		if (test.type == TEST_PATHFIND)
		{
			// Find path
			TimeVal findPathStart = getPerfTime();

			navquery->findPath(startRef, endRef, test.spos, test.epos, &filter, polys, &test.npolys, MAX_POLYS);
			
			TimeVal findPathEnd = getPerfTime();
			test.findPathTime += getPerfDeltaTimeUsec(findPathStart, findPathEnd);
		
			// Find straight path
			if (test.npolys)
			{
				TimeVal findStraightPathStart = getPerfTime();
				
				navquery->findStraightPath(test.spos, test.epos, polys, test.npolys,
										   straight, 0, 0, &test.nstraight, MAX_POLYS);
				TimeVal findStraightPathEnd = getPerfTime();
				test.findStraightPathTime += getPerfDeltaTimeUsec(findStraightPathStart, findStraightPathEnd);
			}
		
			// Copy results
			if (test.npolys)
			{
                test.polys.resize(test.npolys);
				memcpy(test.polys.data(), polys, sizeof(dtPolyRef) * test.npolys);
			}
			if (test.nstraight)
			{
                test.straight.resize(test.nstraight * 3);
				memcpy(test.straight.data(), straight, sizeof(float) * 3 * test.nstraight);
			}
		}
		else if (test.type == TEST_RAYCAST)
		{
			float t = 0;
			float hitNormal[3], hitPos[3];
			
            test.straight.resize(2 * 3);
			test.nstraight = 2;
			
			test.straight[0] = test.spos[0];
			test.straight[1] = test.spos[1];
			test.straight[2] = test.spos[2];
			
			TimeVal findPathStart = getPerfTime();
			
			navquery->raycast(startRef, test.spos, test.epos, &filter, &t, hitNormal, polys, &test.npolys, MAX_POLYS);

			TimeVal findPathEnd = getPerfTime();
			test.findPathTime += getPerfDeltaTimeUsec(findPathStart, findPathEnd);

			if (t > 1)
			{
				// No hit
				dtVcopy(hitPos, test.epos);
			}
			else
			{
				// Hit
				dtVlerp(hitPos, test.spos, test.epos, t);
			}
			// Adjust height.
			if (test.npolys > 0)
			{
				float h = 0;
				navquery->getPolyHeight(polys[test.npolys - 1], hitPos, &h);
				hitPos[1] = h;
			}
			dtVcopy(&test.straight[3], hitPos);

			if (test.npolys)
			{
                test.polys.resize(test.npolys);
				memcpy(test.polys.data(), polys, sizeof(dtPolyRef)*test.npolys);
			}
		}
	}

	std::cout << "Test Results:\n";
	int n = 0;
	for (Test& test : m_tests)
	{
		const int total = test.findNearestPolyTime + test.findPathTime + test.findStraightPathTime;
		printf(" - Path %02d:     %.4f ms\n", n, (float)total/1000.0f);
		printf("    - poly:     %.4f ms\n", (float)test.findNearestPolyTime/1000.0f);
		printf("    - path:     %.4f ms\n", (float)test.findPathTime/1000.0f);
		printf("    - straight: %.4f ms\n", (float)test.findStraightPathTime/1000.0f);
		n++;
	}
}

void TestCase::handleRender()
{
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	for (Test& test : m_tests)
	{
		float dir[3];
		dtVsub(dir, test.epos, test.spos);
		dtVnormalize(dir);
		glColor4ub(128, 25, 0, 192);
		glVertex3f(test.spos[0], test.spos[1]-0.3f, test.spos[2]);
		glVertex3f(test.spos[0], test.spos[1]+0.3f, test.spos[2]);
		glVertex3f(test.spos[0], test.spos[1]+0.3f, test.spos[2]);
		glVertex3f(test.spos[0] + dir[0] * 0.3f, test.spos[1] + 0.3f + dir[1] * 0.3f, test.spos[2] + dir[2] * 0.3f);
		glColor4ub(51, 102, 0, 129);
		glVertex3f(test.epos[0], test.epos[1] - 0.3f, test.epos[2]);
		glVertex3f(test.epos[0], test.epos[1] + 0.3f, test.epos[2]);

		if (test.expand)
		{
			const float s = 0.1f;
			glColor4ub(255,32,0,128);
			glVertex3f(test.spos[0] - s, test.spos[1], test.spos[2]);
			glVertex3f(test.spos[0] + s, test.spos[1], test.spos[2]);
			glVertex3f(test.spos[0], test.spos[1], test.spos[2] - s);
			glVertex3f(test.spos[0], test.spos[1], test.spos[2] + s);
			glColor4ub(255, 192, 0, 255);
			glVertex3f(test.nspos[0] - s, test.nspos[1], test.nspos[2]);
			glVertex3f(test.nspos[0] + s, test.nspos[1], test.nspos[2]);
			glVertex3f(test.nspos[0], test.nspos[1], test.nspos[2] - s);
			glVertex3f(test.nspos[0], test.nspos[1], test.nspos[2] + s);
			
			glColor4ub(255, 32, 0, 128);
			glVertex3f(test.epos[0] - s, test.epos[1], test.epos[2]);
			glVertex3f(test.epos[0] + s, test.epos[1], test.epos[2]);
			glVertex3f(test.epos[0], test.epos[1], test.epos[2] - s);
			glVertex3f(test.epos[0], test.epos[1], test.epos[2] + s);
			glColor4ub(255, 192, 0, 255);
			glVertex3f(test.nepos[0] - s, test.nepos[1], test.nepos[2]);
			glVertex3f(test.nepos[0] + s, test.nepos[1], test.nepos[2]);
			glVertex3f(test.nepos[0], test.nepos[1], test.nepos[2] - s);
			glVertex3f(test.nepos[0], test.nepos[1], test.nepos[2] + s);
		}
		
		if (test.expand)
			glColor4ub(255, 192, 0, 255);
		else
			glColor4ub(0, 0, 0, 64);
			
		for (int i = 0; i < test.nstraight-1; ++i)
		{
			glVertex3f(test.straight[i * 3 + 0], test.straight[i * 3 + 1] + 0.3f, test.straight[i * 3 + 2]);
			glVertex3f(test.straight[(i + 1) * 3 + 0], test.straight[(i + 1) * 3 + 1] + 0.3f, test.straight[(i + 1) * 3 + 2]);
		}
	}
	glEnd();
	glLineWidth(1.0f);
}

bool TestCase::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	string subtext;
	int n = 0;

	static const float LABEL_DIST = 1.0f;

	for (Test& test : m_tests)
	{
		float pt[3], dir[3];
		if (test.nstraight)
		{
			dtVcopy(pt, &test.straight[3]);
			if (dtVdist(pt, test.spos) > LABEL_DIST)
			{
				dtVsub(dir, pt, test.spos);
				dtVnormalize(dir);
				dtVmad(pt, test.spos, dir, LABEL_DIST);
			}
			pt[1]+=0.5f;
		}
		else
		{
			dtVsub(dir, test.epos, test.spos);
			dtVnormalize(dir);
			dtVmad(pt, test.spos, dir, LABEL_DIST);
			pt[1]+=0.5f;
		}
		
		if (gluProject((GLdouble)pt[0], (GLdouble)pt[1], (GLdouble)pt[2], model, proj, view, &x, &y, &z))
		{
			char text[64];
			sprintf(text, "Path %d\n", n);
			unsigned int col = imguiRGBA(0,0,0,128);
			if (test.expand)
				col = imguiRGBA(255,192,0,220);
			imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, text, col);
		}
		n++;
	}
	
	static int resScroll = 0;
	bool mouseOverMenu = imguiBeginScrollArea("Test Results", 10, view[3] - 10 - 350, 200, 350, &resScroll);
	
	char text[64];
	n = 0;
	for (Test& test : m_tests)
	{
		const int total = test.findNearestPolyTime + test.findPathTime + test.findStraightPathTime;
		std::ostringstream os;
		os << std::setprecision(4) << (float)total / 1000.0f << " ms";
		subtext = os.str();

		sprintf(text, "Path %d", n);

		if (imguiCollapse(text, subtext.c_str(), test.expand))
			test.expand = !test.expand;
		if (test.expand)
		{
			snprintf(text, 64, "Poly: %.4f ms", (float)test.findNearestPolyTime / 1000.0f);
			imguiValue(text);

			snprintf(text, 64, "Path: %.4f ms", (float)test.findPathTime / 1000.0f);
			imguiValue(text);

			snprintf(text, 64, "Straight: %.4f ms", (float)test.findStraightPathTime / 1000.0f);
			imguiValue(text);
			
			imguiSeparator();
		}
		
		n++;
	}

	imguiEndScrollArea();
	
	return mouseOverMenu;
}
