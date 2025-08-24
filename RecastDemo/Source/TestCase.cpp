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

#include "TestCase.h"

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "SDL_opengl.h"
#include "SampleInterfaces.h"
#include "imguiHelpers.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif
#include "PerfTimer.h"

#include <imgui.h>

#ifdef WIN32
#	define snprintf _snprintf
#endif

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
			if (start)
			{
				break;
			}
			done = true;
			break;
		case '\r':
			break;
		case '\t':
		case ' ':
			if (start)
			{
				break;
			}
			// else falls through
		default:
			start = false;
			row[n++] = c;
			if (n >= len - 1)
			{
				done = true;
			}
			break;
		}
	}
	row[n] = '\0';
	return buf;
}

static void copyName(std::string& dst, const char* src)
{
	// Skip white spaces
	while (*src && isspace(*src))
	{
		src++;
	}
	dst = src;
}

bool TestCase::load(const std::string& filePath)
{
	FileIO file;
	if (!file.openForRead(filePath.c_str()))
	{
		// ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filePath.c_str());
		return false;
	}

	size_t bufferLen = file.getFileSize();
	char* buffer = new char[bufferLen];

	if (!file.read(buffer, bufferLen))
	{
		// ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filePath.c_str());
		return false;
	}

	char* src = buffer;
	char* srcEnd = buffer + bufferLen;
	char row[512];
	while (src < srcEnd)
	{
		// Parse one row
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, sizeof(row) / sizeof(char));
		if (row[0] == 's')
		{
			// Sample name.
			copyName(sampleName, row + 1);
		}
		else if (row[0] == 'f')
		{
			// File name.
			copyName(geomFileName, row + 1);
		}
		else if (row[0] == 'p' && row[1] == 'f')
		{
			// Pathfind test.
			Test test;
			test.type = TestCase::TestType::PATHFIND;
			test.expand = false;
			sscanf(
				row + 2,
				"%f %f %f %f %f %f %hx %hx",
				&test.spos[0],
				&test.spos[1],
				&test.spos[2],
				&test.epos[0],
				&test.epos[1],
				&test.epos[2],
				&test.includeFlags,
				&test.excludeFlags);
			tests.emplace_back(std::move(test));
		}
		else if (row[0] == 'r' && row[1] == 'c')
		{
			// Pathfind test.
			Test test;
			test.type = TestCase::TestType::RAYCAST;
			test.expand = false;
			sscanf(
				row + 2,
				"%f %f %f %f %f %f %hx %hx",
				&test.spos[0],
				&test.spos[1],
				&test.spos[2],
				&test.epos[0],
				&test.epos[1],
				&test.epos[2],
				&test.includeFlags,
				&test.excludeFlags);
			tests.emplace_back(std::move(test));
		}
	}

	delete[] buffer;
	return true;
}

void TestCase::resetTimes()
{
	for (Test& test : tests)
	{
		test.findNearestPolyTime = 0;
		test.findPathTime = 0;
		test.findStraightPathTime = 0;
	}
}

void TestCase::doTests(dtNavMesh* navmesh, dtNavMeshQuery* navquery)
{
	if (!navmesh || !navquery)
	{
		return;
	}

	resetTimes();

	static const int MAX_POLYS = 256;
	dtPolyRef polys[MAX_POLYS];
	float straight[MAX_POLYS * 3];
	const float polyPickExt[3] = {2, 4, 2};

	for (auto& test : tests)
	{
		test.polys.clear();
		test.straight.clear();

		dtQueryFilter filter;
		filter.setIncludeFlags(test.includeFlags);
		filter.setExcludeFlags(test.excludeFlags);

		// Find start points
		TimeVal findNearestPolyStart = getPerfTime();

		dtPolyRef startRef, endRef;
		navquery->findNearestPoly(test.spos, polyPickExt, &filter, &startRef, test.nspos);
		navquery->findNearestPoly(test.epos, polyPickExt, &filter, &endRef, test.nepos);

		TimeVal findNearestPolyEnd = getPerfTime();
		test.findNearestPolyTime += getPerfTimeUsec(findNearestPolyEnd - findNearestPolyStart);

		if (!startRef || !endRef)
		{
			continue;
		}

		if (test.type == TestCase::TestType::PATHFIND)
		{
			// Find path
			TimeVal findPathStart = getPerfTime();

			int numPolys = 0;
			navquery->findPath(startRef, endRef, test.spos, test.epos, &filter, polys, &numPolys, MAX_POLYS);

			TimeVal findPathEnd = getPerfTime();
			test.findPathTime += getPerfTimeUsec(findPathEnd - findPathStart);

			// Find straight path
			int numStraight = 0;
			if (numPolys)
			{
				TimeVal findStraightPathStart = getPerfTime();

				navquery->findStraightPath(
					test.spos,
					test.epos,
					polys,
					numPolys,
					straight,
					0,
					0,
					&numStraight,
					MAX_POLYS);
				TimeVal findStraightPathEnd = getPerfTime();
				test.findStraightPathTime += getPerfTimeUsec(findStraightPathEnd - findStraightPathStart);
			}

			// Copy results
			if (numPolys)
			{
				test.polys.resize(numPolys);
				memcpy(test.polys.data(), polys, sizeof(dtPolyRef) * numPolys);
			}
			if (numStraight > 0)
			{
				test.straight.resize(numStraight * 3);
				memcpy(test.straight.data(), straight, sizeof(float) * 3 * numStraight);
			}
		}
		else if (test.type == TestCase::TestType::RAYCAST)
		{
			float t = 0;
			float hitNormal[3];
			float hitPos[3];

			test.straight.resize(2 * 3);
			test.straight[0] = test.spos[0];
			test.straight[1] = test.spos[1];
			test.straight[2] = test.spos[2];

			TimeVal findPathStart = getPerfTime();

			int numPolys = 0;
			navquery->raycast(startRef, test.spos, test.epos, &filter, &t, hitNormal, polys, &numPolys, MAX_POLYS);

			TimeVal findPathEnd = getPerfTime();
			test.findPathTime += getPerfTimeUsec(findPathEnd - findPathStart);

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
			if (numPolys > 0)
			{
				float h = 0;
				navquery->getPolyHeight(polys[numPolys - 1], hitPos, &h);
				hitPos[1] = h;
			}
			dtVcopy(&test.straight[3], hitPos);

			if (numPolys)
			{
				test.polys.resize(numPolys);
				memcpy(test.polys.data(), polys, sizeof(dtPolyRef) * numPolys);
			}
		}
	}

	printf("Test Results:\n");
	int n = 0;
	for (auto& test : tests)
	{
		const int total = test.findNearestPolyTime + test.findPathTime + test.findStraightPathTime;
		printf(" - Path %02d:     %.4f ms\n", n, (float)total / 1000.0f);
		printf("    - poly:     %.4f ms\n", (float)test.findNearestPolyTime / 1000.0f);
		printf("    - path:     %.4f ms\n", (float)test.findPathTime / 1000.0f);
		printf("    - straight: %.4f ms\n", (float)test.findStraightPathTime / 1000.0f);
		n++;
	}
}

void TestCase::handleRender()
{
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	for (auto& test : tests)
	{
		float dir[3];
		dtVsub(dir, test.epos, test.spos);
		dtVnormalize(dir);
		glColor4ub(128, 25, 0, 192);
		glVertex3f(test.spos[0], test.spos[1] - 0.3f, test.spos[2]);
		glVertex3f(test.spos[0], test.spos[1] + 0.3f, test.spos[2]);
		glVertex3f(test.spos[0], test.spos[1] + 0.3f, test.spos[2]);
		glVertex3f(test.spos[0] + dir[0] * 0.3f, test.spos[1] + 0.3f + dir[1] * 0.3f, test.spos[2] + dir[2] * 0.3f);
		glColor4ub(51, 102, 0, 129);
		glVertex3f(test.epos[0], test.epos[1] - 0.3f, test.epos[2]);
		glVertex3f(test.epos[0], test.epos[1] + 0.3f, test.epos[2]);

		if (test.expand)
		{
			const float s = 0.1f;
			glColor4ub(255, 32, 0, 128);
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
		{
			glColor4ub(255, 192, 0, 255);
		}
		else
		{
			glColor4ub(0, 0, 0, 64);
		}

		int numStraight = static_cast<int>(test.straight.size()) / 3;
		for (int i = 0; i < numStraight - 1; ++i)
		{
			glVertex3f(test.straight[i * 3 + 0], test.straight[i * 3 + 1] + 0.3f, test.straight[i * 3 + 2]);
			glVertex3f(
				test.straight[(i + 1) * 3 + 0],
				test.straight[(i + 1) * 3 + 1] + 0.3f,
				test.straight[(i + 1) * 3 + 2]);
		}
	}
	glEnd();
	glLineWidth(1.0f);
}

bool TestCase::handleRenderOverlay(double* proj, double* model, int* view)
{
	char text[64];
	int n = 0;

	static constexpr float LABEL_DIST = 1.0f;

	for (auto& test : tests)
	{
		float pt[3];
		float dir[3];
		if (!test.straight.empty())
		{
			dtVcopy(pt, &test.straight[3]);
			if (dtVdist(pt, test.spos) > LABEL_DIST)
			{
				dtVsub(dir, pt, test.spos);
				dtVnormalize(dir);
				dtVmad(pt, test.spos, dir, LABEL_DIST);
			}
			pt[1] += 0.5f;
		}
		else
		{
			dtVsub(dir, test.epos, test.spos);
			dtVnormalize(dir);
			dtVmad(pt, test.spos, dir, LABEL_DIST);
			pt[1] += 0.5f;
		}


		snprintf(text, 64, "Path %d\n", n);
		unsigned int color = test.expand ? IM_COL32(255, 192, 0, 220) : IM_COL32(0, 0, 0, 128);
		DrawWorldspaceText(pt[0], pt[1], pt[2], color, text, true, 25);
		n++;
	}

	ImGui::SetNextWindowPos(ImVec2(10, static_cast<float>(view[3]) - 10 - 350), ImGuiCond_Always);  // Position in screen space
	ImGui::SetNextWindowSize(ImVec2(200, 350), ImGuiCond_Always);                                   // Size of the window
	ImGui::Begin("Test Results");

	n = 0;
	for (auto& test : tests)
	{
		const int total = test.findNearestPolyTime + test.findPathTime + test.findStraightPathTime;
		snprintf(text, 64, "Path %d", n);

		if (ImGui::CollapsingHeader(text, ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGui::Text("Total: %.4f ms", static_cast<float>(total) / 1000.0f);
			ImGui::Text("Poly: %.4f ms", static_cast<float>(test.findNearestPolyTime) / 1000.0f);
			ImGui::Text("Path: %.4f ms", static_cast<float>(test.findPathTime) / 1000.0f);
			ImGui::Text("Straight: %.4f ms", static_cast<float>(test.findStraightPathTime) / 1000.0f);

			ImGui::Separator();
		}
	}

	ImGui::End();

	return false;
}
