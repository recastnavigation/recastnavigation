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

#pragma once

#include "DetourNavMesh.h"

#include <string>

class TestCase
{
	enum TestType
	{
		TEST_PATHFIND,
		TEST_RAYCAST
	};

	struct Test
	{
		Test() = default;
		Test(const Test&) = delete;
		Test(Test&&) = delete;
		Test& operator=(const Test&) = delete;
		Test& operator=(Test&&) = delete;
		~Test()
		{
			delete[] straight;
			delete[] polys;
		}

		TestType type{};
		float spos[3]{};
		float epos[3]{};
		float nspos[3]{};
		float nepos[3]{};
		float radius = 0;
		unsigned short includeFlags = 0;
		unsigned short excludeFlags = 0;
		bool expand = false;

		float* straight = nullptr;
		int nstraight = 0;
		dtPolyRef* polys = nullptr;
		int npolys = 0;

		int findNearestPolyTime = 0;
		int findPathTime = 0;
		int findStraightPathTime = 0;

		Test* next = nullptr;
	};

	std::string m_sampleName;
	std::string m_geomFileName;
	Test* m_tests = nullptr;

	void resetTimes();

public:
	TestCase() = default;
	TestCase(const TestCase&) = delete;
	TestCase(TestCase&&) = delete;
	TestCase& operator=(const TestCase&) = delete;
	TestCase& operator=(TestCase&&) = delete;
	~TestCase();

	bool load(const std::string& filePath);

	const std::string& getSampleName() const { return m_sampleName; }
	const std::string& getGeomFileName() const { return m_geomFileName; }

	void doTests(class dtNavMesh* navmesh, class dtNavMeshQuery* navquery);

	void handleRender();
	bool handleRenderOverlay(double* proj, double* model, int* view);

private:
	// Explicitly disabled copy constructor and copy assignment operator.
};
