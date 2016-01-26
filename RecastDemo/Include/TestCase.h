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

#ifndef TESTCASE_H
#define TESTCASE_H

#include <string>
#include "DetourNavMesh.h"

class TestCase
{
	enum TestType
	{
		TEST_PATHFIND,
		TEST_RAYCAST,
	};
	
	struct Test
	{
		Test() : straight(0), nstraight(0), polys(0), npolys(0) {};
		~Test()
		{
			delete [] straight;
			delete [] polys;
		}
		
		TestType type;
		float spos[3];
		float epos[3];
		float nspos[3];
		float nepos[3];
		float radius;
		unsigned short includeFlags;
		unsigned short excludeFlags;
		bool expand;
		
		float* straight;
		int nstraight;
		dtPolyRef* polys;
		int npolys;
		
		int findNearestPolyTime;
		int findPathTime;
		int findStraightPathTime;
		
		Test* next;
	private:
		// Explicitly disabled copy constructor and copy assignment operator.
		Test(const Test&);
		Test& operator=(const Test&);
	};

	std::string m_sampleName;
	std::string m_geomFileName;
	Test* m_tests;
	
	void resetTimes();
	
public:
	TestCase();
	~TestCase();

	bool load(const std::string& filePath);
	
	const std::string& getSampleName() const { return m_sampleName; }
	const std::string& getGeomFileName() const { return m_geomFileName; }
	
	void doTests(class dtNavMesh* navmesh, class dtNavMeshQuery* navquery);
	
	void handleRender();
	bool handleRenderOverlay(double* proj, double* model, int* view);

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	TestCase(const TestCase&);
	TestCase& operator=(const TestCase&);
};

#endif // TESTCASE_H