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

#include <cstdint>
#include <string>
#include <vector>

class TestCase
{
	enum class TestType : uint8_t
	{
		PATHFIND,
		RAYCAST
	};

	struct Test
	{
		TestCase::TestType type{};
		float spos[3]{};
		float epos[3]{};
		float nspos[3]{};
		float nepos[3]{};
		float radius = 0;
		unsigned short includeFlags = 0;
		unsigned short excludeFlags = 0;
		bool expand = false;

		std::vector<float> straight {};
		std::vector<dtPolyRef*> polys {};

		int findNearestPolyTime = 0;
		int findPathTime = 0;
		int findStraightPathTime = 0;
	};

	std::vector<Test> tests;

	void resetTimes();

public:
	bool load(const std::string& filePath);

	std::string sampleName;
	std::string geomFileName;

	void doTests(class dtNavMesh* navmesh, class dtNavMeshQuery* navquery);

	void render();
	bool renderOverlay();
};
