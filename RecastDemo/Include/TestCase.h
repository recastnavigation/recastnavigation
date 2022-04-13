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
#include <vector>
#include "DetourNavMesh.h"
#include "SampleInterfaces.h"
#include <SampleInterfaces.h>

class TestCase
{
	enum TestType
	{
		TEST_PATHFIND,
		TEST_RAYCAST,
		TEST_SHIFTEDPATH,
	};

	struct Test
	{
		Test() : type(),
				 spos(),
				 epos(),
				 nspos(),
				 nepos(),
				 radius(0),
				 includeFlags(0),
				 excludeFlags(0),
				 expand(false),
				 straight(0),
				 nstraight(0),
				 polys(0),
				 npolys(0),
				 findNearestPolyTime(0),
				 findPathTime(0),
				 findStraightPathTime(0),
				 next(0)
		{
		}

		~Test()
		{
			delete[] straight;
			delete[] polys;
		}

		TestType type;
		int FLAG_EXCLUSION;
		float shift_coe = 0.0;
		float spos[3];
		float epos[3];
		float nspos[3];
		float nepos[3];
		float radius;
		unsigned short includeFlags;
		unsigned short excludeFlags;
		bool expand;

		float *straight;
		int nstraight;
		dtPolyRef *polys;
		int npolys;

		int findNearestPolyTime;
		int findPathTime;
		int findStraightPathTime;

		Test *next;

	private:
		// Explicitly disabled copy constructor and copy assignment operator.
		Test(const Test &);
		Test &operator=(const Test &);
	};
	static const int MAX_POLY_NUM = 256 * 4; // int: recastnavigation api required // MODIFIED
	static const uint32_t MAX_PATH_LEN = 2048 * 4;
	static const uint32_t MAX_PATH_NUM = 10;
	const float EXCLUDE_PROPORTION = 0.98;

	BuildContext *m_ctx;
	int m_path_count;
	int m_nodes_num[MAX_PATH_NUM];
	float m_paths_length[MAX_PATH_NUM];
	float m_path_nodes[MAX_PATH_NUM * MAX_POLY_NUM * 3];
	int m_polys_num_;
	int m_path_len_;
	float m_path_dis_;

	std::string m_sampleName;
	std::string m_geomFileName;
	Test *m_tests;

	void resetTimes();

public:
	TestCase();
	~TestCase();

	bool load(const std::string &filePath);

	const std::string &getSampleName() const { return m_sampleName; }
	const std::string &getGeomFileName() const { return m_geomFileName; }

	void doTests(class dtNavMesh *navmesh, class dtNavMeshQuery *navquery);
	// MODIFIED: set context
	bool validate_arrive(const float pos_a[3], const float pos_b[3],
						 const float min_distance_arrive);
	void setContext(BuildContext *ctx) { m_ctx = ctx; }
	void output_result_file(const std::vector<float> &from, const std::vector<float> &to, const uint32_t paths_num_wanted,
							const float exclude_proportion, uint32_t paths_num,
							std::vector<float> distance, std::vector<uint32_t> path_nodes_num,
							std::vector<std::vector<std::vector<float>>> path_nodes);
	void retrieve_result(
		const std::vector<float> &from, const std::vector<float> &to, const uint32_t paths_num_wanted,
		const float exclude_proportion, const float arrive_judge_dis, uint32_t *paths_num,
		std::vector<float> *paths_distance, std::vector<uint32_t> *paths_nodes_num,
		std::vector<std::vector<std::vector<float>>> *paths_nodes, std::string *log);

	void handleRender();
	bool handleRenderOverlay(double *proj, double *model, int *view);

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	TestCase(const TestCase &);
	TestCase &operator=(const TestCase &);
};

#endif // TESTCASE_H