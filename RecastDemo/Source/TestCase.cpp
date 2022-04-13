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
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif
#include "imgui.h"
#include "PerfTimer.h"
#include <list>
#include <fstream>
#include <iostream>
#include <stdlib.h>

#ifdef WIN32
#define snprintf _snprintf
#endif

TestCase::TestCase() : m_tests(0)
{
}

TestCase::~TestCase()
{
	Test *iter = m_tests;
	while (iter)
	{
		Test *next = iter->next;
		delete iter;
		iter = next;
	}
}

static char *parseRow(char *buf, char *bufEnd, char *row, int len)
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
				break;
			done = true;
			break;
		case '\r':
			break;
		case '\t':
		case ' ':
			if (start)
				break;
			// else falls through
		default:
			start = false;
			row[n++] = c;
			if (n >= len - 1)
				done = true;
			break;
		}
	}
	row[n] = '\0';
	return buf;
}

static void copyName(std::string &dst, const char *src)
{
	// Skip white spaces
	while (*src && isspace(*src))
		src++;
	dst = src;
}

bool TestCase::load(const std::string &filePath)
{
	char *buf = 0;
	FILE *fp = fopen(filePath.c_str(), "rb");
	if (!fp)
		return false;
	if (fseek(fp, 0, SEEK_END) != 0)
	{
		fclose(fp);
		return false;
	}
	long bufSize = ftell(fp);
	if (bufSize < 0)
	{
		fclose(fp);
		return false;
	}
	if (fseek(fp, 0, SEEK_SET) != 0)
	{
		fclose(fp);
		return false;
	}
	buf = new char[bufSize];
	if (!buf)
	{
		fclose(fp);
		return false;
	}
	size_t readLen = fread(buf, bufSize, 1, fp);
	fclose(fp);
	if (readLen != 1)
	{
		delete[] buf;
		return false;
	}

	char *src = buf;
	char *srcEnd = buf + bufSize;
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
			Test *test = new Test();
			test->type = TEST_PATHFIND;
			test->expand = false;
			test->next = m_tests;
			m_tests = test;
			sscanf(row + 2, "%f %f %f %f %f %f %hx %hx",
				   &test->spos[0], &test->spos[1], &test->spos[2],
				   &test->epos[0], &test->epos[1], &test->epos[2],
				   &test->includeFlags, &test->excludeFlags);
		}
		else if (row[0] == 'd' && row[1] == 'f')
		{
			// Pathfind test.
			Test *test = new Test();
			test->type = TEST_SHIFTEDPATH;
			test->expand = false;
			test->next = m_tests;
			m_tests = test;
			sscanf(row + 2, "%d %f %f %f %f %f %f %f %hx %hx",
				   &test->FLAG_EXCLUSION,
				   &test->shift_coe,
				   &test->spos[0], &test->spos[1], &test->spos[2],
				   &test->epos[0], &test->epos[1], &test->epos[2],
				   &test->includeFlags, &test->excludeFlags);
		}
		else if (row[0] == 'r' && row[1] == 'c')
		{
			// Pathfind test.
			Test *test = new Test();
			test->type = TEST_RAYCAST;
			test->expand = false;
			test->next = m_tests;
			m_tests = test;
			sscanf(row + 2, "%f %f %f %f %f %f %hx %hx",
				   &test->spos[0], &test->spos[1], &test->spos[2],
				   &test->epos[0], &test->epos[1], &test->epos[2],
				   &test->includeFlags, &test->excludeFlags);
		}
	}

	delete[] buf;

	return true;
}

void TestCase::resetTimes()
{
	for (Test *iter = m_tests; iter; iter = iter->next)
	{
		iter->findNearestPolyTime = 0;
		iter->findPathTime = 0;
		iter->findStraightPathTime = 0;
	}
}

// validate if path end arrives expected end point
bool TestCase::validate_arrive(const float pos_a[3], const float pos_b[3],
							   const float min_distance_arrive)
{
	const float dx = pos_a[0] - pos_b[0];
	const float dy = pos_a[1] - pos_b[1];
	const float dz = pos_a[2] - pos_b[2];
	float dist = sqrtf(dx * dx + dy * dy + dz * dz);
	return (dist < min_distance_arrive);
}

void TestCase::output_result_file(const std::vector<float> &from, const std::vector<float> &to, const uint32_t paths_num_wanted,
								  const float exclude_proportion, uint32_t paths_num,
								  std::vector<float> distance, std::vector<uint32_t> path_nodes_num,
								  std::vector<std::vector<std::vector<float>>> path_nodes)
{
	// write result

	std::string abs_path = "C:\\Users\\Administrator\\Desktop\\Code\\recastnavigation\\RecastDemo\\Bin\\Results\\";
	std::string filename = m_geomFileName + " -result(" + std::to_string(from[0]) + "," + std::to_string(from[1]) + "," + std::to_string(from[2]) + ")" + "to(" + std::to_string(to[0]) + "," + std::to_string(to[1]) + "," + std::to_string(to[2]) + ").txt";
	std::string final_path = abs_path + filename;
	std::ofstream file(final_path, std::ios::out | std::ios::trunc);
	if (!file.is_open())
	{
		m_ctx->log(RC_LOG_PROGRESS, "File NOT Open!");
		return;
	}
	m_ctx->log(RC_LOG_PROGRESS, "File Open SUCCESS!");

	file << "================Navigation Summary================\n"
		 << std::endl;
	file << "GEOM:" << m_geomFileName << std::endl;
	file << "START: (" << from[0] << ", " << from[1] << ", " << from[2] << ")" << std::endl;
	file << "END: (" << to[0] << ", " << to[1] << ", " << to[2] << ")" << std::endl;
	file << "EXCLUDE PROPORTION: " << exclude_proportion << std::endl;
	file << "# PATH QUERIED: " << paths_num_wanted << std::endl;
	file << "# PATH FOUND: " << paths_num << std::endl;
	file << "\n===============Path(s) Found Below================" << std::endl;

	for (uint32_t i = 0; i < paths_num; ++i)
	{
		file << "\nPath #" << i + 1 << " --> Distance: " << distance[i]
			 << " | Node amount: " << path_nodes_num[i] << std::endl;
		file << "Path #" << i + 1 << " = [";
		for (uint32_t pos_i = 0; pos_i < path_nodes_num[i]; ++pos_i)
		{
			file << "(" << -path_nodes[i][pos_i][0] << ", " << path_nodes[i][pos_i][1] << ", "
				 << path_nodes[i][pos_i][2] << ")";
			if (pos_i != path_nodes_num[i] - 1)
				file << ", " << std::endl;
			else
				file << "]" << std::endl;
		}
		file << "\n==================================================" << std::endl;
	}

	file.close();
}

void TestCase::retrieve_result(
	const std::vector<float> &from, const std::vector<float> &to, const uint32_t paths_num_wanted,
	const float exclude_proportion, const float arrive_judge_dis, uint32_t *paths_num,
	std::vector<float> *paths_distance, std::vector<uint32_t> *paths_nodes_num,
	std::vector<std::vector<std::vector<float>>> *paths_nodes, std::string *log)
{

	(*paths_distance).clear();
	(*paths_nodes_num).clear();
	(*paths_nodes).clear();

	(*paths_num) = m_path_count;

	if ((*paths_num) > 0)
	{
		(*paths_distance).reserve((*paths_num));
		(*paths_nodes_num).reserve((*paths_num));
		(*paths_nodes).reserve((*paths_num));

		for (uint32_t path_i = 0; path_i < (*paths_num); ++path_i)
		{
			// // Store distance of paths.
			// m_ctx->log(RC_LOG_PROGRESS, "Path length: %f", m_paths_length[path_i]);
			(*paths_distance).push_back(m_paths_length[path_i]);
			// Store nodes num of paths.
			// m_ctx->log(RC_LOG_PROGRESS, "Node num: %d", m_nodes_num[path_i]);
			(*paths_nodes_num).push_back(m_nodes_num[path_i]);
			// // Store nodes of paths.
			(*paths_nodes).push_back(std::vector<std::vector<float>>());

			std::vector<std::vector<float>> &one_path_nodes = (*paths_nodes).back();
			// m_ctx->log(RC_LOG_PROGRESS, "Path i =: %d", path_i);
			// m_ctx->log(RC_LOG_PROGRESS, "Reserve for: %d", (*paths_nodes_num)[path_i]);
			one_path_nodes.reserve((*paths_nodes_num)[path_i]);

			for (uint32_t node_i = 0; node_i < (*paths_nodes_num)[path_i]; ++node_i)
			{
				one_path_nodes.push_back(std::vector<float>());
				std::vector<float> &one_node = one_path_nodes.back();
				one_node.reserve(3);
				for (uint32_t ax_i = 0; ax_i < 3; ++ax_i)
				{
					one_node.push_back(m_path_nodes[path_i * MAX_POLY_NUM + node_i * 3 + ax_i]);
				}
			}
		}
	}

	// // wirte out result
	output_result_file(from, to, MAX_PATH_NUM, EXCLUDE_PROPORTION, *paths_num, *paths_distance, *paths_nodes_num, *paths_nodes);
}

void TestCase::doTests(dtNavMesh *navmesh, dtNavMeshQuery *navquery)
{
	if (m_tests->type == TEST_PATHFIND || m_tests->type == TEST_RAYCAST)
	{

		if (!navmesh || !navquery)
			return;

		// resetTimes();
		static const int MAX_POLYS = 256;
		dtPolyRef polys[MAX_POLYS];
		float straight[MAX_POLYS * 3];
		const float polyPickExt[3] = {2, 4, 2};

		std::list<uint32_t> blocked_polys;
		std::list<uint32_t>::iterator it;
		m_path_count = 0;
		for (Test *iter = m_tests; iter && m_path_count < MAX_PATH_NUM; iter = iter->next)
		{
			// Reverse x axis
			iter->spos[0] = iter->spos[0] * -1;
			iter->epos[0] = iter->epos[0] * -1;

			delete[] iter->polys;
			iter->polys = 0;
			iter->npolys = 0;
			delete[] iter->straight;
			iter->straight = 0;
			iter->nstraight = 0;

			dtQueryFilter filter;
			filter.setIncludeFlags(iter->includeFlags);
			filter.setExcludeFlags(iter->excludeFlags);

			// Find start points
			TimeVal findNearestPolyStart = getPerfTime();

			dtPolyRef startRef, endRef;

			navquery->findNearestPoly(iter->spos, polyPickExt, &filter, &startRef, iter->nspos);
			navquery->findNearestPoly(iter->epos, polyPickExt, &filter, &endRef, iter->nepos);

			TimeVal findNearestPolyEnd = getPerfTime();
			iter->findNearestPolyTime += getPerfTimeUsec(findNearestPolyEnd - findNearestPolyStart);

			if (!startRef || !endRef)
				continue;

			// Find path
			TimeVal findPathStart = getPerfTime();

			navquery->findPath(startRef, endRef, iter->spos, iter->epos, &filter, polys, &iter->npolys, MAX_POLYS);

			TimeVal findPathEnd = getPerfTime();
			iter->findPathTime += getPerfTimeUsec(findPathEnd - findPathStart);

			// Find straight path
			if (iter->npolys)
			{

				TimeVal findStraightPathStart = getPerfTime();
				// navquery->findStraightPath(iter->spos, iter->epos, polys, iter->npolys,
				// 						   straight, 0, 0, &iter->nstraight, MAX_POLYS);

				navquery->findStraightPath(iter->spos, iter->epos, polys, iter->npolys,
										   straight, 0, 0, &iter->nstraight, MAX_POLYS);

				TimeVal findStraightPathEnd = getPerfTime();
				float final_pos[] = {straight[(iter->nstraight - 1) * 3], straight[(iter->nstraight - 1) * 3 + 1], straight[(iter->nstraight - 1) * 3 + 2]};

				// m_ctx->log(RC_LOG_PROGRESS, "last point x: %f", final_pos[0]);
				// m_ctx->log(RC_LOG_PROGRESS, "last point y: %f", final_pos[1]);
				// m_ctx->log(RC_LOG_PROGRESS, "last point z: %f", final_pos[2]);

				// remove paths not arrived
				if (!validate_arrive(iter->epos, final_pos, 1))
				{
					delete[] iter->polys;
					iter->polys = 0;
					iter->npolys = 0;
					delete[] iter->straight;
					iter->straight = 0;
					iter->nstraight = 0;
					break;
				}

				iter->findStraightPathTime += getPerfTimeUsec(findStraightPathEnd - findStraightPathStart);

				m_path_dis_ = 0.0f;
				int path_len = iter->nstraight;
				m_nodes_num[m_path_count] = path_len;
				memcpy(&m_path_nodes[m_path_count * MAX_POLY_NUM], straight, sizeof(float) * 3 * path_len);

				// m_ctx->log(RC_LOG_PROGRESS, "path#: %d", m_path_count);
				// m_ctx->log(RC_LOG_PROGRESS, "pathlenth: %d", path_len);
				// m_ctx->log(RC_LOG_PROGRESS, "Node count: %d", m_nodes_num[m_path_count]);

				for (int i = 0; i < path_len; ++i)
				{

					// Reverse x axis in point-based path.

					// straight[m_path_count * 3] = -straight[m_path_count * 3];

					// Calc paths_distance between two points.
					if (i == 0)
					{
						// Cal the distance between the start point and the first point in path
						const float dx = straight[i * 3] - iter->spos[0];
						const float dy = straight[i * 3 + 1] - iter->spos[1];
						const float dz = straight[i * 3 + 2] - iter->spos[2];
						// m_ctx->log(RC_LOG_PROGRESS, "dx: %f dy: %f dz: %f", dx, dy, dz);
						float move = dtMathSqrtf(dx * dx + dy * dy + dz * dz);
						m_path_dis_ += move;
						// m_ctx->log(RC_LOG_PROGRESS, "move: %f", move);
						// m_ctx->log(RC_LOG_PROGRESS, "m_path_dis_0: %f", m_path_dis_);
					}
					else
					{
						const float dx = straight[(i - 1) * 3] - straight[i * 3];
						const float dy = straight[(i - 1) * 3 + 1] - straight[i * 3 + 1];
						const float dz = straight[(i - 1) * 3 + 2] - straight[i * 3 + 2];
						// m_ctx->log(RC_LOG_PROGRESS, "dx: %f dy: %f dz: %f", dx, dy, dz);
						float move = dtMathSqrtf(dx * dx + dy * dy + dz * dz);
						m_path_dis_ += move;
						// m_ctx->log(RC_LOG_PROGRESS, "move: %f", move);
						// m_ctx->log(RC_LOG_PROGRESS, "m_path_dis_: %f", m_path_dis_);
					}
				}
			}

			m_paths_length[m_path_count] = m_path_dis_;

			// m_ctx->log(RC_LOG_PROGRESS, "Path length: %f", m_paths_length[m_path_count]);

			m_path_count++;

			if (iter->npolys)
			{
				iter->polys = new dtPolyRef[iter->npolys];
				memcpy(iter->polys, polys, sizeof(dtPolyRef) * iter->npolys);
			}

			if (iter->nstraight)
			{
				iter->straight = new float[iter->nstraight * 3];
				memcpy(iter->straight, straight, sizeof(float) * 3 * iter->nstraight);
			}

			// Set exclusion flags
			const float exclude_boarder = EXCLUDE_PROPORTION / 2;

			for (uint32_t poly_i = static_cast<uint32_t>(iter->npolys * exclude_boarder); poly_i < static_cast<uint32_t>(iter->npolys * (1 - exclude_boarder)); ++poly_i)
			{
				navmesh->setPolyFlags(polys[poly_i], iter->excludeFlags);
				blocked_polys.push_back(poly_i);
			}
			// Clear exclustion tags
			if (!iter->next)
			{
				for (it = blocked_polys.begin(); it != blocked_polys.end(); it++)
				{
					navmesh->setPolyFlags(polys[*it], iter->excludeFlags);
				}
			}
		}

		// ====================================
		std::vector<float> from;
		std::vector<float> to;
		from.push_back(m_tests->spos[0] * (-1));
		to.push_back(m_tests->epos[0] * (-1));
		for (int i = 1; i < 3; i++)
		{
			from.push_back(m_tests->spos[i]);
			to.push_back(m_tests->epos[i]);
		}

		uint32_t paths_num;
		std::vector<float> distance;
		std::vector<uint32_t> path_nodes_num;
		std::vector<std::vector<std::vector<float>>> path_nodes;
		std::string log;
		retrieve_result(from, to, MAX_PATH_NUM, EXCLUDE_PROPORTION, 1, &paths_num, &distance, &path_nodes_num, &path_nodes, &log);
	}
	else if (m_tests->type == TEST_SHIFTEDPATH)
	{
		if (!navmesh || !navquery)
			return;

		// resetTimes();
		static const int MAX_POLYS = 256;
		dtPolyRef polys[MAX_POLYS];
		float straight[MAX_POLYS * 3];
		float _straight[MAX_POLYS * 3];
		const float polyPickExt[3] = {2, 4, 2};

		std::list<uint32_t> blocked_polys;
		std::list<uint32_t>::iterator it;
		m_path_count = 0;

		std::string abs_path = "C:\\Users\\Administrator\\Desktop\\Code\\recastnavigation\\RecastDemo\\Bin\\Results\\";
		std::string filename = "intermediate.txt";
		std::string final_path = abs_path + filename;
		std::ofstream intermediate(final_path, std::ios::out | std::ios::trunc);
		if (!intermediate.is_open())
		{
			m_ctx->log(RC_LOG_PROGRESS, "intermediate NOT Open!");
			return;
		}
		m_ctx->log(RC_LOG_PROGRESS, "intermediate Open SUCCESS!");

		for (Test *iter = m_tests; iter && m_path_count < MAX_PATH_NUM; iter = iter->next)
		{
			m_ctx->log(RC_LOG_PROGRESS, "FLAG_EXCLUSION %d", iter->FLAG_EXCLUSION);
			m_ctx->log(RC_LOG_PROGRESS, "SHIFT_COE %f", iter->shift_coe);
			// Reverse x axis
			iter->spos[0] = iter->spos[0] * -1;
			iter->epos[0] = iter->epos[0] * -1;

			delete[] iter->polys;
			iter->polys = 0;
			iter->npolys = 0;
			delete[] iter->straight;
			iter->straight = 0;
			delete[] iter->straight;
			iter->straight = 0;
			iter->nstraight = 0;

			dtQueryFilter filter;
			filter.setIncludeFlags(iter->includeFlags);
			filter.setExcludeFlags(iter->excludeFlags);

			// Find start points
			TimeVal findNearestPolyStart = getPerfTime();

			dtPolyRef startRef, endRef;

			navquery->findNearestPoly(iter->spos, polyPickExt, &filter, &startRef, iter->nspos);
			navquery->findNearestPoly(iter->epos, polyPickExt, &filter, &endRef, iter->nepos);

			TimeVal findNearestPolyEnd = getPerfTime();
			iter->findNearestPolyTime += getPerfTimeUsec(findNearestPolyEnd - findNearestPolyStart);

			if (!startRef || !endRef)
				continue;

			// Find path
			TimeVal findPathStart = getPerfTime();

			navquery->findPath(startRef, endRef, iter->spos, iter->epos, &filter, polys, &iter->npolys, MAX_POLYS);

			TimeVal findPathEnd = getPerfTime();
			iter->findPathTime += getPerfTimeUsec(findPathEnd - findPathStart);

			// Find straight path
			if (iter->npolys)
			{

				TimeVal findStraightPathStart = getPerfTime();
				// navquery->findStraightPath(iter->spos, iter->epos, polys, iter->npolys,
				// 						   straight, 0, 0, &iter->nstraight, MAX_POLYS);

				navquery->findStraightPath_shifted(iter->spos, iter->epos, polys, iter->npolys,
												   straight, _straight, iter->shift_coe, 0, 0, &iter->nstraight, MAX_POLYS);
				TimeVal findStraightPathEnd = getPerfTime();

				intermediate << "=============================================" << std::endl;
				intermediate << "straight:" << std::endl;

				for (int i = 0; i < iter->nstraight * 3; i++)
				{
					if (i % 3 == 0)
					{
						intermediate << "---------------------------------------" << std::endl;
					}
					intermediate << straight[i] << std::endl;
				}

				intermediate << "=============================================" << std::endl;
				intermediate << "_straight:" << std::endl;

				for (int i = 0; i < iter->nstraight * 3; i++)
				{
					if (i % 3 == 0)
					{
						intermediate << "---------------------------------------" << std::endl;
					}
					intermediate << _straight[i] << std::endl;
				}

				float final_pos[] = {straight[(iter->nstraight - 1) * 3], straight[(iter->nstraight - 1) * 3 + 1], straight[(iter->nstraight - 1) * 3 + 2]};
				float _final_pos[] = {_straight[(iter->nstraight - 1) * 3], _straight[(iter->nstraight - 1) * 3 + 1], _straight[(iter->nstraight - 1) * 3 + 2]};

				m_ctx->log(RC_LOG_PROGRESS, "last point x: %f", final_pos[0]);
				m_ctx->log(RC_LOG_PROGRESS, "last point y: %f", final_pos[1]);
				m_ctx->log(RC_LOG_PROGRESS, "last point z: %f", final_pos[2]);

				m_ctx->log(RC_LOG_PROGRESS, "last point _x: %f", _final_pos[0]);
				m_ctx->log(RC_LOG_PROGRESS, "last point _y: %f", _final_pos[1]);
				m_ctx->log(RC_LOG_PROGRESS, "last point _z: %f", _final_pos[2]);

				// remove paths not arrived
				if (!validate_arrive(iter->epos, final_pos, 5))
				{
					delete[] iter->polys;
					iter->polys = 0;
					iter->npolys = 0;
					delete[] iter->straight;
					iter->straight = 0;
					iter->nstraight = 0;
					break;
				}

				iter->findStraightPathTime += getPerfTimeUsec(findStraightPathEnd - findStraightPathStart);

				m_path_dis_ = 0.0f;
				int path_len = iter->nstraight;
				m_nodes_num[m_path_count] = path_len;
				memcpy(&m_path_nodes[m_path_count * MAX_POLY_NUM], straight, sizeof(float) * 3 * path_len);

				// m_ctx->log(RC_LOG_PROGRESS, "path#: %d", m_path_count);
				// m_ctx->log(RC_LOG_PROGRESS, "pathlenth: %d", path_len);
				// m_ctx->log(RC_LOG_PROGRESS, "Node count: %d", m_nodes_num[m_path_count]);

				for (int i = 0; i < path_len; ++i)
				{

					// Reverse x axis in point-based path.

					// straight[m_path_count * 3] = -straight[m_path_count * 3];

					// Calc paths_distance between two points.
					if (i == 0)
					{
						// Cal the distance between the start point and the first point in path
						const float dx = straight[i * 3] - iter->spos[0];
						const float dy = straight[i * 3 + 1] - iter->spos[1];
						const float dz = straight[i * 3 + 2] - iter->spos[2];
						// m_ctx->log(RC_LOG_PROGRESS, "dx: %f dy: %f dz: %f", dx, dy, dz);
						float move = dtMathSqrtf(dx * dx + dy * dy + dz * dz);
						m_path_dis_ += move;
						// m_ctx->log(RC_LOG_PROGRESS, "move: %f", move);
						// m_ctx->log(RC_LOG_PROGRESS, "m_path_dis_0: %f", m_path_dis_);
					}
					else
					{
						const float dx = straight[(i - 1) * 3] - straight[i * 3];
						const float dy = straight[(i - 1) * 3 + 1] - straight[i * 3 + 1];
						const float dz = straight[(i - 1) * 3 + 2] - straight[i * 3 + 2];
						// m_ctx->log(RC_LOG_PROGRESS, "dx: %f dy: %f dz: %f", dx, dy, dz);
						float move = dtMathSqrtf(dx * dx + dy * dy + dz * dz);
						m_path_dis_ += move;
						// m_ctx->log(RC_LOG_PROGRESS, "move: %f", move);
						// m_ctx->log(RC_LOG_PROGRESS, "m_path_dis_: %f", m_path_dis_);
					}
				}
			}

			m_paths_length[m_path_count] = m_path_dis_;

			// m_ctx->log(RC_LOG_PROGRESS, "Path length: %f", m_paths_length[m_path_count]);

			m_path_count++;

			if (iter->npolys)
			{
				iter->polys = new dtPolyRef[iter->npolys];
				memcpy(iter->polys, polys, sizeof(dtPolyRef) * iter->npolys);
			}

			if (iter->nstraight)
			{
				iter->straight = new float[iter->nstraight * 3];
				memcpy(iter->straight, straight, sizeof(float) * 3 * iter->nstraight);
			}
			// Set exclusion flags
			if (iter->FLAG_EXCLUSION == 1)
			{
				const float exclude_boarder = EXCLUDE_PROPORTION / 2;

				for (uint32_t poly_i = static_cast<uint32_t>(iter->npolys * exclude_boarder); poly_i < static_cast<uint32_t>(iter->npolys * (1 - exclude_boarder)); ++poly_i)
				{
					navmesh->setPolyFlags(polys[poly_i], iter->excludeFlags);
					blocked_polys.push_back(poly_i);
				}
				// Clear exclustion tags
				if (!iter->next)
				{
					for (it = blocked_polys.begin(); it != blocked_polys.end(); it++)
					{
						navmesh->setPolyFlags(polys[*it], iter->excludeFlags);
					}
				}
			}
		}
		intermediate.close();

		// ====================================
		std::vector<float> from;
		std::vector<float> to;
		from.push_back(m_tests->spos[0] * (-1));
		to.push_back(m_tests->epos[0] * (-1));
		for (int i = 1; i < 3; i++)
		{
			from.push_back(m_tests->spos[i]);
			to.push_back(m_tests->epos[i]);
		}

		uint32_t paths_num;
		std::vector<float> distance;
		std::vector<uint32_t> path_nodes_num;
		std::vector<std::vector<std::vector<float>>> path_nodes;
		std::string log;
		retrieve_result(from, to, MAX_PATH_NUM, EXCLUDE_PROPORTION, 1, &paths_num, &distance, &path_nodes_num, &path_nodes, &log);
	}
}

void TestCase::handleRender()
{
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	for (Test *iter = m_tests; iter; iter = iter->next)
	{
		float dir[3];
		dtVsub(dir, iter->epos, iter->spos);
		dtVnormalize(dir);
		glColor4ub(128, 25, 0, 192);
		glVertex3f(iter->spos[0], iter->spos[1] - 0.3f, iter->spos[2]);
		glVertex3f(iter->spos[0], iter->spos[1] + 0.3f, iter->spos[2]);
		glVertex3f(iter->spos[0], iter->spos[1] + 0.3f, iter->spos[2]);
		glVertex3f(iter->spos[0] + dir[0] * 0.3f, iter->spos[1] + 0.3f + dir[1] * 0.3f, iter->spos[2] + dir[2] * 0.3f);
		glColor4ub(51, 102, 0, 129);
		glVertex3f(iter->epos[0], iter->epos[1] - 0.3f, iter->epos[2]);
		glVertex3f(iter->epos[0], iter->epos[1] + 0.3f, iter->epos[2]);

		if (iter->expand)
		{
			const float s = 0.1f;
			glColor4ub(255, 32, 0, 128);
			glVertex3f(iter->spos[0] - s, iter->spos[1], iter->spos[2]);
			glVertex3f(iter->spos[0] + s, iter->spos[1], iter->spos[2]);
			glVertex3f(iter->spos[0], iter->spos[1], iter->spos[2] - s);
			glVertex3f(iter->spos[0], iter->spos[1], iter->spos[2] + s);
			glColor4ub(255, 192, 0, 255);
			glVertex3f(iter->nspos[0] - s, iter->nspos[1], iter->nspos[2]);
			glVertex3f(iter->nspos[0] + s, iter->nspos[1], iter->nspos[2]);
			glVertex3f(iter->nspos[0], iter->nspos[1], iter->nspos[2] - s);
			glVertex3f(iter->nspos[0], iter->nspos[1], iter->nspos[2] + s);

			glColor4ub(255, 32, 0, 128);
			glVertex3f(iter->epos[0] - s, iter->epos[1], iter->epos[2]);
			glVertex3f(iter->epos[0] + s, iter->epos[1], iter->epos[2]);
			glVertex3f(iter->epos[0], iter->epos[1], iter->epos[2] - s);
			glVertex3f(iter->epos[0], iter->epos[1], iter->epos[2] + s);
			glColor4ub(255, 192, 0, 255);
			glVertex3f(iter->nepos[0] - s, iter->nepos[1], iter->nepos[2]);
			glVertex3f(iter->nepos[0] + s, iter->nepos[1], iter->nepos[2]);
			glVertex3f(iter->nepos[0], iter->nepos[1], iter->nepos[2] - s);
			glVertex3f(iter->nepos[0], iter->nepos[1], iter->nepos[2] + s);
		}

		if (iter->expand)
			glColor4ub(255, 192, 0, 255);
		else
			glColor4ub(0, 0, 0, 64);

		for (int i = 0; i < iter->nstraight - 1; ++i)
		{
			glVertex3f(iter->straight[i * 3 + 0], iter->straight[i * 3 + 1] + 0.3f, iter->straight[i * 3 + 2]);
			glVertex3f(iter->straight[(i + 1) * 3 + 0], iter->straight[(i + 1) * 3 + 1] + 0.3f, iter->straight[(i + 1) * 3 + 2]);
		}
	}
	glEnd();
	glLineWidth(1.0f);
}

bool TestCase::handleRenderOverlay(double *proj, double *model, int *view)
{

	GLdouble x, y, z;
	char text[64], subtext[64];
	int n = 0;

	static const float LABEL_DIST = 1.0f;

	for (Test *iter = m_tests; iter && n < m_path_count; iter = iter->next)
	{
		float pt[3], dir[3];
		if (iter->nstraight)
		{
			dtVcopy(pt, &iter->straight[3]);
			if (dtVdist(pt, iter->spos) > LABEL_DIST)
			{
				dtVsub(dir, pt, iter->spos);
				dtVnormalize(dir);
				dtVmad(pt, iter->spos, dir, LABEL_DIST);
			}
			pt[1] += 0.5f;
		}
		else
		{
			dtVsub(dir, iter->epos, iter->spos);
			dtVnormalize(dir);
			dtVmad(pt, iter->spos, dir, LABEL_DIST);
			pt[1] += 0.5f;
		}

		if (gluProject((GLdouble)pt[0], (GLdouble)pt[1], (GLdouble)pt[2],
					   model, proj, view, &x, &y, &z))
		{
			snprintf(text, 64, "Path %d\n", n);
			unsigned int col = imguiRGBA(0, 0, 0, 128);
			if (iter->expand)
				col = imguiRGBA(255, 192, 0, 220);
			imguiDrawText((int)x, (int)(y - 25), IMGUI_ALIGN_CENTER, text, col);
		}
		n++;
	}

	static int resScroll = 0;
	bool mouseOverMenu = imguiBeginScrollArea("Test Results", 10, view[3] - 10 - 350, 200, 350, &resScroll);
	//		mouseOverMenu = true;

	n = 0;
	for (Test *iter = m_tests; iter && n < m_path_count; iter = iter->next)
	{
		const int total = iter->findNearestPolyTime + iter->findPathTime + iter->findStraightPathTime;
		snprintf(subtext, 64, "%.4f ms", (float)total / 1000.0f);
		snprintf(text, 64, "Path %d", n);

		if (imguiCollapse(text, subtext, iter->expand))
			iter->expand = !iter->expand;
		if (iter->expand)
		{
			snprintf(text, 64, "Poly: %.4f ms", (float)iter->findNearestPolyTime / 1000.0f);
			imguiValue(text);

			snprintf(text, 64, "Path: %.4f ms", (float)iter->findPathTime / 1000.0f);
			imguiValue(text);

			snprintf(text, 64, "Straight: %.4f ms", (float)iter->findStraightPathTime / 1000.0f);
			imguiValue(text);

			imguiSeparator();
		}

		n++;
	}

	imguiEndScrollArea();

	return mouseOverMenu;
}
