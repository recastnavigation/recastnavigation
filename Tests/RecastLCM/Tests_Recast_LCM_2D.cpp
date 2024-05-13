#include "BuildContext.h"
#include "Generators.h"
#include "InputGeom.h"
#include <Recast.h>
#include <RecastAlloc.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <ios>
#include <ostream>
#include <string>
#include <vector>

#include <catch2/catch_all.hpp>

#include "Tests_Recast_LCM.h"

TEST_CASE("Watershed - City") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/City.obj"};
  const float cellSize{GENERATE(0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  processBourderEdges("CSV/minima-City.csv", output, name + "_" + std::to_string(static_cast<int>(cellSize * 10)), pGeom, config, pEdges, edgeCount);
}
TEST_CASE("Watershed - Maze 8") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Maze8.obj"};
  const float cellSize{GENERATE(0.1f, 0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  rcFree(pEdges);
}
TEST_CASE("Watershed - Maze 16") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Maze16.obj"};
  const float cellSize{GENERATE(0.1f, 0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  rcFree(pEdges);
}
TEST_CASE("Watershed - Maze 32") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Maze32.obj"};
  const float cellSize{GENERATE(0.1f, 0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  rcFree(pEdges);
}
TEST_CASE("Watershed - Maze 64") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Maze64.obj"};
  const float cellSize{GENERATE(0.1f, 0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  rcFree(pEdges);
}
TEST_CASE("Watershed - Maze 128") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Maze128.obj"};
  const float cellSize{GENERATE(0.1f, 0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  rcFree(pEdges);
}
TEST_CASE("Watershed - Military") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Military.obj"};
  const float cellSize{GENERATE(0.1f, 0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  processBourderEdges("CSV/minima-Military.csv", output, name + "_" + std::to_string(static_cast<int>(cellSize * 10)), pGeom, config, pEdges, edgeCount);
}
TEST_CASE("Watershed - Simple") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Simple.obj"};
  const float cellSize{GENERATE(0.1f, 0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  rcFree(pEdges);
}
TEST_CASE("Watershed - University") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/University.obj"};
  const float cellSize{GENERATE(0.1f, 0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  rcFree(pEdges);
}
TEST_CASE("Watershed - Zelda") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Zelda.obj"};
  const float cellSize{GENERATE(0.1f, 0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  processBourderEdges("CSV/minima-Zelda.csv", output, name + "_" + std::to_string(static_cast<int>(cellSize * 10)), pGeom, config, pEdges, edgeCount);
}
TEST_CASE("Watershed - Zelda 2x2") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Zelda2x2.obj"};
  const float cellSize{GENERATE(0.1f, 0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  processBourderEdges("CSV/minima-Zelda2x2.csv", output, name + "_" + std::to_string(static_cast<int>(cellSize * 10)), pGeom, config, pEdges, edgeCount);
}
TEST_CASE("Watershed - Zelda 4x4") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Zelda4x4.obj"};
  const float cellSize{GENERATE(0.1f, 0.2f, 0.3f, 0.4f, 0.5f)};
  const std::string name{fileName.substr(7, fileName.size() - 11)};

  rcConfig config{};
  config.cs = cellSize;
  config.ch = g_cellHeight;
  config.walkableSlopeAngle = g_agentMaxSlope;
  config.walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight));
  config.walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight));
  config.walkableRadius = static_cast<int>(std::ceil(g_agentRadius / g_cellHeight));
  config.maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize);
  config.maxSimplificationError = g_edgeMaxError;
  config.minRegionArea = static_cast<int>(rcSqr(g_regionMinSize));
  config.mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize));
  config.maxVertsPerPoly = static_cast<int>(g_vertsPerPoly);
  config.detailSampleDist = cellSize * g_detailSampleDist;

  BuildContext context{};
  InputGeom pGeom{};
  bool const success = pGeom.load(&context, fileName);
  if (!success)
    context.dumpLog("Geom load log %s:", fileName.c_str());
  REQUIRE(success);

  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, name, cellSize, context, pGeom, config, pEdges, edgeCount);
  processBourderEdges("CSV/minima-Zelda4x4.csv", output, name + "_" + std::to_string(static_cast<int>(cellSize * 10)), pGeom, config, pEdges, edgeCount);
}