#include "BuildContext.h"
#include "InputGeom.h"
#include <Recast.h>
#include <RecastAlloc.h>

#include <fstream>
#include <ios>
#include <ostream>
#include <string>

#include <catch2/catch_all.hpp>

#include "Tests_Recast_LCM.h"

TEST_CASE("Watershed - as_oilrig") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/as_oilrig.obj"};
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
TEST_CASE("Watershed - BigCity") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/BigCity.obj"};
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
  rcFree(pEdges);
}
TEST_CASE("Watershed - cs_assault") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/cs_assault.obj"};
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
TEST_CASE("Watershed - cs_siege") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/cs_siege.obj"};
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
TEST_CASE("Watershed - de_dust") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/de_dust.obj"};
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
TEST_CASE("Watershed - Dungeon") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Dungeon.obj"};
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
TEST_CASE("Watershed - Jardin") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Jardin.obj"};
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
TEST_CASE("Watershed - Library") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Library.obj"};
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
TEST_CASE("Watershed - Nav Test") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/NavTest.obj"};
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
TEST_CASE("Watershed - Neogen 1") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Neogen1.obj"};
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
TEST_CASE("Watershed - Neogen 2") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Neogen2.obj"};
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
TEST_CASE("Watershed - Neogen 3") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Neogen3.obj"};
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
TEST_CASE("Watershed - Parking Lot") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/ParkingLot.obj"};
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
TEST_CASE("Watershed - Tower") {
  std::string const output{"Data"};
  std::ofstream csvFile{output + "/Timings.csv", std::ios::out | std::ios::app};
  if (csvFile.tellp() == 0) {
    csvFile.write(header, sizeof(header)).put('\n');
  }
  csvFile.close();
  const std::string fileName{"Meshes/Tower.obj"};
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
