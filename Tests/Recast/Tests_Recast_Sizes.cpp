#include <filesystem>
#include <fstream>
#include <string>

#include <RecastAlloc.h>
#include <catch2/catch_all.hpp>

#include "../../Recast/Include/Recast.h"
#include "../../RecastCLI/Include/BuildContext.h"
#include "../../RecastCLI/Include/Generators.h"
#include "../../RecastCLI/Include/InputGeom.h"

// For comparing to rcVector in benchmarks.
constexpr float g_cellHeight = 0.2f;
constexpr float g_agentHeight = 2.0f;
constexpr float g_agentMaxClimb = 0.9f;
constexpr float g_agentMaxSlope = 45.0f;
constexpr float g_edgeMaxLen = 12.0f;
constexpr float g_regionMinSize = 8.0f;
constexpr float g_regionMergeSize = 20.0f;
constexpr float g_edgeMaxError = 1.3f;
constexpr float g_vertsPerPoly = 6.0f;
constexpr float g_detailSampleDist = 6.0f;
constexpr float g_detailSampleMaxError = 1.0f;
constexpr bool g_filterLedgeSpans = true;
constexpr bool g_filterWalkableLowHeightSpans = true;
constexpr bool g_filterLowHangingObstacles = true;

constexpr int g_loopCount = 100;

inline std::array<float, g_loopCount * RC_MAX_TIMERS> generateThesisTimes(BuildContext &context, const InputGeom &pGeom, rcConfig &config, int *&pEdges, int &edgeCount) {
  std::array<float, g_loopCount * RC_MAX_TIMERS> times{};
  for (int i{}; i < g_loopCount; i++) {
    rcPolyMesh *pMesh{nullptr};
    rcPolyMeshDetail *pDMesh{nullptr};
    if (!generateTheses(context, pGeom, config, g_filterLowHangingObstacles, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, pMesh, pDMesh, pEdges, edgeCount))
      context.dumpLog("Error Thesis:");
    rcFreePolyMesh(pMesh);
    rcFreePolyMeshDetail(pDMesh);
    pMesh = nullptr;
    pDMesh = nullptr;
    if (i != g_loopCount - 1) {
      rcFree(pEdges);
    }
    const int offset{i * RC_MAX_TIMERS};
    for (int j = 0; j < RC_MAX_TIMERS; ++j) {
      times[offset + j] = static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f;
    }
  }
  return times;
}

inline std::array<float, g_loopCount * RC_MAX_TIMERS> generateSingleMeshTimes(BuildContext &context, const InputGeom &pGeom, rcConfig &config) {
  std::array<float, g_loopCount * RC_MAX_TIMERS> times{};
  for (int i{}; i < g_loopCount; i++) {
    rcPolyMesh *pMesh{nullptr};
    rcPolyMeshDetail *pDMesh{nullptr};
    generateSingle(context, pGeom, config, g_filterLowHangingObstacles, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, pMesh, pDMesh);
    rcFreePolyMesh(pMesh);
    rcFreePolyMeshDetail(pDMesh);
    pMesh = nullptr;
    pDMesh = nullptr;

    const int offset{i * RC_MAX_TIMERS};
    for (int j = 0; j < RC_MAX_TIMERS; ++j) {
      times[offset + j] = static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f;
    }
  }
  return times;
}

inline void writeCsvFile(const std::string &filePath, const std::array<float, g_loopCount * RC_MAX_TIMERS> &timerData, const char *header, const int headerSize) {
  std::ofstream csvFile{filePath, std::ios::out};
  csvFile.write(header, headerSize).put('\n');
  for (int i{}; i < g_loopCount; ++i) {
    for (int j{}; j < RC_MAX_TIMERS; ++j) {
      csvFile << timerData[i * RC_MAX_TIMERS + j] << ',';
    }
    csvFile << std::endl;
  }
  csvFile.close();
}

inline void generateTimes(const std::string &output, const std::string &fileName, BuildContext &context, const InputGeom &pGeom, rcConfig config, int *&pEdge, int &edgeCount) {
  const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, config)};
  const std::array thesisTimes{generateThesisTimes(context, pGeom, config, pEdge, edgeCount)};

  constexpr char header[]{
      "Total (ms),"
      "Temp (ms),"
      "Rasterize Triangles (ms),"
      "Build Compact Height Field (ms),"
      "Build Contours (ms),"
      "Build Contours Trace (ms),"
      "Build Contours Simplify (ms),"
      "Filter Border (ms),"
      "Filter Walkable (ms),"
      "Median Area (ms),"
      "Filter Low Obstacles (ms),"
      "Build Polymesh (ms),"
      "Merge Polymeshes (ms),"
      "Erode Area (ms),"
      "Mark Box Area (ms),"
      "Mark Cylinder Area (ms),"
      "Mark Convex Area (ms),"
      "Build Distance Field (ms),"
      "Build Distance Field Distance (ms),"
      "Build Distance Field Blur (ms),"
      "Build Regions (ms),"
      "Build Regions Watershed (ms),"
      "Build Regions Expand (ms),"
      "Build Regions Flood (ms),"
      "Build Regions Filter (ms),"
      "Extract Region Portal (ms)"
      "Build Layers (ms),"
      "Build Polymesh Detail (ms),"
      "Merge Polymesh Details (ms),"};
  std::filesystem::create_directories(output);
  writeCsvFile(output + "/default_" + fileName + ".csv", defaultTimes, header, sizeof header);
  writeCsvFile(output + "/thesis_" + fileName + ".csv", thesisTimes, header, sizeof header);
}

TEST_CASE("Watershed") {
  std::string output{"Data"};
  std::filesystem::create_directories(output);

  const float cellSize{GENERATE(range(0.1f, 0.5f, 0.1f))};
  constexpr float agentRadius{0.0f};
  rcConfig config{
      .cs = cellSize,
      .ch = g_cellHeight,
      .walkableSlopeAngle = g_agentMaxSlope,
      .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
      .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
      .walkableRadius = static_cast<int>(std::ceil(agentRadius / g_cellHeight)),
      .maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize),
      .maxSimplificationError = g_edgeMaxError,
      .minRegionArea = static_cast<int>(rcSqr(g_regionMinSize)),
      .mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize)),
      .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
      .detailSampleDist = cellSize * g_detailSampleDist,
      .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
  };
  SECTION("City") {
    const std::string fileName{"Meshes/City.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
  SECTION("Maze8") {
    const std::string fileName{"Meshes/Maze8.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
  SECTION("Maze16") {
    const std::string fileName{"Meshes/Maze16.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
  SECTION("Maze32") {
    const std::string fileName{"Meshes/Maze32.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
  SECTION("Maze64") {
    const std::string fileName{"Meshes/Maze64.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
  SECTION("Maze128") {
    const std::string fileName{"Meshes/Maze128.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
  SECTION("Military") {
    const std::string fileName{"Meshes/Military.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
  SECTION("Simple") {
    const std::string fileName{"Meshes/Simple.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
  SECTION("University") {
    const std::string fileName{"Meshes/University.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
  SECTION("Zelda") {
    const std::string fileName{"Meshes/Zelda.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
  SECTION("Zelda2x2") {
    const std::string fileName{"Meshes/Zelda2x2.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
  SECTION("Zelda4x4") {
    const std::string fileName{"Meshes/Zelda4x4.obj"};
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, fileName);
    if (!success)
      context.dumpLog("Geom load log %s:", fileName.c_str());
    REQUIRE(success);

    int *pEdges{nullptr};
    int edgeCount{};
    generateTimes(output, fileName.substr(7, fileName.size() - 11) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  }
}