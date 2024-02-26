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
constexpr float agentRadius = 0.0f;
constexpr float g_agentHeight = 2.0f;
constexpr float g_agentMaxClimb = 0.9f;
constexpr float g_agentMaxSlope = 45.0f;
constexpr float g_edgeMaxLen = 12.0f;
constexpr float g_edgeMaxError = 1.3f;
constexpr float g_vertsPerPoly = 6.0f;
constexpr float g_detailSampleDist = 6.0f;
constexpr float g_detailSampleMaxError = 1.0f;
constexpr float mergeS = 20.0f;
constexpr float minS = 8.0f;
constexpr bool g_filterLedgeSpans = true;
constexpr bool g_filterWalkableLowHeightSpans = true;
constexpr bool g_filterLowHangingObstacles = true;

constexpr int g_loopCount = 10;

inline bool runThesis(BuildContext &context, const InputGeom &pGeom, const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans, const bool filterLowHangingObstacles, rcConfig &config, int *&pEdges, int &edgesSize) {
  rcPolyMesh *pMesh{nullptr};
  rcPolyMeshDetail *pDMesh{nullptr};
  float totalBuildTimeMs{};
  const bool success{generateTheses(context, pGeom, config, filterLowHangingObstacles, filterLedgeSpans, filterWalkableLowHeightSpans, totalBuildTimeMs, pMesh, pDMesh, pEdges, edgesSize)};
  rcFreePolyMesh(pMesh);
  rcFreePolyMeshDetail(pDMesh);
  pMesh = nullptr;
  pDMesh = nullptr;

  return success;
}

inline std::array<float, g_loopCount * RC_MAX_TIMERS> generateThesisTimes(BuildContext &context, const InputGeom &pGeom, const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans, const bool filterLowHangingObstacles, rcConfig &config) {
  context.resetLog();
  std::array<float, g_loopCount * RC_MAX_TIMERS> times{};
  for (int i{}; i < g_loopCount; i++) {
    int *pEdges{nullptr};
    int edgesSize{};
    const bool succes{runThesis(context, pGeom, filterLedgeSpans, filterWalkableLowHeightSpans, filterLowHangingObstacles, config, pEdges, edgesSize)};
    if (!succes)
      context.dumpLog("Thesis Error: ");
    REQUIRE(succes);
    rcFree(pEdges);
    const int offset{i * RC_MAX_TIMERS};
    for (int j = 0; j < RC_MAX_TIMERS; ++j) {
      times[offset + j] = static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f;
    }
  }
  return times;
}

inline std::array<float, g_loopCount * RC_MAX_TIMERS> generateSingleMeshTimes(BuildContext &context, const InputGeom &pGeom, const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans, const bool filterLowHangingObstacles, rcConfig &config) {
  context.resetLog();
  std::array<float, g_loopCount * RC_MAX_TIMERS> times{};
  for (int i{}; i < g_loopCount; i++) {
    rcPolyMesh *pMesh{nullptr};
    rcPolyMeshDetail *pDMesh{nullptr};
    const bool succes{generateSingle(context, pGeom, config, filterLowHangingObstacles, filterLedgeSpans, filterWalkableLowHeightSpans, pMesh, pDMesh)};
    if (!succes)
      context.dumpLog("Defualt Error: ");
    REQUIRE(succes);
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

inline void writeTimeToCsv(const std::string &filePath, const std::array<float, g_loopCount * RC_MAX_TIMERS> &timerData, const char *header, const int headerSize) {
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

TEST_CASE("Watershed") {
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
  std::string output{"Data"};
  std::filesystem::create_directories(output);

  SECTION("City") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);
    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"City" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }
  SECTION("Maze8") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);

    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"Maze8" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }
  SECTION("Maze16") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);

    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"Maze16" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }
  SECTION("Maze32") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);

    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"Maze32" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }

  SECTION("Maze64") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);

    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"Maze64" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }
  SECTION("Maze128") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);

    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"Maze128" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }
  SECTION("Military") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);

    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"Military" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }
  SECTION("Simple") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);

    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"Simple" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }
  SECTION("University") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);

    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"University" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }
  SECTION("Zelda") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);

    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"Zelda" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }
  SECTION("Zelda2x2") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);

    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"Zelda2x2" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }
  SECTION("Zelda4x4") {
    rcConfig config{
        .cs = 0.5f,
        .ch = g_cellHeight,
        .walkableSlopeAngle = g_agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
        .maxSimplificationError = g_edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
        .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
    };
    BuildContext context{};
    InputGeom pGeom{};
    bool success = pGeom.load(&context, "Meshes/Maze8.obj");
    if (!success)
      context.dumpLog("Geom load log %s:", "Meshes/Maze8.obj");
    REQUIRE(success);

    for (int i{5}; i > 0; --i) {
      config.cs = static_cast<float>(i) * 0.1f;
      const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};
      const std::array thesisTimes{generateThesisTimes(context, pGeom, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, g_filterLowHangingObstacles, config)};

      const std::string prefix{"Zelda4x4" + std::to_string(i) + "_"};
      writeTimeToCsv(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
      writeTimeToCsv(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
    }
  }
}