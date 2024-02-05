#include <fstream>
#include <sstream>
#include <string>

#define CATCH_CONFIG_MAIN

#include <filesystem>
#include <iostream>

#include "catch2/catch_all.hpp"

#include "Recast.h"
#include "InputGeom.h"
#include "Generators.h"
#include "BuildContext.h"

// For comparing to rcVector in benchmarks.
constexpr float cellHeight = 0.2f;
constexpr float agentRadius = 0.0f;
constexpr float agentHeight = 2.0f;
constexpr float agentMaxClimb = 0.9f;
constexpr float agentMaxSlope = 45.0f;
constexpr float edgeMaxLen = 12.0f;
constexpr float edgeMaxError = 1.3f;
constexpr float vertsPerPoly = 6.0f;
constexpr float detailSampleDist = 6.0f;
constexpr float detailSampleMaxError = 1.0f;
constexpr float mergeS = 20.0f;
constexpr float minS = 8.0f;
constexpr bool filterLedgeSpans = true;
constexpr bool filterWalkableLowHeightSpans = true;
constexpr bool filterLowHangingObstacles = true;

constexpr int LOOP_COUNT = 10;

inline bool RunThesis(BuildContext &context, const InputGeom *pGeom, const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans, const bool filterLowHangingObstacles, rcConfig &config, int *&pEdges, int &edgesSize) {
    rcPolyMesh *pMesh{nullptr};
    rcPolyMeshDetail *pDMesh{nullptr};
    float totalBuildTimeMs{};
    const bool success{ GenerateTheses(&context, pGeom, config, filterLowHangingObstacles, filterLedgeSpans, filterWalkableLowHeightSpans, totalBuildTimeMs, pMesh, pDMesh, pEdges, edgesSize) };
    rcFreePolyMesh(pMesh);
    rcFreePolyMeshDetail(pDMesh);
    pMesh = nullptr;
    pDMesh = nullptr;

    return success;
}

inline std::array<float, LOOP_COUNT * RC_MAX_TIMERS> GenerateThesisTimes(BuildContext &context, const InputGeom *pGeom, const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans, const bool filterLowHangingObstacles, rcConfig &config) {
    std::array<float, LOOP_COUNT * RC_MAX_TIMERS> times{};
    for (int i{}; i < LOOP_COUNT; i++) {
        int *pEdges{nullptr};
        int edgesSize{};
        bool succes{ RunThesis(context, pGeom, filterLedgeSpans, filterWalkableLowHeightSpans, filterLowHangingObstacles, config, pEdges, edgesSize) };
        REQUIRE(succes);
        delete pEdges;
        const int offset{i * RC_MAX_TIMERS};
        for (int j = 0; j < RC_MAX_TIMERS; ++j) {
            times[offset + j] = static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f;
        }
    }
    return times;
}

inline std::array<float, LOOP_COUNT * RC_MAX_TIMERS> GenerateSingleMeshTimes(BuildContext &context, const InputGeom *pGeom, const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans, const bool filterLowHangingObstacles, rcConfig &config) {
    std::array<float, LOOP_COUNT * RC_MAX_TIMERS> times{};
    for (int i{}; i < LOOP_COUNT; i++) {
        rcPolyMesh *pMesh{nullptr};
        rcPolyMeshDetail *pDMesh{nullptr};
        float totalBuildTimeMs{};
        bool succes{
            GenerateSingleMeshWaterShed(&context, pGeom, config, filterLowHangingObstacles, filterLedgeSpans,
                                        filterWalkableLowHeightSpans,
                                        totalBuildTimeMs, pMesh, pDMesh)
        };
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

inline void WriteTimeToCSV(const std::string &filePath, const std::array<float, LOOP_COUNT * RC_MAX_TIMERS> &timerData,
                           const char *header, const int headerSize) {
    std::ofstream csvFile{filePath, std::ios::out};
    csvFile.write(header, headerSize).put('\n');
    for (int i{}; i < LOOP_COUNT; ++i) {
        for (int j{}; j < RC_MAX_TIMERS; ++j) {
            csvFile << timerData[i * RC_MAX_TIMERS + j] << ',';
        }
        csvFile << std::endl;
    }
    csvFile.close();
}

TEST_CASE("Watershed") {
    rcConfig config{
        .ch = cellHeight,
        .walkableSlopeAngle = agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(agentHeight / cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(agentMaxClimb / cellHeight)),
        .maxSimplificationError = edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(minS)),
        .mergeRegionArea = static_cast<int>(rcSqr(mergeS)),
        .maxVertsPerPoly = static_cast<int>(vertsPerPoly),
        .detailSampleMaxError = cellHeight * detailSampleMaxError,
    };

    const std::string env{
        GENERATE(Catch::Generators::values<std::string>({
            "Meshes/City.obj",
            "Meshes/military.obj",
            "Meshes/Simple.obj",
            "Meshes/University.obj",
            "Meshes/Zelda.obj",
            "Meshes/Zelda2x2.obj",
            "Meshes/Zelda4x4.obj"
            }))
    };

    BuildContext context{};
    std::cout << "Creating Geometry" << std::endl;
    auto pGeom{new(std::nothrow) InputGeom{}};
    REQUIRE(pGeom != nullptr);
    std::cout << "Loading Geometry: " << env << std::endl;
    bool success = pGeom->load(&context, env);
    REQUIRE(success);

    const float cellS{GENERATE(Catch::Generators::range(0.1f,0.5f,0.1f))};


    const std::array defaultTimes{
        GenerateSingleMeshTimes(context, pGeom, filterLedgeSpans, filterWalkableLowHeightSpans,
                                filterLowHangingObstacles,
                                config)
    };
    const std::array thesisTimes{
        GenerateThesisTimes(context, pGeom, filterLedgeSpans, filterWalkableLowHeightSpans,
                            filterLowHangingObstacles,
                            config)
    };
    delete pGeom;

    constexpr char header[]
    {
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
        "Merge Polymesh Details (ms),"
    };
    constexpr auto output{"Data"};
    const std::string prefix{env + std::to_string(cellS) + "_"};
    std::filesystem::create_directories(output);
    WriteTimeToCSV(output + '/' + prefix + "default.csv", defaultTimes, header, sizeof header);
    WriteTimeToCSV(output + '/' + prefix + "thesis.csv", thesisTimes, header, sizeof header);
}
