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
constexpr float agentHeight = 2.0f;
constexpr float agentMaxClimb = 0.9f;
constexpr float agentMaxSlope = 45.0f;
constexpr float edgeMaxLen = 12.0f;
constexpr float edgeMaxError = 1.3f;
constexpr float vertsPerPoly = 6.0f;
constexpr float detailSampleDist = 6.0f;
constexpr float detailSampleMaxError = 1.0f;
constexpr float mergeS = 20.f;
constexpr float minS = 8.f;
constexpr bool filterLedgeSpans = true;
constexpr bool filterWalkableLowHeightSpans = true;
constexpr bool filterLowHangingObstacles = true;

constexpr int LOOP_COUNT = 10;

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

    std::string env{
        GENERATE(Catch::Generators::values<std::string>({
            "Meshes/City.obj",
            "Meshes/military.obj",
            "Meshes/Simple.obj",
            "Meshes/Univerity.obj",
            "Meshes/Zelda.obj",
            "Meshes/Zelda2x2.obj",
            "Meshes/Zelda4x4.obj"
            }))
    };

    BuildContext context{};
    std::cout << "Creating Geometry" << std::endl;
    std::unique_ptr<InputGeom> pGeom{new(std::nothrow) InputGeom{}};
    REQUIRE(pGeom != nullptr);
    std::cout << "Loading Geometry: " << env << std::endl;
    bool success = pGeom->load(&context, env);
    REQUIRE(success);

    const float cellS{GENERATE(Catch::Generators::range(0.1f,0.5f,0.1f))};
    const float agentR{GENERATE(Catch::Generators::range(0.f,0.5f,0.25f))};

    config.cs = cellS;
    config.maxEdgeLen = static_cast<int>(edgeMaxLen / cellS);
    config.walkableRadius = static_cast<int>(std::ceil(agentR / config.cs));
    config.detailSampleDist = cellS * detailSampleDist;

    float totalBuildTimeMs{};
    std::stringstream ssDefault{};
    std::stringstream ssThesis{};

    for (int i{}; i < LOOP_COUNT; i++) {
        std::unique_ptr<rcPolyMesh> pMesh;
        std::unique_ptr<rcPolyMeshDetail> pDMesh;

        rcPolyMesh *pPolyTemp{pMesh.get()};
        rcPolyMeshDetail *pPolyDetailTemp{pDMesh.get()};
        GenerateSingleMeshWaterShed(&context, pGeom.get(), config, filterLowHangingObstacles, filterLedgeSpans,
                                    filterWalkableLowHeightSpans, totalBuildTimeMs, pPolyTemp, pPolyDetailTemp);

        for (int j = 0; j < RC_MAX_TIMERS; ++j) {
            ssDefault << static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f << ',';
        }
        ssDefault << std::endl;
    }

    for (int i{}; i < LOOP_COUNT; i++) {
        std::unique_ptr<rcPolyMesh> pMesh;
        std::unique_ptr<rcPolyMeshDetail> pDMesh;
        std::unique_ptr<int[]> pEdges;

        rcPolyMesh *pPolyTemp{pMesh.get()};
        rcPolyMeshDetail *pPolyDetailTemp{pDMesh.get()};
        int *pPedgesTemp{pEdges.get()};
        int edgesSize{};
        GenerateTheses(&context, pGeom.get(), config, filterLowHangingObstacles, filterLedgeSpans,
                       filterWalkableLowHeightSpans, totalBuildTimeMs, pPolyTemp, pPolyDetailTemp, pPedgesTemp,
                       edgesSize);

        for (int j = 0; j < RC_MAX_TIMERS; ++j) {
            ssThesis << static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f << ',';
        }
        ssThesis << std::endl;
    }

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
        "Build Layers (ms),"
        "Build Polymesh Detail (ms),"
        "Merge Polymesh Details (ms),"
    };
    constexpr auto output{"Data"};
    const std::string prefix{env + std::to_string(cellS) + "_" + std::to_string(agentR) + "_"};
    std::filesystem::create_directories(output);
    std::ofstream csvFileDefault{std::string(output) + '/' + prefix + "_output_default.csv", std::ios::out};
    std::ofstream csvFileThesis{std::string(output) + '/' + prefix + "_output_thesis.csv", std::ios::out};

    csvFileDefault.write(header, sizeof header).write("\n", 1)
            .write(ssDefault.str().c_str(), ssDefault.gcount())
            .flush();

    csvFileThesis.write(header, sizeof header)
            .write("\n", 1)
            .write(ssDefault.str().c_str(), ssDefault.gcount())
            .flush();

    csvFileDefault.close();
    csvFileThesis.close();
}
