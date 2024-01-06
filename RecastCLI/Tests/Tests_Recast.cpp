#include <fstream>
#include <sstream>
#include <string>

#define CATCH_CONFIG_MAIN

#include <filesystem>

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
constexpr bool filterLedgeSpans = true;
constexpr bool filterWalkableLowHeightSpans = true;
constexpr bool filterLowHangingObstacles = true;

constexpr int LOOP_COUNT = 10;

TEST_CASE("Watershed")
{
    rcConfig config{};
    config.ch = cellHeight;
    config.walkableSlopeAngle = agentMaxSlope;
    config.walkableHeight = static_cast<int>(std::ceil(agentHeight / config.ch));
    config.walkableClimb = static_cast<int>(std::floor(agentMaxClimb / config.ch));
    config.maxSimplificationError = edgeMaxError;
    config.maxVertsPerPoly = static_cast<int>(vertsPerPoly);
    config.detailSampleMaxError = cellHeight * detailSampleMaxError;

    auto env = GENERATE(
        Catch::Generators::values<std::string>({
            "Meshes/City.obj",
            "Meshes/military.obj",
            "Meshes/Simple.obj",
            "Meshes/univerity.obj",
            "Meshes/zelda.obj",
            "Meshes/zelda2x2.obj",
            "Meshes/zelda4x4.obj"
            }));
    auto cellS = GENERATE(
        Catch::Generators::values<float>({0.1f, 0.15f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.45f, 0.5f, 0.6f}));
    auto agentR = GENERATE(
        Catch::Generators::values<float>({0.0f, 0.25f, 0.5f}));
    SECTION("Thesis")
    {
        float mergeS = 20.f;
        float minS = 8.f;
        BuildContext context{};
        auto pGeom{new(std::nothrow) InputGeom};
        REQUIRE(pGeom != nullptr);
        bool success = pGeom->load(&context, env);
        REQUIRE(success);

        config.cs = cellS;
        config.maxEdgeLen = static_cast<int>(edgeMaxLen / cellS);
        config.walkableRadius = static_cast<int>(std::ceil(agentR / config.cs));
        config.minRegionArea = static_cast<int>(rcSqr(minS));
        config.mergeRegionArea = static_cast<int>(rcSqr(mergeS));
        config.detailSampleDist = cellS * detailSampleDist;
        float totalBuildTimeMs{};
        std::stringstream ssDefault{};
        std::stringstream ssThesis{};
        for (int i{}; i < LOOP_COUNT; i++)
        {
            rcPolyMesh* m_pmesh{nullptr};
            rcPolyMeshDetail* m_dmesh{nullptr};
            GenerateSingleMeshWaterShed(&context, pGeom, config, filterLowHangingObstacles, filterLedgeSpans,
                                        filterWalkableLowHeightSpans,
                                        totalBuildTimeMs, m_pmesh, m_dmesh);
            // todo : extract / process / save portal edges
            delete m_pmesh;
            delete m_dmesh;
            for (int j = 0; j < RC_MAX_TIMERS; ++j)
            {
                ssDefault << static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f <<
                    ',';
            }
            ssDefault << std::endl;
        }

        for (int i{}; i < LOOP_COUNT; i++)
        {
            rcPolyMesh* m_pmesh{nullptr};
            rcPolyMeshDetail* m_dmesh{nullptr};
            GenerateTheses(&context, pGeom, config, filterLowHangingObstacles, filterLedgeSpans,
                           filterWalkableLowHeightSpans,
                           totalBuildTimeMs, m_pmesh, m_dmesh);
            // todo : extract / process / save portal edges
            delete m_pmesh;
            delete m_dmesh;

            for (int j = 0; j < RC_MAX_TIMERS; ++j)
            {
                ssThesis << static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f << ',';
            }
            ssThesis << std::endl;
        }
        constexpr auto header
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
        std::filesystem::create_directories(output);
        std::ofstream csvFileDefault{std::string(output) + "/output_default.csv", std::ios::out};
        std::ofstream csvFileThesis{std::string(output) + "/output_thesis.csv", std::ios::out};
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
}
