#include <fstream>
#include <sstream>
#include <string>

#define CATCH_CONFIG_MAIN
#include <iostream>

#include "catch2/catch_all.hpp"

#include "Recast.h"
#include "InputGeom.h"
#include "Generators.h"
#include "SampleInterfaces.h"

// For comparing to rcVector in benchmarks.
const std::string environments[3]{"Meshes/zelda.obj", "Meshes/zelda2x2.obj", "Meshes/zelda4x4.obj"};
const float cellSizes[]{0.1f, 0.15f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.45f, 0.5f, 0.55f, 0.6};
const float agentRadii[]{0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};
const float regionMinSizes[]{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 30, 40, 50};
const float regionMergeSizes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 30, 40, 50};
const float cellHeight = 0.2f;
const float agentHeight = 2.0f;
const float agentMaxClimb = 0.9f;
const float agentMaxSlope = 45.0f;
const float edgeMaxLen = 12.0f;
const float edgeMaxError = 1.3f;
const float vertsPerPoly = 6.0f;
const float detailSampleDist = 6.0f;
const float detailSampleMaxError = 1.0f;
const bool filterLedgeSpans = true;
const bool filterWalkableLowHeightSpans = true;
const bool filterLowHangingObstacles = true;

const int LOOP_COUNT = 10;

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
        Catch::Generators::values<std::string>({"Meshes/zelda.obj", "Meshes/zelda2x2.obj", "Meshes/zelda4x4.obj"}));
    auto cellS = GENERATE(
        Catch::Generators::values<float>({0.1f, 0.2f,0.3f,0.4f,0.5f, 0.6}));
    auto agentR = GENERATE(
        Catch::Generators::values<float>({0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f}));
    auto minS = GENERATE(Catch::Generators::values<float>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}));
    auto mergeS = GENERATE(Catch::Generators::values<float>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}));
    SECTION("Thesis")
    {
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
        std::stringstream ss{};
        for (int i{}; i < LOOP_COUNT; i++)
        {
            success = GenerateTheses(&context, pGeom, config, filterLowHangingObstacles, filterLedgeSpans,
                                     filterWalkableLowHeightSpans,
                                     totalBuildTimeMs);
            REQUIRE(success);
            for (int j = 0; j < RC_MAX_TIMERS; ++j)
            {
                ss << static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f << "ms,";
            }
            ss << std::endl;
        }
        ss << std::endl;
        for (int i{}; i < LOOP_COUNT; i++)
        {
            success = GenerateSingleMeshWaterShed(&context, pGeom, config, filterLowHangingObstacles, filterLedgeSpans,
                                                  filterWalkableLowHeightSpans,
                                                  totalBuildTimeMs);
            REQUIRE(success);
            for (int j = 0; j < RC_MAX_TIMERS; ++j)
            {
                ss << static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f << "ms,";
            }
            ss << std::endl;
        }
        std::ofstream csvFile{"output.csv", std::ios::in | std::ios::app};
        csvFile << ss.str() << std::endl;
        csvFile.close();
        std::cout << env  << ";\t" << cellS << ";\t" << agentR << ";\t" << minS << ";\t" << mergeS << std::endl;
    }
}