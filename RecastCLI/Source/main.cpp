//
// Created by joran on 14/12/2023.
//
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "BuildContext.h"
#include "Generators.h"
#include "InputGeom.h"
#include "Recast.h"

class InputParser
{
public:
    InputParser(const int argc, char** argv)
    {
        for (int i = 1; i < argc; ++i)
        {
            m_tokens.emplace_back(argv[i]);
            if (m_tokens.back()[0] == '-') for (char& ch : m_tokens.back()) ch = static_cast<char>(tolower(ch));
        }
    }

    /// @author iain
    [[nodiscard]] const std::string& getCmdOption(const std::string& option) const
    {
        if (const auto& itr = std::find_if(m_tokens.begin(), m_tokens.end(), [option](const std::string& token)
        {
            std::stringstream ss{option};
            std::string s;
            while (std::getline(ss, s, ';'))
            {
                return s == token;
            }
            return false;
        }); itr != m_tokens.cend() && itr + 1 != m_tokens.cend())
        {
            return *(itr + 1);
        }
        static std::string empty{};
        return empty;
    }

    /// @author iain
    [[nodiscard]] bool cmdOptionExists(const std::string& option) const
    {
        const auto& iter = std::find_if(m_tokens.begin(), m_tokens.end(), [option](const std::string& token)
        {
            std::stringstream ss{option};
            std::string s;
            while (std::getline(ss, s, ';'))
            {
                return s.find(token) != std::string::npos;
            }
            return false;
        });
        return iter != m_tokens.end();
    }

private:
    std::vector<std::string> m_tokens;
};

void printOptions()
{
    std::cout << "------------------------------------------------------------------------------------------------" <<
        std::endl;
    std::cout << "Usage: ./RecastCLI -f <input_file.obj> -o <output_directory> -g <navmesh_generator> [options]" <<
        std::endl;
    std::cout << "------------------------------------------------------------------------------------------------" <<
        std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "-h;--help\t\tPrint Out Commands and Quit" << std::endl;
    std::cout << "------------------------------------------------------------------------------------------------" <<
        std::endl;
    std::cout << "-f;--file\t\tDeclare Input environment (.obj)" << std::endl;
    std::cout << "-o;--open\t\tDeclare Output directory" << std::endl;
    std::cout << "------------------------------------------------------------------------------------------------" <<
        std::endl;
    std::cout << "-cs;--cellsize\t\t\t(optional) cell size (float)" << std::endl;
    std::cout << "-ch;--cellheight\t\t(optional) cell height (float)" << std::endl;
    std::cout << "-ah;--agentheight\t\t(optional) agent height (float)" << std::endl;
    std::cout << "-ar;--agentradius\t\t(optional) agent radius (float)" << std::endl;
    std::cout << "-amc;--agentmaxclimb\t\t(optional) agent max climb (float)" << std::endl;
    std::cout << "-ams;--agentmaxslope\t\t(optional) agent max slope (float)" << std::endl;
    std::cout << "-rms;--regionminsize\t\t(optional) region min size (float)" << std::endl;
    std::cout << "-rmms;--regionmergesize\t\t(optional) region merge size (float)" << std::endl;
    std::cout << "-eml;--edgemaxlen\t\t(optional) edge max length (float)" << std::endl;
    std::cout << "-eme;--edgemaxerror\t\t(optional) edge max error (float)" << std::endl;
    std::cout << "-vpp;--vertsperpoly\t\t(optional) vertices per polygon (float)" << std::endl;
    std::cout << "-dsd;--detailsampledist\t\t(optional) detail sample distance (float)" << std::endl;
    std::cout << "-dsme;--detailsamplemaxerror\t(optional) detail sample max error (float)" << std::endl;
    std::cout << "------------------------------------------------------------------------------------------------" <<
        std::endl;
}

constexpr int LOOP_COUNT = 10;

int main(const int argc, char* argv[])
{
    const InputParser parser(argc, argv);
    if (parser.cmdOptionExists("-h;--help"))
    {
        printOptions();
        return 0;
    }
    const std::string& fileName = parser.getCmdOption("-f;--file");
    const std::string& output = parser.getCmdOption("-o;--output");
    bool shouldFial{};
    if (fileName.empty())
    {
        std::cout << "An input file model is required (-f;--file)" << std::endl;
        shouldFial = true;
    }
    if (output.empty())
    {
        std::cout << "An output path required (-o;--output)" << std::endl;
        shouldFial = true;
    }

    BuildContext context{};
    auto pGeom{new(std::nothrow) InputGeom};
    if (!pGeom || !pGeom->load(&context, fileName))
    {
        delete pGeom;
        pGeom = nullptr;
        context.dumpLog("Geom load log %s:", fileName.c_str());
    }
    if (shouldFial)
        return 1;

    float cellSize = 0.3f;
    float cellHeight = 0.2f;
    float agentRadius = 0.6f;
    float agentHeight = 2.0f;
    float agentMaxClimb = 0.9f;
    float agentMaxSlope = 45.0f;
    float edgeMaxLen = 12.0f;
    float regionMinSize = 8.0f;
    float regionMergeSize = 20.0f;
    float edgeMaxError = 1.3f;
    float vertsPerPoly = 6.0f;
    float detailSampleDist = 6.0f;
    float detailSampleMaxError = 1.0f;
    constexpr bool filterLedgeSpans = true;
    constexpr bool filterWalkableLowHeightSpans = true;
    constexpr bool filterLowHangingObstacles = true;

    if (parser.cmdOptionExists("-cs;--cellsize"))
        cellSize = std::stof(parser.getCmdOption("-cs;--cellsize"));
    if (parser.cmdOptionExists("-ch;--cellheight"))
        cellHeight = std::stof(parser.getCmdOption("-ch;--cellheight"));
    if (parser.cmdOptionExists("-ah;--agentheight"))
        agentHeight = std::stof(parser.getCmdOption("-ah;--agentheight"));
    if (parser.cmdOptionExists("-ar;--agentradius"))
        agentRadius = std::stof(parser.getCmdOption("-ar;--agentradius"));
    if (parser.cmdOptionExists("-amc;--agentmaxclimb"))
        agentMaxClimb = std::stof(parser.getCmdOption("-amc;--agentmaxclimb"));
    if (parser.cmdOptionExists("-ams;--agentmaxslope"))
        agentMaxSlope = std::stof(parser.getCmdOption("-ams;--agentmaxslope"));
    if (parser.cmdOptionExists("-ams;--regionminsize"))
        regionMinSize = std::stof(parser.getCmdOption("-rms;--regionminsize"));
    if (parser.cmdOptionExists("-cs;--regionmergesize"))
        regionMergeSize = std::stof(parser.getCmdOption("-rmms;--regionmergesize"));
    if (parser.cmdOptionExists("-eml;--edgemaxlen"))
        edgeMaxLen = std::stof(parser.getCmdOption("-eml;--edgemaxlen"));
    if (parser.cmdOptionExists("-eme;--edgemaxerror"))
        edgeMaxError = std::stof(parser.getCmdOption("-eme;--edgemaxerror"));
    if (parser.cmdOptionExists("-vpp;--vertsperpoly"))
        vertsPerPoly = std::stof(parser.getCmdOption("-vpp;--vertsperpoly"));
    if (parser.cmdOptionExists("-dsd;--detailsampledist"))
        detailSampleDist = std::stof(parser.getCmdOption("-dsd;--detailsampledist"));
    if (parser.cmdOptionExists("-dsme;--detailsamplemaxerror"))
        detailSampleMaxError = std::stof(parser.getCmdOption("-dsme;--detailsamplemaxerror"));

    rcConfig config{};
    config.cs = cellSize;
    config.ch = cellHeight;
    config.walkableSlopeAngle = agentMaxSlope;
    config.walkableHeight = static_cast<int>(std::ceil(agentHeight / config.ch));
    config.walkableClimb = static_cast<int>(std::floor(agentMaxClimb / config.ch));
    config.walkableRadius = static_cast<int>(std::ceil(agentRadius / config.cs));
    config.maxEdgeLen = static_cast<int>(edgeMaxLen / cellSize);
    config.maxSimplificationError = edgeMaxError;
    config.minRegionArea = static_cast<int>(rcSqr(regionMinSize));
    config.mergeRegionArea = static_cast<int>(rcSqr(regionMergeSize));
    config.maxVertsPerPoly = static_cast<int>(vertsPerPoly);
    config.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
    config.detailSampleMaxError = cellHeight * detailSampleMaxError;
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
            ssDefault << static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f << ',';
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
        "Merge Polymesh Details (ms)"
    };
    std::filesystem::create_directories(output);

    std::ofstream csvFileDefault{output + "/output_default.csv", std::ios::out};
    csvFileDefault.write(header, sizeof (header))
                  .put(csvFileDefault.widen('\n'))
                  .write(ssDefault.str().c_str(), static_cast<int64_t>(ssDefault.str().size()))
                  .flush();
    csvFileDefault.close();

    std::ofstream csvFileThesis{output + "/output_thesis.csv", std::ios::out};
    csvFileThesis.write(header, sizeof header)
                 .put(csvFileThesis.widen('\n'))
                 .write(ssThesis.str().c_str(), static_cast<int64_t>(ssThesis.str().size()))
                 .flush();
    csvFileThesis.close();

    return 0;
}
