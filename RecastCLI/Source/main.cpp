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
#include <array>

#include "BuildContext.h"
#include "Generators.h"
#include "InputGeom.h"
#include "Recast.h"

class InputParser {
public:
    InputParser(const int argc, char **argv) {
        for (int i = 1; i < argc; ++i) {
            m_tokens.emplace_back(argv[i]);
            if (m_tokens.back()[0] == '-') for (char &ch: m_tokens.back()) ch = static_cast<char>(tolower(ch));
        }
    }

    /// @author iain
    [[nodiscard]] const std::string &getCmdOption(const std::string &option) const {
        if (const auto &itr = std::ranges::find_if(m_tokens, [option](const std::string &token) {
            std::stringstream ss{option};
            std::string s;
            while (std::getline(ss, s, ';')) {
                return s == token;
            }
            return false;
        }); itr != m_tokens.cend() && itr + 1 != m_tokens.cend()) {
            return *(itr + 1);
        }
        static std::string empty{};
        return empty;
    }

    /// @author iain
    [[nodiscard]] bool cmdOptionExists(const std::string &option) const {
        const auto &iter = std::ranges::find_if(m_tokens, [option](const std::string &token) {
            std::stringstream ss{option};
            std::string s;
            while (std::getline(ss, s, ';')) {
                return s.find(token) != std::string::npos;
            }
            return false;
        });
        return iter != m_tokens.end();
    }

private:
    std::vector<std::string> m_tokens;
};

void printOptions() {
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
    std::cout << "-ar;--agentradius\t\t(optional) agent radius (float)" << std::endl;
    std::cout << "------------------------------------------------------------------------------------------------" <<
            std::endl;
}

constexpr int LOOP_COUNT = 100;

inline void RunThesis(BuildContext &context, const InputGeom *pGeom, const bool filterLedgeSpans,
                      const bool filterWalkableLowHeightSpans, const bool filterLowHangingObstacles, rcConfig &config,
                      int *&pEdges, int &edgesSize) {
    rcPolyMesh *pMesh{nullptr};
    rcPolyMeshDetail *pDMesh{nullptr};
    float totalBuildTimeMs{};
    GenerateTheses(&context, pGeom, config, filterLowHangingObstacles, filterLedgeSpans,
                   filterWalkableLowHeightSpans,
                   totalBuildTimeMs, pMesh, pDMesh, pEdges, edgesSize);
    delete pMesh;
    delete pDMesh;
}

inline std::array<float, LOOP_COUNT * RC_MAX_TIMERS> GenerateThesisTimes(
    BuildContext &context, const InputGeom *pGeom, const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans,
    const bool filterLowHangingObstacles, rcConfig &config) {
    std::array<float, LOOP_COUNT * RC_MAX_TIMERS> times{};
    for (int i{}; i < LOOP_COUNT; i++) {
        int *pEdges{nullptr};
        int edgesSize{};
        RunThesis(context, pGeom, filterLedgeSpans, filterWalkableLowHeightSpans, filterLowHangingObstacles, config,
                  pEdges,
                  edgesSize);
        delete pEdges;
        const int offset{i * RC_MAX_TIMERS};
        for (int j = 0; j < RC_MAX_TIMERS; ++j) {
            times[offset + j] = static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f;
        }
    }
    return times;
}

inline std::array<float, LOOP_COUNT * RC_MAX_TIMERS> GenerateSingleMeshTimes(
    BuildContext &context, const InputGeom *pGeom,
    const bool filterLedgeSpans,
    const bool filterWalkableLowHeightSpans,
    const bool filterLowHangingObstacles,
    rcConfig &config) {
    std::array<float, LOOP_COUNT * RC_MAX_TIMERS> times{};
    for (int i{}; i < LOOP_COUNT; i++) {
        rcPolyMesh *pMesh{nullptr};
        rcPolyMeshDetail *pDMesh{nullptr};
        float totalBuildTimeMs{};
        GenerateSingleMeshWaterShed(&context, pGeom, config, filterLowHangingObstacles, filterLedgeSpans,
                                    filterWalkableLowHeightSpans,
                                    totalBuildTimeMs, pMesh, pDMesh);
        delete pMesh;
        delete pDMesh;
        const int offset{i * RC_MAX_TIMERS};
        for (int j = 0; j < RC_MAX_TIMERS; ++j) {
            times[offset + j] = static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f;
        }
    }
    return times;
}

inline void writeCsvFile(const std::string &filePath, const std::array<float, LOOP_COUNT * RC_MAX_TIMERS> &timerData,
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

inline void GenerateTimes(const std::string &output, BuildContext &context, const InputGeom *pGeom,
                          const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans,
                          const bool filterLowHangingObstacles, rcConfig config) {
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
    std::filesystem::create_directories(output);
    writeCsvFile(output + "/output_default.csv", defaultTimes, header, sizeof header);
    writeCsvFile(output + "/output_thesis.csv", thesisTimes, header, sizeof header);
}

struct Edge {
    int vertices[8];
};

inline bool compareEdges(const Edge &edge1, const Edge &edge2) {
    for (int i = 0; i < 4; ++i) {
        if (edge1.vertices[i] != edge2.vertices[i]) {
            return edge1.vertices[i] < edge2.vertices[i];
        }
    }
    return false; // The edges are equal
}

inline void ProcessBourderEdges(const std::string &output, BuildContext &context, const InputGeom *pGeom,
                                const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans,
                                const bool filterLowHangingObstacles, rcConfig config) {
    int *pEdges{nullptr};
    int edgesSize{};
    RunThesis(context, pGeom, filterLedgeSpans, filterWalkableLowHeightSpans, filterLowHangingObstacles, config,
              pEdges,
              edgesSize);
    if (edgesSize & 3)
        return;

    const auto edgeArray = reinterpret_cast<Edge *>(pEdges);
    if (!edgeArray) { return; }

    std::sort(edgeArray, edgeArray + (edgesSize >> 3), compareEdges);

    std::filesystem::create_directories(output);
    std::ofstream csvFile{output + "/output_edges.csv", std::ios::out};
    const float *min = pGeom->getNavMeshBoundsMin();
    for (int i{}; i < edgesSize / 8; ++i) {
        csvFile << static_cast<float>(pEdges[i * 8 + 0]) * config.cs + min[0] << ','
        << static_cast<float>(pEdges[i * 8 + 2]) * config.cs + min[2] << ','
        << static_cast<float>(pEdges[i * 8 + 4]) * config.cs + min[0] << ','
        << static_cast<float>(pEdges[i * 8 + 6]) * config.cs + min[2] << std::endl;
    }
    csvFile.close();
    delete pEdges;
}

constexpr float cellHeight = 0.2f;
constexpr float agentHeight = 2.0f;
constexpr float agentMaxClimb = 0.9f;
constexpr float agentMaxSlope = 45.0f;
constexpr float edgeMaxLen = 12.0f;
constexpr float regionMinSize = 8.0f;
constexpr float regionMergeSize = 20.0f;
constexpr float edgeMaxError = 1.3f;
constexpr float vertsPerPoly = 6.0f;
constexpr float detailSampleDist = 6.0f;
constexpr float detailSampleMaxError = 1.0f;
constexpr bool filterLedgeSpans = true;
constexpr bool filterWalkableLowHeightSpans = true;
constexpr bool filterLowHangingObstacles = true;

int main(const int argc, char *argv[]) {
    const InputParser parser(argc, argv);
    if (parser.cmdOptionExists("-h;--help")) {
        printOptions();
        return 0;
    }
    const std::string &fileName = parser.getCmdOption("-f;--file");
    const std::string &output = parser.getCmdOption("-o;--output");
    bool shouldFial{};
    if (fileName.empty()) {
        std::cout << "An input file model is required (-f;--file)" << std::endl;
        shouldFial = true;
    }
    if (output.empty()) {
        std::cout << "An output path required (-o;--output)" << std::endl;
        shouldFial = true;
    }

    BuildContext context{};
    auto pGeom{new(std::nothrow) InputGeom};
    if (!pGeom || !pGeom->load(&context, fileName)) {
        delete pGeom;
        pGeom = nullptr;
        context.dumpLog("Geom load log %s:", fileName.c_str());
    }
    if (shouldFial)
        return 1;

    float cellSize = 0.3f;
    float agentRadius = 0.6f;

    bool aqquireLCM{};
    if (parser.cmdOptionExists("-cs;--cellsize"))
        cellSize = std::stof(parser.getCmdOption("-cs;--cellsize"));
    if (parser.cmdOptionExists("-ar;--agentradius"))
        agentRadius = std::stof(parser.getCmdOption("-ar;--agentradius"));
    if (parser.cmdOptionExists("-lcm;--localclearanceminimum"))
        aqquireLCM = true;

    const rcConfig config{
        .cs = cellSize,
        .ch = cellHeight,
        .walkableSlopeAngle = agentMaxSlope,
        .walkableHeight = static_cast<int>(std::ceil(agentHeight / cellHeight)),
        .walkableClimb = static_cast<int>(std::floor(agentMaxClimb / cellHeight)),
        .walkableRadius = static_cast<int>(std::ceil(agentRadius / cellHeight)),
        .maxEdgeLen = static_cast<int>(edgeMaxLen / cellSize),
        .maxSimplificationError = edgeMaxError,
        .minRegionArea = static_cast<int>(rcSqr(regionMinSize)),
        .mergeRegionArea = static_cast<int>(rcSqr(regionMergeSize)),
        .maxVertsPerPoly = static_cast<int>(vertsPerPoly),
        .detailSampleDist = cellSize * detailSampleDist,
        .detailSampleMaxError = cellHeight * detailSampleMaxError,
    };

    if (aqquireLCM)
        ProcessBourderEdges(output, context, pGeom, filterLedgeSpans, filterWalkableLowHeightSpans,
                            filterLowHangingObstacles, config);
    else
        GenerateTimes(output, context, pGeom, filterLedgeSpans, filterWalkableLowHeightSpans,
                      filterLowHangingObstacles,
                      config);

    return 0;
}
