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
#include <set>

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
    rcFreePolyMesh(pMesh);
    rcFreePolyMeshDetail(pDMesh);
    pMesh = nullptr;
    pDMesh = nullptr;
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

struct Vertex {
    int x;
    int y;
};

struct Edge {
    Vertex v1{};
    Vertex v2{};
};

inline bool compareVertex(const Vertex &v1, const Vertex &v2) {
    if (v1.x == v2.x)
        return v1.y < v2.y;
    return v1.x < v2.x;
}

inline bool compareEdges(const Edge &edge1, const Edge &edge2) {
    if (edge1.v1.x == edge2.v1.x)
        return edge1.v1.y < edge2.v1.y;
    return edge1.v1.x < edge2.v1.x;
}

inline bool operator<(const Edge &e1, const Edge &e2) { return compareEdges(e1, e2); }

inline void ProcessBourderEdges(const std::string &input, const std::string &output, BuildContext &context,
                                const InputGeom *pGeom,
                                const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans,
                                const bool filterLowHangingObstacles, rcConfig config) {
    int *pEdges{nullptr};
    int edgesSize{};
    RunThesis(context, pGeom, filterLedgeSpans, filterWalkableLowHeightSpans, filterLowHangingObstacles, config,
              pEdges,
              edgesSize);
    if (edgesSize & 1)
        return;


    // load in actual svg file
    const float *min = pGeom->getMeshBoundsMin();
    const float inverseSellSize{1.0f / config.cs};
    std::set<Edge> referenceEdgesSet;
    std::ifstream csfFileRef{input};

    std::string line;
    // Read each line from the file
    while (std::getline(csfFileRef, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<int> row{};
        // Split the line into cells using a comma as a delimiter and convert to integers
        while (std::getline(ss, cell, ',')) {
            float value = std::stof(cell);
            if ((row.size() & 3u) == 0u || (row.size() & 3u) == 2u) {
                value -= min[0];
            } else {
                value -= min[2];
            }
            row.push_back(static_cast<int>(value * inverseSellSize));
        }
        if (row[0] > row[2] || (row[0] == row[2] && row[1] > row[3])) {
            std::swap(row[0], row[2]);
            std::swap(row[1], row[3]);
        }
        referenceEdgesSet.emplace(Edge{Vertex{row[0], config.height - row[1]}, Vertex{row[2], config.height - row[3]}});
    }
    csfFileRef.close();
    std::set<Edge> resultEdgesSet{};
    for (int i = 0; i < edgesSize / 2; ++i) {
        int ii = (i + 1) % (edgesSize / 2);
        if (pEdges[i * 2] == -1 || pEdges[ii * 2] == -1)
            continue;

        if (pEdges[i * 2 + 0] > pEdges[ii * 2 + 0] || (
                pEdges[i * 2 + 0] == pEdges[ii * 2 + 0] && pEdges[ii * 2 + 1] > pEdges[ii * 2 + 1])) {
            resultEdgesSet.emplace(Edge{
                {pEdges[ii * 2 + 0], pEdges[ii * 2 + 1]}, {pEdges[i * 2 + 0], pEdges[i * 2 + 1]}
            });
        } else {
            resultEdgesSet.emplace(Edge{
                {pEdges[i * 2 + 0], pEdges[i * 2 + 1]}, {pEdges[ii * 2 + 0], pEdges[ii * 2 + 1]}
            });
        }
    }
    // std::memcpy(resultEdges.data(), pEdges, edgesSize * sizeof(int));
    delete pEdges;

    std::vector<Edge> referenceEdges{};
    std::vector<Edge> resultEdges{};
    std::ranges::copy(referenceEdgesSet, std::back_inserter(referenceEdges));
    std::ranges::copy(resultEdgesSet, std::back_inserter(resultEdges));

    // std::ranges::for_each(referenceEdges, [](Edge &edge) {
    //     if (edge.v1.x > edge.v2.x || (
    //             edge.v1.x == edge.v2.x && edge.v1.y > edge.v2.y)) {
    //     }
    // });
    // std::ranges::for_each(resultEdges, [](Edge &edge) {
    //     if (edge.v1.x > edge.v2.x || (
    //             edge.v1.x == edge.v2.x && edge.v1.y > edge.v2.y)) {
    //         std::swap(edge.v1.x, edge.v2.x);
    //         std::swap(edge.v1.y, edge.v2.y);
    //     }
    // });
    // std::ranges::sort(referenceEdges, compareEdges);
    // std::ranges::sort(resultEdges, compareEdges);

    std::filesystem::create_directories(output);
    std::ofstream resultSvg{output + "/output_edges_result.svg"};
    std::ofstream referenceSvg{output + "/output_edges_ref.svg"};
    resultSvg << std::format(R"(<svg width="{}" height="{}" xmlns="http://www.w3.org/2000/svg">)", config.width,
                             config.height);
    referenceSvg << std::format(R"(<svg width="{}" height="{}" xmlns="http://www.w3.org/2000/svg">)", config.width,
                                config.height);
    resultSvg.put(resultSvg.widen('\n'));
    referenceSvg.put(referenceSvg.widen('\n'));
    const size_t maximum{std::max(referenceEdges.size(), resultEdges.size())};
    for (int i = 0; i < maximum; ++i) {
        if (i < resultEdges.size()) {
            const auto &[v1, v2]{resultEdges[i]};
            resultSvg << std::format(
                R"(<line x1="{}" y1="{}" x2="{}" y2="{}" style="stroke: black; stroke-width: 2;" />)",
                v1.x, v1.y, v2.x, v2.y) << '\n';
        }
        if (i < referenceEdges.size()) {
            const auto &[v1, v2]{referenceEdges[i]};
            referenceSvg << std::format(
                R"(<line x1="{}" y1="{}" x2="{}" y2="{}" style="stroke: black; stroke-width: 2;" />)",
                v1.x, v1.y, v2.x, v2.y) << '\n';
        }
    }
    resultSvg << R"(</svg>)";
    referenceSvg << R"(</svg>)";
    resultSvg.close();
    referenceSvg.close();

    uint32_t tp{};
    uint32_t fp{};
    for (const auto &[resultV1, resultV2]: resultEdges) {
        bool found = false;
        for (const auto &[refV1, refV2]: referenceEdges) {
            if (resultV1.x == refV1.x && resultV1.y == refV1.y && resultV2.x == refV2.x && resultV2.y == refV2.y) {
                found = true;
                break;
            }
        }
        if (found) {
            ++tp;
        } else {
            ++fp;
        }
    }

    float precision = static_cast<float>(tp) / static_cast<float>(tp + fp);
    float recall = static_cast<float>(tp) / static_cast<float>(referenceEdges.size());
    std::cout << "precision: " << precision << "\t recal: " << recall << std::endl;
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
    std::string lcmRef{};
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
    auto pGeom{ std::make_unique<InputGeom>()};
    if (!pGeom || !pGeom->load(&context, fileName)) {
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
    if (parser.cmdOptionExists("-lcm;--localclearanceminimum")) {
        aqquireLCM = true;
        lcmRef = parser.getCmdOption("-lcmr;--localclearanceminimumrefference");
    }

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
        ProcessBourderEdges(lcmRef, output, context, pGeom.get(), filterLedgeSpans, filterWalkableLowHeightSpans,
                            filterLowHangingObstacles, config);
    else
        GenerateTimes(output, context, pGeom.get(), filterLedgeSpans, filterWalkableLowHeightSpans,
                      filterLowHangingObstacles,
                      config);

    return 0;
}
