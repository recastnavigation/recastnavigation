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
#include <RecastAlloc.h>
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

    const std::string &GetCmdOption(const std::string &option) const {
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

    bool CmdOptionExists(const std::string &option) const {
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

void PrintOptions() {
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

constexpr int g_loopCount = 100;

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

inline std::array<float, g_loopCount * RC_MAX_TIMERS> GenerateThesisTimes(
    BuildContext &context, const InputGeom *pGeom, const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans,
    const bool filterLowHangingObstacles, rcConfig &config) {
    std::array<float, g_loopCount * RC_MAX_TIMERS> times{};
    for (int i{}; i < g_loopCount; i++) {
        int *pEdges{nullptr};
        int edgesSize{};
        RunThesis(context, pGeom, filterLedgeSpans, filterWalkableLowHeightSpans, filterLowHangingObstacles, config,
                  pEdges,
                  edgesSize);
        rcFree(pEdges);
        const int offset{i * RC_MAX_TIMERS};
        for (int j = 0; j < RC_MAX_TIMERS; ++j) {
            times[offset + j] = static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f;
        }
    }
    return times;
}

inline std::array<float, g_loopCount * RC_MAX_TIMERS> GenerateSingleMeshTimes(
    BuildContext &context, const InputGeom *pGeom,
    const bool filterLedgeSpans,
    const bool filterWalkableLowHeightSpans,
    const bool filterLowHangingObstacles,
    rcConfig &config) {
    std::array<float, g_loopCount * RC_MAX_TIMERS> times{};
    for (int i{}; i < g_loopCount; i++) {
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

inline void WriteCsvFile(const std::string &filePath, const std::array<float, g_loopCount * RC_MAX_TIMERS> &timerData,
                         const char *header, const int headerSize) {
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
    WriteCsvFile(output + "/output_default.csv", defaultTimes, header, sizeof header);
    WriteCsvFile(output + "/output_thesis.csv", thesisTimes, header, sizeof header);
}

struct Vertex {
    int x;
    int y;
};

struct Edge {
    Vertex v1{};
    Vertex v2{};
};

inline bool CompareVertex(const Vertex &v1, const Vertex &v2) {
    if (v1.x == v2.x)
        return v1.y < v2.y;
    return v1.x < v2.x;
}

inline bool CompareEdges(const Edge &edge1, const Edge &edge2) {
    if (edge1.v1.x == edge2.v1.x)
        return edge1.v1.y < edge2.v1.y;
    return edge1.v1.x < edge2.v1.x;
}

inline bool operator<(const Edge &e1, const Edge &e2) { return CompareEdges(e1, e2); }

inline void ProcessBourderEdges(const std::string &input, const std::string &output, BuildContext &context,
                                const InputGeom *pGeom,
                                const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans,
                                const bool filterLowHangingObstacles, rcConfig config) {
    int *pEdges{nullptr};
    int edgesSize{};
    RunThesis(context, pGeom, filterLedgeSpans, filterWalkableLowHeightSpans, filterLowHangingObstacles, config,
              pEdges,
              edgesSize);
    // if (edgesSize & 1)
    // return;


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
    for (int i = 0; i < edgesSize / 2 - 1; i += 2) {
        const int ii = i + 1;
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
    rcFree(pEdges);

    std::vector<Edge> referenceEdges{};
    std::vector<Edge> resultEdges{};
    std::ranges::copy(referenceEdgesSet, std::back_inserter(referenceEdges));
    std::ranges::copy(resultEdgesSet, std::back_inserter(resultEdges));

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
    constexpr uint8_t epsilon{2};
    const auto edgeCompare{
        [epsilon](const Edge &e1, const Edge &e2)-> bool {
            if (e1.v1.x == e2.v1.x && e1.v1.y == e2.v1.y &&
                e1.v2.x == e2.v2.x && e1.v2.y == e2.v2.y)
                return true;

            const int diffX = e1.v1.x - e2.v1.x;
            const int diffY = e1.v1.y - e2.v1.y;

            // Compare the squared length of the difference with the squared epsilon
            if(diffX * diffX + diffY * diffY <= epsilon * epsilon)
                return true;

            // Calculate the direction vectors for e1 and e2
            const Vertex e2Direction{e2.v2.x - e2.v1.x, e2.v2.y - e2.v1.y};

            // Calculate the normal of e2
            Vertex e2Normal{e2Direction.y, -e2Direction.x}; // Assuming 2D

            // Normalize the normal vector
            const auto e2NormalLength = static_cast<float>(
                std::sqrt(e2Normal.x * e2Normal.x + e2Normal.y * e2Normal.y));
            e2Normal.x = static_cast<int>(static_cast<float>(e2Normal.x) / e2NormalLength);
            e2Normal.y = static_cast<int>(static_cast<float>(e2Normal.y) / e2NormalLength);

            // Project e2.v1 onto e1 using the normal of e2
            const int dotProduct1 = e2Normal.x * (e2.v1.x - e1.v1.x) + e2Normal.y * (e2.v1.y - e1.v1.y);
            const Vertex projectedPoint1{e2.v1.x - dotProduct1 * e2Normal.x, e2.v1.y - dotProduct1 * e2Normal.y};

            // Project e2.v2 onto e1 using the normal of e2
            const int dotProduct2 = e2Normal.x * (e2.v2.x - e1.v1.x) + e2Normal.y * (e2.v2.y - e1.v1.y);
            const Vertex projectedPoint2{e2.v2.x - dotProduct2 * e2Normal.x, e2.v2.y - dotProduct2 * e2Normal.y};

            // Create a new edge using the projected points
            const Edge projectedEdge{projectedPoint1.x, projectedPoint1.y, projectedPoint2.x, projectedPoint2.y};

            // Calculate the difference between the projected edge and e1
            const int projectedDiffX1 = projectedEdge.v1.x - e1.v1.x;
            const int projectedDiffY1 = projectedEdge.v1.y - e1.v1.y;

            const int projectedDiffX2 = projectedEdge.v2.x - e1.v2.x;
            const int projectedDiffY2 = projectedEdge.v2.y - e1.v2.y;
            // Compare the squared length of the difference with the squared epsilon
            return (projectedDiffX1 * projectedDiffX1 + projectedDiffY1 * projectedDiffY1 <= epsilon * epsilon)&&(projectedDiffX2 * projectedDiffX2 + projectedDiffY2 * projectedDiffY2 <= epsilon * epsilon);
        }
    };

    std::size_t referenceEdgesSize = referenceEdges.size();
    for (const auto &edge1: resultEdges) {
        bool found = false;

        for (const auto &edge2: referenceEdges) {
            if (edgeCompare(edge2, edge1)) {
                found = true;
                std::erase_if(referenceEdges, [edge2](const Edge& edge) {
                    return edge.v1.x == edge2.v1.x && edge.v1.y == edge2.v1.y && edge.v2.x == edge2.v2.x && edge.v2.y == edge2.v2.y;
                });
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
    float recall = static_cast<float>(tp) / static_cast<float>(referenceEdgesSize);
    std::cout << "precision: " << precision << "\t recal: " << recall << std::endl;
}

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

int main(const int argc, char *argv[]) {
    const InputParser parser(argc, argv);
    if (parser.CmdOptionExists("-h;--help")) {
        PrintOptions();
        return 0;
    }
    const std::string &fileName = parser.GetCmdOption("-f;--file");
    const std::string &output = parser.GetCmdOption("-o;--output");
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
    if (shouldFial)
        return 1;

    BuildContext context{};
    const auto pGeom{std::make_unique<InputGeom>()};
    if (!pGeom || !pGeom->load(&context, fileName)) {
        context.dumpLog("Geom load log %s:", fileName.c_str());
        shouldFial = true;
    }
    if (shouldFial)
        return 1;

    float cellSize = 0.3f;
    float agentRadius = 0.6f;

    bool aqquireLcm{};
    if (parser.CmdOptionExists("-cs;--cellsize"))
        cellSize = std::stof(parser.GetCmdOption("-cs;--cellsize"));
    if (parser.CmdOptionExists("-ar;--agentradius"))
        agentRadius = std::stof(parser.GetCmdOption("-ar;--agentradius"));
    if (parser.CmdOptionExists("-lcm;--localclearanceminimum")) {
        aqquireLcm = true;
        lcmRef = parser.GetCmdOption("-lcmr;--localclearanceminimumrefference");
    }

    const rcConfig config{
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

    if (aqquireLcm)
        ProcessBourderEdges(lcmRef, output, context, pGeom.get(), g_filterLedgeSpans, g_filterWalkableLowHeightSpans,
                            g_filterLowHangingObstacles, config);
    else
        GenerateTimes(output, context, pGeom.get(), g_filterLedgeSpans, g_filterWalkableLowHeightSpans,
                      g_filterLowHangingObstacles,
                      config);

    return 0;
}
