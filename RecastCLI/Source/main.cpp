//
// Created by joran on 14/12/2023.
//
#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "Recast.h"
#include "InputGeom.h"
#include "Generators.h"
#include "SampleInterfaces.h"

class InputParser
{
public:
    InputParser(const int argc, char** argv)
    {
        for (int i = 1; i < argc; ++i)
        {
            m_tokens.emplace_back(argv[i]);
            for (char& ch : m_tokens.back()) ch = static_cast<char>(tolower(ch));
        }
    }

    /// @author iain
    const std::string& getCmdOption(const std::string& option) const
    {
        const auto& itr = std::find_if(m_tokens.begin(), m_tokens.end(), [option](const std::string& token)
        {
            std::stringstream ss{option};
            std::string s;
            while (std::getline(ss, s, ';'))
            {
                return s == token;
            }
            return false;
        });
        if (itr != m_tokens.cend() && itr + 1 != m_tokens.cend())
        {
            return *(itr + 1);
        }
        static std::string empty{};
        return empty;
    }

    /// @author iain
    bool cmdOptionExists(const std::string& option) const
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

/*
 * todo add following as option
 *	float m_cellSize;
 *	float m_cellHeight;
 *	float m_agentHeight;
 *	float m_agentRadius;
 *	float m_agentMaxClimb;
 *	float m_agentMaxSlope;
 *	float m_regionMinSize;
 *	float m_regionMergeSize;
 *	float m_edgeMaxLen;
 *	float m_edgeMaxError;
 *	float m_vertsPerPoly;
 *	float m_detailSampleDist;
 *	float m_detailSampleMaxError;
 *
 *	bool m_filterLowHangingObstacles;
 *	bool m_filterLedgeSpans;
 *	bool m_filterWalkableLowHeightSpans;
 */
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
    auto pGeom{new (std::nothrow) InputGeom};
    if (!pGeom || !pGeom->load(&context, fileName))
    {
        delete pGeom;
        pGeom = nullptr;
        context.dumpLog("Geom load log %s:", fileName.c_str());
    }
    if (shouldFial)
        return 1;

    float cellSize = 0.3f;
    float m_cellHeight = 0.2f;
    float m_agentHeight = 2.0f;
    float m_agentRadius = 0.6f;
    float m_agentMaxClimb = 0.9f;
    float m_agentMaxSlope = 45.0f;
    float m_regionMinSize = 8;
    float m_regionMergeSize = 20;
    float m_edgeMaxLen = 12.0f;
    float m_edgeMaxError = 1.3f;
    float m_vertsPerPoly = 6.0f;
    float m_detailSampleDist = 6.0f;
    float m_detailSampleMaxError = 1.0f;
    const bool filterLedgeSpans = true;
    const bool filterWalkableLowHeightSpans = true;
    const bool filterLowHangingObstacles = true;
    if (parser.cmdOptionExists("-cs;--cellsize"))
        cellSize = std::stof(parser.getCmdOption("-cs;--cellsize"));
    if (parser.cmdOptionExists("-ch;--cellheight"))
        m_cellHeight = std::stof(parser.getCmdOption("-ch;--cellheight"));
    if (parser.cmdOptionExists("-cs;--cellsize"))
        m_agentHeight = std::stof(parser.getCmdOption("-ah;--agentheight"));
    if (parser.cmdOptionExists("-cs;--cellsize"))
        m_agentRadius = std::stof(parser.getCmdOption("-ar;--agentradius"));
    if (parser.cmdOptionExists("-cs;--cellsize"))
        m_agentMaxClimb = std::stof(parser.getCmdOption("-amc;--agentmaxclimb"));
    if (parser.cmdOptionExists("-cs;--cellsize"))
        m_agentMaxSlope = std::stof(parser.getCmdOption("-ams;--agentmaxslope"));
    if (parser.cmdOptionExists("-cs;--cellsize"))
        m_regionMinSize = std::stof(parser.getCmdOption("-rms;--regionminsize"));
    if (parser.cmdOptionExists("-cs;--cellsize"))
        m_regionMergeSize = std::stof(parser.getCmdOption("-rmms;--regionmergesize"));
    if (parser.cmdOptionExists("-cs;--cellsize"))
        m_edgeMaxLen = std::stof(parser.getCmdOption("-eml;--edgemaxlen"));
    if (parser.cmdOptionExists("-cs;--cellsize"))
        m_edgeMaxError = std::stof(parser.getCmdOption("-eme;--edgemaxerror"));
    if (parser.cmdOptionExists("-cs;--cellsize"))
        m_vertsPerPoly = std::stof(parser.getCmdOption("-vpp;--vertsperpoly"));
    if (parser.cmdOptionExists("-cs;--cellsize"))
        m_detailSampleDist = std::stof(parser.getCmdOption("-dsd;--detailsampledist"));
    if (parser.cmdOptionExists("-cs;--cellsize"))
        m_detailSampleMaxError = std::stof(parser.getCmdOption("-dsme;--detailsamplemaxerror"));

    rcConfig config{};
    config.cs = cellSize;
    config.ch = m_cellHeight;
    config.walkableSlopeAngle = m_agentMaxSlope;
    config.walkableHeight = static_cast<int>(ceilf(m_agentHeight / config.ch));
    config.walkableClimb = static_cast<int>(floorf(m_agentMaxClimb / config.ch));
    config.walkableRadius = static_cast<int>(ceilf(m_agentRadius / config.cs));
    config.maxEdgeLen = static_cast<int>(m_edgeMaxLen / cellSize);
    config.maxSimplificationError = m_edgeMaxError;
    config.minRegionArea = static_cast<int>(rcSqr(m_regionMinSize));
    config.mergeRegionArea = static_cast<int>(rcSqr(m_regionMergeSize));
    config.maxVertsPerPoly = static_cast<int>(m_vertsPerPoly);
    config.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : cellSize * m_detailSampleDist;
    config.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
    float totalBuildTimeMs{};

    GenerateTheses(&context, pGeom, config, filterLowHangingObstacles, filterLedgeSpans, filterWalkableLowHeightSpans,
                   totalBuildTimeMs);

    return 0;
}
