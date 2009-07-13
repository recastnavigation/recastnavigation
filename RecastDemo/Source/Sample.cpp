#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "imgui.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


Sample::Sample() :
	m_verts(0), m_nverts(0), m_tris(0), m_trinorms(0), m_ntris(0)
{
	resetCommonSettings();
}

Sample::~Sample()
{
}

void Sample::handleSettings()
{
}

void Sample::handleTools()
{
}

void Sample::handleDebugMode()
{
}

void Sample::handleRender()
{
	if (!m_verts || !m_tris || !m_trinorms)
		return;
	// Draw mesh
	rcDebugDrawMesh(m_verts, m_nverts, m_tris, m_trinorms, m_ntris, 0);
	// Draw bounds
	float col[4] = {1,1,1,0.5f};
	rcDebugDrawBoxWire(m_bmin[0],m_bmin[1],m_bmin[2], m_bmax[0],m_bmax[1],m_bmax[2], col);
}

void Sample::handleRenderOverlay(double* proj, double* model, int* view)
{
}

void Sample::handleMeshChanged(const float* verts, int nverts,
								const int* tris, const float* trinorms, int ntris,
								const float* bmin, const float* bmax)
{
	m_verts = verts;
	m_nverts = nverts;
	m_tris = tris;
	m_trinorms = trinorms;
	m_ntris = ntris;
	vcopy(m_bmin, bmin);
	vcopy(m_bmax, bmax);
}

void Sample::resetCommonSettings()
{
	m_cellSize = 0.3f;
	m_cellHeight = 0.2f;
	m_agentHeight = 2.0f;
	m_agentRadius = 0.6f;
	m_agentMaxClimb = 0.9f;
	m_agentMaxSlope = 45.0f;
	m_regionMinSize = 50;
	m_regionMergeSize = 20;
	m_edgeMaxLen = 12.0f;
	m_edgeMaxError = 1.3f;
	m_vertsPerPoly = 6.0f;
}

void Sample::handleCommonSettings()
{
	imguiLabel("Rasterization");
	imguiSlider("Cell Size", &m_cellSize, 0.1f, 1.0f, 0.01f);
	imguiSlider("Cell Height", &m_cellHeight, 0.1f, 1.0f, 0.01f);
	
	int gw = 0, gh = 0;
	rcCalcGridSize(m_bmin, m_bmax, m_cellSize, &gw, &gh);
	char text[64];
	snprintf(text, 64, "Voxels  %d x %d", gw, gh);
	imguiValue(text);
	
	imguiSeparator();
	imguiLabel("Agent");
	imguiSlider("Height", &m_agentHeight, 0.1f, 5.0f, 0.1f);
	imguiSlider("Radius", &m_agentRadius, 0.0f, 5.0f, 0.1f);
	imguiSlider("Max Climb", &m_agentMaxClimb, 0.1f, 5.0f, 0.1f);
	imguiSlider("Max Slope", &m_agentMaxSlope, 0.0f, 90.0f, 1.0f);
	
	imguiSeparator();
	imguiLabel("Region");
	imguiSlider("Min Region Size", &m_regionMinSize, 0.0f, 150.0f, 1.0f);
	imguiSlider("Merged Region Size", &m_regionMergeSize, 0.0f, 150.0f, 1.0f);
	
	imguiSeparator();
	imguiLabel("Polygonization");
	imguiSlider("Max Edge Length", &m_edgeMaxLen, 0.0f, 50.0f, 1.0f);
	imguiSlider("Max Edge Error", &m_edgeMaxError, 0.1f, 3.0f, 0.1f);
	imguiSlider("Verts Per Poly", &m_vertsPerPoly, 3.0f, 12.0f, 1.0f);		

	imguiSeparator();
}

void Sample::setToolStartPos(const float* p)
{
}

void Sample::setToolEndPos(const float* p)
{
}

bool Sample::handleBuild()
{
	return true;
}
