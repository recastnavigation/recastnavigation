#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "imgui.h"
#include "SDL.h"
#include "SDL_opengl.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


void DebugDrawGL::begin(duDebugDrawPrimitives prim, float size)
{
	switch (prim)
	{
		case DU_DRAW_POINTS:
			glPointSize(size);
			glBegin(GL_POINTS);
			break;
		case DU_DRAW_LINES:
			glLineWidth(size);
			glBegin(GL_LINES);
			break;
		case DU_DRAW_TRIS:
			glBegin(GL_TRIANGLES);
			break;
		case DU_DRAW_QUADS:
			glBegin(GL_QUADS);
			break;
	};
}
	
void DebugDrawGL::vertex(const float* pos, unsigned int color)
{
	glColor4ubv((GLubyte*)&color);
	glVertex3fv(pos);
}
	
void DebugDrawGL::vertex(const float x, const float y, const float z, unsigned int color)
{
	glColor4ubv((GLubyte*)&color);
	glVertex3f(x,y,z);
}
	
void DebugDrawGL::end()
{
	glEnd();
	glLineWidth(1.0f);
	glPointSize(1.0f);
}


Sample::Sample() :
	m_verts(0),
	m_nverts(0),
	m_tris(0),
	m_trinorms(0),
	m_ntris(0),
	m_tool(0)
{
	resetCommonSettings();
}

Sample::~Sample()
{
	delete m_tool;
}

void Sample::setTool(SampleTool* tool)
{
	delete m_tool;
	m_tool = tool;
	if (tool)
		m_tool->init(this);
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
	
	DebugDrawGL dd;
		
	// Draw mesh
	duDebugDrawTriMesh(&dd, m_verts, m_nverts, m_tris, m_trinorms, m_ntris, 0);
	// Draw bounds
	float col[4] = {1,1,1,0.5f};
	duDebugDrawBoxWire(&dd, m_bmin[0],m_bmin[1],m_bmin[2], m_bmax[0],m_bmax[1],m_bmax[2], col);
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
	m_detailSampleDist = 6.0f;
	m_detailSampleMaxError = 1.0f;
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
	imguiLabel("Detail Mesh");
	imguiSlider("Sample Distance", &m_detailSampleDist, 0.0f, 16.0f, 1.0f);
	imguiSlider("Max Sample Error", &m_detailSampleMaxError, 0.0f, 16.0f, 1.0f);
	
	imguiSeparator();
}

void Sample::handleClick(const float* p, bool shift)
{
	if (m_tool)
		m_tool->handleClick(p, shift);
}

bool Sample::handleBuild()
{
	return true;
}
