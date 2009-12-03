#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "Sample.h"
#include "Sample_SoloMeshTiled.h"
#include "Recast.h"
#include "RecastTimer.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

Sample_SoloMeshTiled::Sample_SoloMeshTiled() :
	m_measurePerTileTimings(false),
	m_keepInterResults(false),
	m_tileSize(64),
	m_chunkyMesh(0),
	m_pmesh(0),
	m_dmesh(0),
	m_tileSet(0),
	m_statPolysPerTileSamples(0),
	m_statTimePerTileSamples(0),
	m_drawMode(DRAWMODE_NAVMESH)
{
}

Sample_SoloMeshTiled::~Sample_SoloMeshTiled()
{
	cleanup();
}

void Sample_SoloMeshTiled::cleanup()
{
	delete m_chunkyMesh;
	m_chunkyMesh = 0;
	delete m_tileSet;
	m_tileSet = 0;
	delete m_pmesh;
	m_pmesh = 0;
	delete m_dmesh;
	m_dmesh = 0;
	toolCleanup();
	m_statTimePerTileSamples = 0;
	m_statPolysPerTileSamples = 0;
}

void Sample_SoloMeshTiled::handleSettings()
{
	Sample::handleCommonSettings();
	
	imguiLabel("Tiling");
	imguiSlider("TileSize", &m_tileSize, 16.0f, 1024.0f, 16.0f);

	char text[64];
	int gw = 0, gh = 0;
	rcCalcGridSize(m_bmin, m_bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;
	snprintf(text, 64, "Tiles  %d x %d", tw, th);
	imguiValue(text);
	
	imguiSeparator();
	if (imguiCheck("Keep Itermediate Results", m_keepInterResults))
		m_keepInterResults = !m_keepInterResults;
	if (imguiCheck("Measure Per Tile Timings", m_measurePerTileTimings))
		m_measurePerTileTimings = !m_measurePerTileTimings;
	
	imguiSeparator();
}

void Sample_SoloMeshTiled::handleDebugMode()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		valid[i] = false;

	bool hasChf = false;
	bool hasSolid = false;
	bool hasCset = false;
	bool hasPmesh = false;
	bool hasDmesh = false;
	if (m_tileSet)
	{
		for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
		{
			if (m_tileSet->tiles[i].solid) hasSolid = true;
			if (m_tileSet->tiles[i].chf) hasChf = true;
			if (m_tileSet->tiles[i].cset) hasCset = true;
			if (m_tileSet->tiles[i].pmesh) hasPmesh = true;
			if (m_tileSet->tiles[i].dmesh) hasDmesh = true;
		}
	}
	if (m_pmesh) hasPmesh = true;
	if (m_dmesh) hasDmesh = true;
	
	if (m_verts && m_tris)
	{
		valid[DRAWMODE_NAVMESH] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_TRANS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_BVTREE] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_INVIS] = m_navMesh != 0;
		valid[DRAWMODE_MESH] = true;
		valid[DRAWMODE_VOXELS] = hasSolid;
		valid[DRAWMODE_VOXELS_WALKABLE] = hasSolid;
		valid[DRAWMODE_COMPACT] = hasChf;
		valid[DRAWMODE_COMPACT_DISTANCE] = hasChf;
		valid[DRAWMODE_COMPACT_REGIONS] = hasChf;
		valid[DRAWMODE_REGION_CONNECTIONS] = hasCset;
		valid[DRAWMODE_RAW_CONTOURS] = hasCset;
		valid[DRAWMODE_BOTH_CONTOURS] = hasCset;
		valid[DRAWMODE_CONTOURS] = hasCset;
		valid[DRAWMODE_POLYMESH] = hasPmesh;
		valid[DRAWMODE_POLYMESH_DETAIL] = hasDmesh;
	}
	
	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		if (!valid[i]) unavail++;
	
	if (unavail == MAX_DRAWMODE)
		return;
	
	imguiLabel("Draw");
	if (imguiCheck("Input Mesh", m_drawMode == DRAWMODE_MESH, valid[DRAWMODE_MESH]))
		m_drawMode = DRAWMODE_MESH;
	if (imguiCheck("Navmesh", m_drawMode == DRAWMODE_NAVMESH, valid[DRAWMODE_NAVMESH]))
		m_drawMode = DRAWMODE_NAVMESH;
	if (imguiCheck("Navmesh Invis", m_drawMode == DRAWMODE_NAVMESH_INVIS, valid[DRAWMODE_NAVMESH_INVIS]))
		m_drawMode = DRAWMODE_NAVMESH_INVIS;
	if (imguiCheck("Navmesh Trans", m_drawMode == DRAWMODE_NAVMESH_TRANS, valid[DRAWMODE_NAVMESH_TRANS]))
		m_drawMode = DRAWMODE_NAVMESH_TRANS;
	if (imguiCheck("Navmesh BVTree", m_drawMode == DRAWMODE_NAVMESH_BVTREE, valid[DRAWMODE_NAVMESH_BVTREE]))
		m_drawMode = DRAWMODE_NAVMESH_BVTREE;
	if (imguiCheck("Voxels", m_drawMode == DRAWMODE_VOXELS, valid[DRAWMODE_VOXELS]))
		m_drawMode = DRAWMODE_VOXELS;
	if (imguiCheck("Walkable Voxels", m_drawMode == DRAWMODE_VOXELS_WALKABLE, valid[DRAWMODE_VOXELS_WALKABLE]))
		m_drawMode = DRAWMODE_VOXELS_WALKABLE;
	if (imguiCheck("Compact", m_drawMode == DRAWMODE_COMPACT, valid[DRAWMODE_COMPACT]))
		m_drawMode = DRAWMODE_COMPACT;
	if (imguiCheck("Compact Distance", m_drawMode == DRAWMODE_COMPACT_DISTANCE, valid[DRAWMODE_COMPACT_DISTANCE]))
		m_drawMode = DRAWMODE_COMPACT_DISTANCE;
	if (imguiCheck("Compact Regions", m_drawMode == DRAWMODE_COMPACT_REGIONS, valid[DRAWMODE_COMPACT_REGIONS]))
		m_drawMode = DRAWMODE_COMPACT_REGIONS;
	if (imguiCheck("Region Connections", m_drawMode == DRAWMODE_REGION_CONNECTIONS, valid[DRAWMODE_REGION_CONNECTIONS]))
		m_drawMode = DRAWMODE_REGION_CONNECTIONS;
	if (imguiCheck("Raw Contours", m_drawMode == DRAWMODE_RAW_CONTOURS, valid[DRAWMODE_RAW_CONTOURS]))
		m_drawMode = DRAWMODE_RAW_CONTOURS;
	if (imguiCheck("Both Contours", m_drawMode == DRAWMODE_BOTH_CONTOURS, valid[DRAWMODE_BOTH_CONTOURS]))
		m_drawMode = DRAWMODE_BOTH_CONTOURS;
	if (imguiCheck("Contours", m_drawMode == DRAWMODE_CONTOURS, valid[DRAWMODE_CONTOURS]))
		m_drawMode = DRAWMODE_CONTOURS;
	if (imguiCheck("Poly Mesh", m_drawMode == DRAWMODE_POLYMESH, valid[DRAWMODE_POLYMESH]))
		m_drawMode = DRAWMODE_POLYMESH;
	if (imguiCheck("Poly Mesh Detail", m_drawMode == DRAWMODE_POLYMESH_DETAIL, valid[DRAWMODE_POLYMESH_DETAIL]))
		m_drawMode = DRAWMODE_POLYMESH_DETAIL;
	
	if (unavail)
	{
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("to see more debug mode options.");
	}
}

void Sample_SoloMeshTiled::handleRender()
{
	if (!m_verts || !m_tris || !m_trinorms)
		return;
	
	float col[4];
	
	DebugDrawGL dd;
	
	glEnable(GL_FOG);
	glDepthMask(GL_TRUE);
	
	if (m_drawMode == DRAWMODE_MESH)
	{
		// Draw mesh
		rcDebugDrawMeshSlope(&dd, m_verts, m_nverts, m_tris, m_trinorms, m_ntris, m_agentMaxSlope);
	}
	else if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		rcDebugDrawMesh(&dd, m_verts, m_nverts, m_tris, m_trinorms, m_ntris, 0);
	}
	
	glDisable(GL_FOG);
	glDepthMask(GL_FALSE);
	
	// Draw bounds
	col[0] = 1; col[1] = 1; col[2] = 1; col[3] = 0.5f;
	rcDebugDrawBoxWire(&dd, m_bmin[0],m_bmin[1],m_bmin[2], m_bmax[0],m_bmax[1],m_bmax[2], col);
	
	// Tiling grid.
	const int ts = (int)m_tileSize;
	int gw = 0, gh = 0;
	rcCalcGridSize(m_bmin, m_bmax, m_cellSize, &gw, &gh);
	int tw = (gw + ts-1) / ts;
	int th = (gh + ts-1) / ts;
	const float s = ts*m_cellSize;
	glBegin(GL_LINES);
	glColor4ub(0,0,0,64);
	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			float fx, fy, fz;
			fx = m_bmin[0] + x*s;
			fy = m_bmin[1];
			fz = m_bmin[2] + y*s;
			
			glVertex3f(fx,fy,fz);
			glVertex3f(fx+s,fy,fz);
			glVertex3f(fx,fy,fz);
			glVertex3f(fx,fy,fz+s);
			
			if (x+1 >= tw)
			{
				glVertex3f(fx+s,fy,fz);
				glVertex3f(fx+s,fy,fz+s);
			}
			if (y+1 >= th)
			{
				glVertex3f(fx,fy,fz+s);
				glVertex3f(fx+s,fy,fz+s);
			}
		}
	}
	glEnd();
	
	
	if (m_navMesh &&
		(m_drawMode == DRAWMODE_NAVMESH ||
		 m_drawMode == DRAWMODE_NAVMESH_TRANS ||
		 m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
		 m_drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		int flags = NAVMESH_TOOLS;
		if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
			flags |= NAVMESH_POLYS;
		if (m_drawMode == DRAWMODE_NAVMESH_BVTREE)
			flags |= NAVMESH_BVTREE;
		toolRender(flags);
	}
	
	glDepthMask(GL_TRUE);
	
	if (m_tileSet)
	{
		if (m_drawMode == DRAWMODE_COMPACT)
		{
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].chf)
					rcDebugDrawCompactHeightfieldSolid(&dd, *m_tileSet->tiles[i].chf);
			}
		}
		
		if (m_drawMode == DRAWMODE_COMPACT_DISTANCE)
		{
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].chf)
					rcDebugDrawCompactHeightfieldDistance(&dd, *m_tileSet->tiles[i].chf);
			}
		}
		if (m_drawMode == DRAWMODE_COMPACT_REGIONS)
		{
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].chf)
					rcDebugDrawCompactHeightfieldRegions(&dd, *m_tileSet->tiles[i].chf);
			}
		}
			
		if (m_drawMode == DRAWMODE_VOXELS)
		{
			glEnable(GL_FOG);
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].solid)
					rcDebugDrawHeightfieldSolid(&dd, *m_tileSet->tiles[i].solid);
			}
			glDisable(GL_FOG);
		}
		if (m_drawMode == DRAWMODE_VOXELS_WALKABLE)
		{
			glEnable(GL_FOG);
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].solid)
					rcDebugDrawHeightfieldWalkable(&dd, *m_tileSet->tiles[i].solid);
			}
			glDisable(GL_FOG);
		}
		if (m_drawMode == DRAWMODE_RAW_CONTOURS)
		{
			glDepthMask(GL_FALSE);
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].cset)
					rcDebugDrawRawContours(&dd, *m_tileSet->tiles[i].cset);
			}
			glDepthMask(GL_TRUE);
		}
		if (m_drawMode == DRAWMODE_BOTH_CONTOURS)
		{
			glDepthMask(GL_FALSE);
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].cset)
				{
					rcDebugDrawRawContours(&dd, *m_tileSet->tiles[i].cset, 0.5f);
					rcDebugDrawContours(&dd, *m_tileSet->tiles[i].cset);
				}
			}
			glDepthMask(GL_TRUE);
		}
		if (m_drawMode == DRAWMODE_CONTOURS)
		{
			glDepthMask(GL_FALSE);
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].cset)
					rcDebugDrawContours(&dd, *m_tileSet->tiles[i].cset);
			}
			glDepthMask(GL_TRUE);
		}
		if (m_drawMode == DRAWMODE_REGION_CONNECTIONS)
		{
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].chf)
					rcDebugDrawCompactHeightfieldRegions(&dd, *m_tileSet->tiles[i].chf);
			}
			
			glDepthMask(GL_FALSE);
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].cset)
					rcDebugDrawRegionConnections(&dd, *m_tileSet->tiles[i].cset);
			}
			glDepthMask(GL_TRUE);
		}
		if (/*m_pmesh &&*/ m_drawMode == DRAWMODE_POLYMESH)
		{
			glDepthMask(GL_FALSE);
			if (m_pmesh)
			{
				rcDebugDrawPolyMesh(&dd, *m_pmesh);
			}
			else
			{
				for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
				{
					if (m_tileSet->tiles[i].pmesh)
						rcDebugDrawPolyMesh(&dd, *m_tileSet->tiles[i].pmesh);
				}
			}
			
			glDepthMask(GL_TRUE);
		}
		if (/*m_dmesh &&*/ m_drawMode == DRAWMODE_POLYMESH_DETAIL)
		{
			glDepthMask(GL_FALSE);
			if (m_dmesh)
			{
				rcDebugDrawPolyMeshDetail(&dd, *m_dmesh);
			}
			else
			{
				for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
				{
					if (m_tileSet->tiles[i].dmesh)
						rcDebugDrawPolyMeshDetail(&dd, *m_tileSet->tiles[i].dmesh);
				}
			}
			glDepthMask(GL_TRUE);
		}
	}
		
	static const float startCol[4] = { 0.5f, 0.1f, 0.0f, 0.75f };
	static const float endCol[4] = { 0.2f, 0.4f, 0.0f, 0.75f };
	if (m_sposSet)
		drawAgent(m_spos, m_agentRadius, m_agentHeight, m_agentMaxClimb, startCol);
	if (m_eposSet)
		drawAgent(m_epos, m_agentRadius, m_agentHeight, m_agentMaxClimb, endCol);
	
}

static float nicenum(float x, int round)
{
	float expv = floorf(log10f(x));
	float f = x / powf(10.0f, expv);
	float nf;
	if (round)
	{
		if (f < 1.5f) nf = 1.0f;
		else if (f < 3.0f) nf = 2.0f;
		else if (f < 7.0f) nf = 5.0f;
		else nf = 10.0f;
	}
	else
	{
		if (f <= 1.0f) nf = 1.0f;
		else if (f <= 2.0f) nf = 2.0f;
		else if (f <= 5.0f) nf = 5.0f;
		else nf = 10.0f;
	}
	return nf*powf(10.0f, expv);
}

static void drawLabels(int x, int y, int w, int h,
					   int nticks, float vmin, float vmax, const char* unit)
{
	char str[8], temp[32];
	
	float range = nicenum(vmax-vmin, 0);
	float d = nicenum(range/(float)(nticks-1), 1);
	float graphmin = floorf(vmin/d)*d;
	float graphmax = ceilf(vmax/d)*d;
	int nfrac = (int)-floorf(log10f(d));
	if (nfrac < 0) nfrac = 0;
	snprintf(str, 6, "%%.%df %%s", nfrac);

	for (float v = graphmin; v < graphmax+d/2; v += d)
	{
		float lx = x + (v-vmin)/(vmax-vmin)*w;
		if (lx < 0 || lx > w) continue;
		snprintf(temp, 20, str, v, unit);
		imguiDrawText((int)lx+2, (int)y+2, IMGUI_ALIGN_LEFT, temp, imguiRGBA(255,255,255));
		glColor4ub(0,0,0,64);
		glBegin(GL_LINES);
		glVertex2f(lx,(float)y);
		glVertex2f(lx,(float)(y+h));
		glEnd();
	}
}

static void drawGraph(const char* name, int x, int y, int w, int h, float sd,
					  const int* samples, int n, int nsamples, const char* unit)
{
	char text[64];
	int first, last, maxval;
	first = 0;
	last = n-1;
	while (first < n && samples[first] == 0)
		first++;
	while (last >= 0 && samples[last] == 0)
		last--;
	if (first == last)
		return;
	maxval = 1;
	for (int i = first; i <= last; ++i)
	{
		if (samples[i] > maxval)
			maxval = samples[i];
	}
	const float sx = (float)w / (float)(last-first);
	const float sy = (float)h / (float)maxval;
	
	glBegin(GL_QUADS);
	glColor4ub(32,32,32,64);
	glVertex2i(x,y);
	glVertex2i(x+w,y);
	glVertex2i(x+w,y+h);
	glVertex2i(x,y+h);
	glEnd();
	
	glColor4ub(255,255,255,64);
	glBegin(GL_LINES);
	for (int i = 0; i <= 4; ++i)
	{
		int yy = y+i*h/4;
		glVertex2i(x,yy);
		glVertex2i(x+w,yy);
	}
	glEnd();

	glColor4ub(0,196,255,255);
	glBegin(GL_LINE_STRIP);
	for (int i = first; i <= last; ++i)
	{
		float fx = x + (i-first)*sx;
		float fy = y + samples[i]*sy;
		glVertex2f(fx,fy);
	}
	glEnd();

	snprintf(text,64,"%d", maxval);
	imguiDrawText((int)x+w-2, (int)y+h-20, IMGUI_ALIGN_RIGHT, text, imguiRGBA(0,0,0));
	imguiDrawText((int)x+2, (int)y+h-20, IMGUI_ALIGN_LEFT, name, imguiRGBA(255,255,255));
	
	drawLabels(x, y, w, h, 10, first*sd, last*sd, unit);
}

void Sample_SoloMeshTiled::handleRenderOverlay(double* proj, double* model, int* view)
{
	toolRenderOverlay(proj, model, view);

	if (m_measurePerTileTimings)
	{
		if (m_statTimePerTileSamples)
			drawGraph("Build Time/Tile", 10, 10, 500, 100, 1.0f, m_statTimePerTile, MAX_STAT_BUCKETS, m_statTimePerTileSamples, "ms");

		if (m_statPolysPerTileSamples)
			drawGraph("Polygons/Tile", 10, 120, 500, 100, 1.0f, m_statPolysPerTile, MAX_STAT_BUCKETS, m_statPolysPerTileSamples, "");

		int validTiles = 0;
		if (m_tileSet)
		{
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].buildTime > 0)
					validTiles++;
			}
		}
		char text[64];
		snprintf(text,64,"Tiles %d\n", validTiles);
		imguiDrawText(10, 240, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255));
	}
}

void Sample_SoloMeshTiled::handleMeshChanged(const float* verts, int nverts,
											  const int* tris, const float* trinorms, int ntris,
											  const float* bmin, const float* bmax)
{
	Sample::handleMeshChanged(verts, nverts, tris, trinorms, ntris, bmin, bmax);
	toolCleanup();
	toolReset();
	m_statTimePerTileSamples = 0;
	m_statPolysPerTileSamples = 0;
}

bool Sample_SoloMeshTiled::handleBuild()
{
	if (!m_verts || ! m_tris)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}

	if (m_measurePerTileTimings)
	{
		memset(m_statPolysPerTile, 0, sizeof(m_statPolysPerTile));
		memset(m_statTimePerTile, 0, sizeof(m_statTimePerTile));
		m_statPolysPerTileSamples = 0;
		m_statTimePerTileSamples = 0;
	}
	
	cleanup();
	toolCleanup();
	toolReset();
	
	// Init build configuration from GUI
	memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.cs = m_cellSize;
	m_cfg.ch = m_cellHeight;
	m_cfg.walkableSlopeAngle = m_agentMaxSlope;
	m_cfg.walkableHeight = (int)ceilf(m_agentHeight / m_cfg.ch);
	m_cfg.walkableClimb = (int)ceilf(m_agentMaxClimb / m_cfg.ch);
	m_cfg.walkableRadius = (int)ceilf(m_agentRadius / m_cfg.cs);
	m_cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	m_cfg.maxSimplificationError = m_edgeMaxError;
	m_cfg.minRegionSize = (int)rcSqr(m_regionMinSize);
	m_cfg.mergeRegionSize = (int)rcSqr(m_regionMergeSize);
	m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	m_cfg.tileSize = (int)m_tileSize;
	m_cfg.borderSize = m_cfg.walkableRadius + 3; // Reserve enough padding.
	m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
		
	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by an user defined box, etc.
	vcopy(m_cfg.bmin, m_bmin);
	vcopy(m_cfg.bmax, m_bmax);
	rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);
	
	// Reset build times gathering.
	memset(&m_buildTimes, 0, sizeof(m_buildTimes));
	rcSetBuildTimes(&m_buildTimes);
	
	// Start the build process.	
	rcTimeVal totStartTime = rcGetPerformanceTimer();

	// Calculate the number of tiles in the output and initialize tiles.
	m_tileSet = new TileSet;
	if (!m_tileSet)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'tileSet'.");
		return false;
	}
	vcopy(m_tileSet->bmin, m_cfg.bmin);
	vcopy(m_tileSet->bmax, m_cfg.bmax);
	m_tileSet->cs = m_cfg.cs;
	m_tileSet->ch = m_cfg.ch;
	m_tileSet->width = (m_cfg.width + m_cfg.tileSize-1) / m_cfg.tileSize;
	m_tileSet->height = (m_cfg.height + m_cfg.tileSize-1) / m_cfg.tileSize;
	m_tileSet->tiles = new Tile[m_tileSet->height * m_tileSet->width];
	if (!m_tileSet->tiles)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'tileSet->tiles' (%d).", m_tileSet->height * m_tileSet->width);
		return false;
	}

	// Build chunky trimesh for local polygon queries.
	rcTimeVal chunkyStartTime = rcGetPerformanceTimer();
	m_chunkyMesh = new rcChunkyTriMesh;
	if (!m_chunkyMesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
		return false;
	}
	if (!rcCreateChunkyTriMesh(m_verts, m_tris, m_ntris, 256, m_chunkyMesh))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Could not build chunky mesh.");
		return false;
	}
	rcTimeVal chunkyEndTime = rcGetPerformanceTimer();
	
	if (rcGetLog())
	{
		rcGetLog()->log(RC_LOG_PROGRESS, "Building navigation:");
		rcGetLog()->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);
		rcGetLog()->log(RC_LOG_PROGRESS, " - %d x %d tiles", m_tileSet->width, m_tileSet->height);
		rcGetLog()->log(RC_LOG_PROGRESS, " - %.1f verts, %.1f tris", m_nverts/1000.0f, m_ntris/1000.0f);
	}
		
	// Initialize per tile config.
	rcConfig tileCfg;
	memcpy(&tileCfg, &m_cfg, sizeof(rcConfig));
	tileCfg.width = m_cfg.tileSize + m_cfg.borderSize*2;
	tileCfg.height = m_cfg.tileSize + m_cfg.borderSize*2;
		
	// Allocate array that can hold triangle flags for all geom chunks.
	unsigned char* triangleFlags = new unsigned char[m_chunkyMesh->maxTrisPerChunk];
	if (!triangleFlags)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'triangleFlags' (%d).", m_chunkyMesh->maxTrisPerChunk);
		return false;
	}
		
	rcHeightfield* solid = 0;
	rcCompactHeightfield* chf = 0;
	rcContourSet* cset = 0;
		
	for (int y = 0; y < m_tileSet->height; ++y)
	{
		for (int x = 0; x < m_tileSet->width; ++x)
		{
			rcTimeVal startTime = rcGetPerformanceTimer();
			
			Tile& tile = m_tileSet->tiles[x + y*m_tileSet->width]; 
			
			// Calculate the per tile bounding box.
			tileCfg.bmin[0] = m_cfg.bmin[0] + (x*m_cfg.tileSize - m_cfg.borderSize)*m_cfg.cs;
			tileCfg.bmin[2] = m_cfg.bmin[2] + (y*m_cfg.tileSize - m_cfg.borderSize)*m_cfg.cs;
			tileCfg.bmax[0] = m_cfg.bmin[0] + ((x+1)*m_cfg.tileSize + m_cfg.borderSize)*m_cfg.cs;
			tileCfg.bmax[2] = m_cfg.bmin[2] + ((y+1)*m_cfg.tileSize + m_cfg.borderSize)*m_cfg.cs;
			
			delete solid;
			delete chf;
			solid = 0;
			chf = 0;
			
			float tbmin[2], tbmax[2];
			tbmin[0] = tileCfg.bmin[0];
			tbmin[1] = tileCfg.bmin[2];
			tbmax[0] = tileCfg.bmax[0];
			tbmax[1] = tileCfg.bmax[2];
			int cid[256];// TODO: Make grow when returning too many items.
			const int ncid = rcGetChunksInRect(m_chunkyMesh, tbmin, tbmax, cid, 256);
			if (!ncid)
				continue;
			
			solid = new rcHeightfield;
			if (!solid)
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Out of memory 'solid'.", x, y);
				continue;
			}
			if (!rcCreateHeightfield(*solid, tileCfg.width, tileCfg.height, tileCfg.bmin, tileCfg.bmax, tileCfg.cs, tileCfg.ch))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not create solid heightfield.", x, y);
				continue;
			}
			
			for (int i = 0; i < ncid; ++i)
			{
				const rcChunkyTriMeshNode& node = m_chunkyMesh->nodes[cid[i]];
				const int* tris = &m_chunkyMesh->tris[node.i*3];
				const int ntris = node.n;
				
				memset(triangleFlags, 0, ntris*sizeof(unsigned char));
				rcMarkWalkableTriangles(tileCfg.walkableSlopeAngle,
										m_verts, m_nverts, tris, ntris, triangleFlags);
				
				rcRasterizeTriangles(m_verts, m_nverts, tris, triangleFlags, ntris, *solid);
			}	
			
			rcFilterLedgeSpans(tileCfg.walkableHeight, tileCfg.walkableClimb, *solid);
			
			rcFilterWalkableLowHeightSpans(tileCfg.walkableHeight, *solid);
			
			chf = new rcCompactHeightfield;
			if (!chf)
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Out of memory 'chf'.", x, y);
				continue;
			}
			if (!rcBuildCompactHeightfield(tileCfg.walkableHeight, tileCfg.walkableClimb,
										   RC_WALKABLE, *solid, *chf))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not build compact data.", x, y);
				continue;
			}
			
			if (!rcBuildDistanceField(*chf))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not build distance fields.", x, y);
				continue;
			}
			
			if (!rcBuildRegions(*chf, tileCfg.walkableRadius, tileCfg.borderSize, tileCfg.minRegionSize, tileCfg.mergeRegionSize))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not build regions.", x, y);
				continue;
			}
			
			cset = new rcContourSet;
			if (!cset)
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Out of memory 'cset'.", x, y);
				continue;
			}
			if (!rcBuildContours(*chf, tileCfg.maxSimplificationError, tileCfg.maxEdgeLen, *cset))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not create contours.", x, y);
				continue;
			}
			
			if (!cset->nconts)
			{
				delete cset;
				cset = 0;
				continue;
			}
			
			tile.pmesh = new rcPolyMesh;
			if (!tile.pmesh)
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Out of memory 'pmesh'.", x, y);
				continue;
			}
			if (!rcBuildPolyMesh(*cset, tileCfg.maxVertsPerPoly, *tile.pmesh))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not create poly mesh.", x, y);
				continue;
			}

			tile.dmesh = new rcPolyMeshDetail;
			if (!tile.dmesh)
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Out of memory 'dmesh'.", x, y);
				continue;
			}
			
			if (!rcBuildPolyMeshDetail(*tile.pmesh, *chf, tileCfg.detailSampleDist, tileCfg	.detailSampleMaxError, *tile.dmesh))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not build detail mesh.", x, y);
				continue;
			}

			if (m_keepInterResults)
			{
				tile.solid = solid;
				solid = 0;
				tile.chf = chf;
				chf = 0;
				tile.cset = cset;
				cset = 0;
			}
			
			rcTimeVal endTime = rcGetPerformanceTimer();
			tile.buildTime += rcGetDeltaTimeUsec(startTime, endTime);
			
			// Some extra code to measure some per tile statistics,
			// such as build time and how many polygons there are per tile.
			if (tile.pmesh)
			{
				int bucket = tile.pmesh->npolys;
				if (bucket < 0) bucket = 0;
				if (bucket >= MAX_STAT_BUCKETS) bucket = MAX_STAT_BUCKETS-1;
				m_statPolysPerTile[bucket]++;
				m_statPolysPerTileSamples++;
			}
			int bucket = (tile.buildTime+500)/1000;
			if (bucket < 0) bucket = 0;
			if (bucket >= MAX_STAT_BUCKETS) bucket = MAX_STAT_BUCKETS-1;
			m_statTimePerTile[bucket]++;
			m_statTimePerTileSamples++;
		}
	}
	
	delete [] triangleFlags;
	delete solid;
	delete chf;
	
	// Merge per tile poly and detail meshes.
	rcPolyMesh** pmmerge = new rcPolyMesh*[m_tileSet->width*m_tileSet->height];
	if (!pmmerge)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'pmmerge' (%d).", m_tileSet->width*m_tileSet->height);
		return false;
	}
	
	rcPolyMeshDetail** dmmerge = new rcPolyMeshDetail*[m_tileSet->width*m_tileSet->height];
	if (!dmmerge)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'dmmerge' (%d).", m_tileSet->width*m_tileSet->height);
		return false;
	}
	
	int nmerge = 0;
	for (int y = 0; y < m_tileSet->height; ++y)
	{
		for (int x = 0; x < m_tileSet->width; ++x)
		{
			Tile& tile = m_tileSet->tiles[x + y*m_tileSet->width]; 
			if (tile.pmesh)
			{
				pmmerge[nmerge] = tile.pmesh;
				dmmerge[nmerge] = tile.dmesh;
				nmerge++;
			}
		}
	}
	
	m_pmesh = new rcPolyMesh;
	if (!m_pmesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	rcMergePolyMeshes(pmmerge, nmerge, *m_pmesh);

	m_dmesh = new rcPolyMeshDetail;
	if (!m_dmesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'dmesh'.");
		return false;
	}
	rcMergePolyMeshDetails(dmmerge, nmerge, *m_dmesh);
	
	delete [] pmmerge;
	delete [] dmmerge;
	
	if (!m_keepInterResults)
	{
		for (int y = 0; y < m_tileSet->height; ++y)
		{
			for (int x = 0; x < m_tileSet->width; ++x)
			{
				Tile& tile = m_tileSet->tiles[x + y*m_tileSet->width]; 
				delete tile.cset;
				tile.cset = 0;
				delete tile.pmesh;
				tile.pmesh = 0;
				delete tile.dmesh;
				tile.dmesh = 0;
			}
		}
	}
	
	if (m_pmesh && m_cfg.maxVertsPerPoly <= DT_STAT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;
		if (!dtCreateNavMeshData(m_pmesh->verts, m_pmesh->nverts,
								 m_pmesh->polys, m_pmesh->npolys, m_pmesh->nvp,
								 m_dmesh->meshes, m_dmesh->verts, m_dmesh->nverts,
								 m_dmesh->tris, m_dmesh->ntris, 
								 m_pmesh->bmin, m_pmesh->bmax, m_cfg.cs, m_cfg.ch, 0, m_cfg.walkableClimb,
								 &navData, &navDataSize))
		{
			if (rcGetLog())
				rcGetLog()->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}
		
		m_navMesh = new dtNavMesh;
		if (!m_navMesh)
		{
			delete [] navData;
			if (rcGetLog())
				rcGetLog()->log(RC_LOG_ERROR, "Could not create Detour navmesh");
			return false;
		}
		
		if (!m_navMesh->init(navData, navDataSize, true, 2048))
		{
			delete [] navData;
			if (rcGetLog())
				rcGetLog()->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return false;
		}
	}
		
	rcTimeVal totEndTime = rcGetPerformanceTimer();
	
	if (rcGetLog())
	{
		const float pc = 100.0f / rcGetDeltaTimeUsec(totStartTime, totEndTime);

		rcGetLog()->log(RC_LOG_PROGRESS, "Chunky Mesh: %.1fms (%.1f%%)", rcGetDeltaTimeUsec(chunkyStartTime, chunkyEndTime)/1000.0f, rcGetDeltaTimeUsec(chunkyStartTime, chunkyEndTime)*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Rasterize: %.1fms (%.1f%%)", m_buildTimes.rasterizeTriangles/1000.0f, m_buildTimes.rasterizeTriangles*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Compact: %.1fms (%.1f%%)", m_buildTimes.buildCompact/1000.0f, m_buildTimes.buildCompact*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Border: %.1fms (%.1f%%)", m_buildTimes.filterBorder/1000.0f, m_buildTimes.filterBorder*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Walkable: %.1fms (%.1f%%)", m_buildTimes.filterWalkable/1000.0f, m_buildTimes.filterWalkable*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Reachable: %.1fms (%.1f%%)", m_buildTimes.filterMarkReachable/1000.0f, m_buildTimes.filterMarkReachable*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Distancefield: %.1fms (%.1f%%)", m_buildTimes.buildDistanceField/1000.0f, m_buildTimes.buildDistanceField*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - distance: %.1fms (%.1f%%)", m_buildTimes.buildDistanceFieldDist/1000.0f, m_buildTimes.buildDistanceFieldDist*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - blur: %.1fms (%.1f%%)", m_buildTimes.buildDistanceFieldBlur/1000.0f, m_buildTimes.buildDistanceFieldBlur*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Regions: %.1fms (%.1f%%)", m_buildTimes.buildRegions/1000.0f, m_buildTimes.buildRegions*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - watershed: %.1fms (%.1f%%)", m_buildTimes.buildRegionsReg/1000.0f, m_buildTimes.buildRegionsReg*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "    - expand: %.1fms (%.1f%%)", m_buildTimes.buildRegionsExp/1000.0f, m_buildTimes.buildRegionsExp*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "    - find catchment basins: %.1fms (%.1f%%)", m_buildTimes.buildRegionsFlood/1000.0f, m_buildTimes.buildRegionsFlood*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - filter: %.1fms (%.1f%%)", m_buildTimes.buildRegionsFilter/1000.0f, m_buildTimes.buildRegionsFilter*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Contours: %.1fms (%.1f%%)", m_buildTimes.buildContours/1000.0f, m_buildTimes.buildContours*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - trace: %.1fms (%.1f%%)", m_buildTimes.buildContoursTrace/1000.0f, m_buildTimes.buildContoursTrace*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - simplify: %.1fms (%.1f%%)", m_buildTimes.buildContoursSimplify/1000.0f, m_buildTimes.buildContoursSimplify*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Polymesh: %.1fms (%.1f%%)", m_buildTimes.buildPolymesh/1000.0f, m_buildTimes.buildPolymesh*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Polymesh Detail: %.1fms (%.1f%%)", m_buildTimes.buildDetailMesh/1000.0f, m_buildTimes.buildDetailMesh*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "Merge Polymeshes: %.1fms (%.1f%%)", m_buildTimes.mergePolyMesh/1000.0f, m_buildTimes.mergePolyMesh*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "Merge Polymesh Details: %.1fms (%.1f%%)", m_buildTimes.mergePolyMeshDetail/1000.0f, m_buildTimes.mergePolyMeshDetail*pc);
		
		if (m_pmesh)
			rcGetLog()->log(RC_LOG_PROGRESS, "Polymesh: Verts:%d  Polys:%d", m_pmesh->nverts, m_pmesh->npolys);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "TOTAL: %.1fms", rcGetDeltaTimeUsec(totStartTime, totEndTime)/1000.0f);
	}

	toolRecalc();

	return true;
}
