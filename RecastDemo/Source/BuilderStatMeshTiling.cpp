#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "SDL.h"
#include "SDL_Opengl.h"
#include "imgui.h"
#include "glfont.h"
#include "Builder.h"
#include "BuilderStatMeshTiling.h"
#include "Recast.h"
#include "RecastTimer.h"
#include "RecastDebugDraw.h"
#include "DetourStatNavMesh.h"
#include "DetourStatNavMeshBuilder.h"
#include "DetourDebugDraw.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

BuilderStatMeshTiling::BuilderStatMeshTiling() :
	m_keepInterResults(false),
	m_tileSize(64),
	m_chunkyMesh(0),
	m_tileSet(0),
	m_polyMesh(0),
	m_drawMode(DRAWMODE_NAVMESH)
{
}

BuilderStatMeshTiling::~BuilderStatMeshTiling()
{
	cleanup();
}

void BuilderStatMeshTiling::cleanup()
{
	delete m_chunkyMesh;
	m_chunkyMesh = 0;
	delete m_tileSet;
	m_tileSet = 0;
	delete m_polyMesh;
	m_polyMesh = 0;
	toolCleanup();
}

void BuilderStatMeshTiling::handleSettings()
{
	Builder::handleCommonSettings();
	
	imguiLabel(GENID, "Tiling");
	imguiSlider(GENID, "TileSize", &m_tileSize, 16.0f, 1024.0f, 16.0f);

	char text[64];
	int gw = 0, gh = 0;
	rcCalcGridSize(m_bmin, m_bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;
	snprintf(text, 64, "Tiles  %d x %d", tw, th);
	imguiValue(GENID, text);
	
	imguiSeparator();
	if (imguiCheck(GENID, "Keep Itermediate Results", m_keepInterResults))
		m_keepInterResults = !m_keepInterResults;
	
	imguiSeparator();
}

void BuilderStatMeshTiling::handleDebugMode()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		valid[i] = false;

	bool hasChf = false;
	bool hasSolid = false;
	bool hasCset = false;
	if (m_tileSet)
	{
		for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
		{
			if (m_tileSet->tiles[i].solid) hasSolid = true;
			if (m_tileSet->tiles[i].chf) hasChf = true;
			if (m_tileSet->tiles[i].cset) hasCset = true;
		}
	}
	
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
		valid[DRAWMODE_POLYMESH] = m_polyMesh != 0;
	}
	
	if (!valid[m_drawMode])
		m_drawMode = DRAWMODE_MESH;
	
	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		if (!valid[i]) unavail++;
	
	if (unavail == MAX_DRAWMODE)
		return;
	
	imguiLabel(GENID, "Draw");
	if (valid[DRAWMODE_MESH] && imguiCheck(GENID, "Input Mesh", m_drawMode == DRAWMODE_MESH))
		m_drawMode = DRAWMODE_MESH;
	if (valid[DRAWMODE_NAVMESH] && imguiCheck(GENID, "Navmesh", m_drawMode == DRAWMODE_NAVMESH))
		m_drawMode = DRAWMODE_NAVMESH;
	if (valid[DRAWMODE_NAVMESH_INVIS] && imguiCheck(GENID, "Navmesh Invis", m_drawMode == DRAWMODE_NAVMESH_INVIS))
		m_drawMode = DRAWMODE_NAVMESH_INVIS;
	if (valid[DRAWMODE_NAVMESH_TRANS] && imguiCheck(GENID, "Navmesh Trans", m_drawMode == DRAWMODE_NAVMESH_TRANS))
		m_drawMode = DRAWMODE_NAVMESH_TRANS;
	if (valid[DRAWMODE_NAVMESH_BVTREE] && imguiCheck(GENID, "Navmesh BVTree", m_drawMode == DRAWMODE_NAVMESH_BVTREE))
		m_drawMode = DRAWMODE_NAVMESH_BVTREE;
	if (valid[DRAWMODE_VOXELS] && imguiCheck(GENID, "Voxels", m_drawMode == DRAWMODE_VOXELS))
		m_drawMode = DRAWMODE_VOXELS;
	if (valid[DRAWMODE_VOXELS_WALKABLE] && imguiCheck(GENID, "Walkable Voxels", m_drawMode == DRAWMODE_VOXELS_WALKABLE))
		m_drawMode = DRAWMODE_VOXELS_WALKABLE;
	if (valid[DRAWMODE_COMPACT] && imguiCheck(GENID, "Compact", m_drawMode == DRAWMODE_COMPACT))
		m_drawMode = DRAWMODE_COMPACT;
	if (valid[DRAWMODE_COMPACT_DISTANCE] && imguiCheck(GENID, "Compact Distance", m_drawMode == DRAWMODE_COMPACT_DISTANCE))
		m_drawMode = DRAWMODE_COMPACT_DISTANCE;
	if (valid[DRAWMODE_COMPACT_REGIONS] && imguiCheck(GENID, "Compact Regions", m_drawMode == DRAWMODE_COMPACT_REGIONS))
		m_drawMode = DRAWMODE_COMPACT_REGIONS;
	if (valid[DRAWMODE_REGION_CONNECTIONS] && imguiCheck(GENID, "Region Connections", m_drawMode == DRAWMODE_REGION_CONNECTIONS))
		m_drawMode = DRAWMODE_REGION_CONNECTIONS;
	if (valid[DRAWMODE_RAW_CONTOURS] && imguiCheck(GENID, "Raw Contours", m_drawMode == DRAWMODE_RAW_CONTOURS))
		m_drawMode = DRAWMODE_RAW_CONTOURS;
	if (valid[DRAWMODE_BOTH_CONTOURS] && imguiCheck(GENID, "Both Contours", m_drawMode == DRAWMODE_BOTH_CONTOURS))
		m_drawMode = DRAWMODE_BOTH_CONTOURS;
	if (valid[DRAWMODE_CONTOURS] && imguiCheck(GENID, "Contours", m_drawMode == DRAWMODE_CONTOURS))
		m_drawMode = DRAWMODE_CONTOURS;
	if (valid[DRAWMODE_POLYMESH] && imguiCheck(GENID, "Poly Mesh", m_drawMode == DRAWMODE_POLYMESH))
		m_drawMode = DRAWMODE_POLYMESH;
	
	if (unavail)
	{
		imguiValue(GENID, "Tick 'Keep Itermediate Results'");
		imguiValue(GENID, "to see more debug mode options.");
	}
}

void BuilderStatMeshTiling::handleRender()
{
	if (!m_verts || !m_tris || !m_trinorms)
		return;
	
	float col[4];
	
	glEnable(GL_FOG);
	glDepthMask(GL_TRUE);
	
	if (m_drawMode == DRAWMODE_MESH)
	{
		// Draw mesh
		rcDebugDrawMeshSlope(m_verts, m_nverts, m_tris, m_trinorms, m_ntris, m_agentMaxSlope);
	}
	else if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		rcDebugDrawMesh(m_verts, m_nverts, m_tris, m_trinorms, m_ntris, 0);
	}
	
	glDisable(GL_FOG);
	glDepthMask(GL_FALSE);
	
	// Draw bounds
	col[0] = 1; col[1] = 1; col[2] = 1; col[3] = 0.5f;
	rcDebugDrawBoxWire(m_bmin[0],m_bmin[1],m_bmin[2], m_bmax[0],m_bmax[1],m_bmax[2], col);
	
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
					rcDebugDrawCompactHeightfieldSolid(*m_tileSet->tiles[i].chf);
			}
		}
		
		if (m_drawMode == DRAWMODE_COMPACT_DISTANCE)
		{
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].chf)
					rcDebugDrawCompactHeightfieldDistance(*m_tileSet->tiles[i].chf);
			}
		}
		if (m_drawMode == DRAWMODE_COMPACT_REGIONS)
		{
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].chf)
					rcDebugDrawCompactHeightfieldRegions(*m_tileSet->tiles[i].chf);
			}
		}
			
		if (m_drawMode == DRAWMODE_VOXELS)
		{
			glEnable(GL_FOG);
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].solid)
					rcDebugDrawHeightfieldSolid(*m_tileSet->tiles[i].solid);
			}
			glDisable(GL_FOG);
		}
		if (m_drawMode == DRAWMODE_VOXELS_WALKABLE)
		{
			glEnable(GL_FOG);
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].solid)
					rcDebugDrawHeightfieldWalkable(*m_tileSet->tiles[i].solid);
			}
			glDisable(GL_FOG);
		}
		if (m_drawMode == DRAWMODE_RAW_CONTOURS)
		{
			glDepthMask(GL_FALSE);
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].cset)
					rcDebugDrawRawContours(*m_tileSet->tiles[i].cset, m_cfg.bmin, m_cfg.cs, m_cfg.ch);
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
					rcDebugDrawRawContours(*m_tileSet->tiles[i].cset, m_cfg.bmin, m_cfg.cs, m_cfg.ch, 0.5f);
					rcDebugDrawContours(*m_tileSet->tiles[i].cset, m_cfg.bmin, m_cfg.cs, m_cfg.ch);
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
					rcDebugDrawContours(*m_tileSet->tiles[i].cset, m_cfg.bmin, m_cfg.cs, m_cfg.ch);
			}
			glDepthMask(GL_TRUE);
		}
		if (m_drawMode == DRAWMODE_REGION_CONNECTIONS)
		{
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].chf)
					rcDebugDrawCompactHeightfieldRegions(*m_tileSet->tiles[i].chf);
			}
			
			glDepthMask(GL_FALSE);
			for (int i = 0; i < m_tileSet->width*m_tileSet->height; ++i)
			{
				if (m_tileSet->tiles[i].cset)
					rcDebugDrawRegionConnections(*m_tileSet->tiles[i].cset, m_cfg.bmin, m_cfg.cs, m_cfg.ch);
			}
			glDepthMask(GL_TRUE);
		}
		if (m_polyMesh && m_drawMode == DRAWMODE_POLYMESH)
		{
			glDepthMask(GL_FALSE);
			rcDebugDrawPolyMesh(*m_polyMesh);
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

void BuilderStatMeshTiling::handleRenderOverlay(class GLFont* font, double* proj, double* model, int* view)
{
	toolRenderOverlay(font, proj, model, view);
}

void BuilderStatMeshTiling::handleMeshChanged(const float* verts, int nverts,
											  const int* tris, const float* trinorms, int ntris,
											  const float* bmin, const float* bmax)
{
	Builder::handleMeshChanged(verts, nverts, tris, trinorms, ntris, bmin, bmax);
	toolCleanup();
	toolReset();
}

bool BuilderStatMeshTiling::handleBuild()
{
	if (!m_verts || ! m_tris)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
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
	m_cfg.borderSize = m_cfg.walkableRadius*2 + 2; // Reserve enough padding.
	
	if (m_cfg.maxVertsPerPoly == DT_VERTS_PER_POLYGON)
		m_drawMode = DRAWMODE_NAVMESH;
	else
		m_drawMode = DRAWMODE_POLYMESH;
	
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
			
			if (m_keepInterResults)
			{
				tile.solid = solid;
				solid = 0;
				tile.chf = chf;
				chf = 0;
			}
			
			if (!cset->nconts)
			{
				delete cset;
				cset = 0;
				continue;
			}
			
			tile.cset = cset;
			// Offset the vertices in the cset.
			rcTranslateContours(tile.cset, x*tileCfg.tileSize - tileCfg.borderSize, 0, y*tileCfg.tileSize - tileCfg.borderSize);
			
			rcTimeVal endTime = rcGetPerformanceTimer();
			tile.buildTime += rcGetDeltaTimeUsec(startTime, endTime);
		}
	}
	
	delete [] triangleFlags;
	delete solid;
	delete chf;
		
	// Make sure that the vertices along the tile edges match,
	// so that they can be later properly stitched together.
	for (int y = 0; y < m_tileSet->height; ++y)
	{
		for (int x = 0; x < m_tileSet->width; ++x)
		{
			rcTimeVal startTime = rcGetPerformanceTimer();
			if ((x+1) < m_tileSet->width)
			{
				if (!rcFixupAdjacentContours(m_tileSet->tiles[x + y*m_tileSet->width].cset,
											 m_tileSet->tiles[x+1 + y*m_tileSet->width].cset,
											 m_cfg.walkableClimb, (x+1)*m_cfg.tileSize, -1))
				{
					if (rcGetLog())
						rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not fixup x+1.", x, y);
					return false;
				}
			}
			
			if ((y+1) < m_tileSet->height)
			{
				if (!rcFixupAdjacentContours(m_tileSet->tiles[x + y*m_tileSet->width].cset,
											 m_tileSet->tiles[x + (y+1)*m_tileSet->width].cset,
											 m_cfg.walkableClimb, -1, (y+1)*m_cfg.tileSize))
				{
					if (rcGetLog())
						rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not fixup y+1.", x, y);
					return false;
				}
			}
			rcTimeVal endTime = rcGetPerformanceTimer();
			m_tileSet->tiles[x+y*m_tileSet->width].buildTime += rcGetDeltaTimeUsec(startTime, endTime);
		}
	}
		
		
	// Combine contours.
	rcContourSet combSet;
	
	combSet.nconts = 0;
	for (int y = 0; y < m_tileSet->height; ++y)
	{
		for (int x = 0; x < m_tileSet->width; ++x)
		{
			Tile& tile = m_tileSet->tiles[x + y*m_tileSet->width]; 
			if (!tile.cset) continue;
			combSet.nconts += tile.cset->nconts;
		}
	}
	combSet.conts = new rcContour[combSet.nconts];
	if (!combSet.conts)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'combSet.conts' (%d).", combSet.nconts);
		return false;
	}
	int n = 0;
	for (int y = 0; y < m_tileSet->height; ++y)
	{
		for (int x = 0; x < m_tileSet->width; ++x)
		{
			Tile& tile = m_tileSet->tiles[x + y*m_tileSet->width]; 
			if (!tile.cset) continue;
			for (int i = 0; i < tile.cset->nconts; ++i)
			{
				combSet.conts[n].verts = tile.cset->conts[i].verts;
				combSet.conts[n].nverts = tile.cset->conts[i].nverts;
				combSet.conts[n].reg = tile.cset->conts[i].reg;
				n++;
			}
		}
	}
	
	m_polyMesh = new rcPolyMesh;
	if (!m_polyMesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'polyMesh'.");
		return false;
	}

	bool polyRes = rcBuildPolyMesh(combSet, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch, m_cfg.maxVertsPerPoly, *m_polyMesh);
	
	// Remove vertex binding to avoid double deletion.
	for (int i = 0; i < combSet.nconts; ++i)
	{
		combSet.conts[i].verts = 0;
		combSet.conts[i].nverts = 0;
	}
	
	if (!polyRes)
	{	
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Could not triangulate contours.");
		return false;
	}
	
	if (!m_keepInterResults)
	{
		for (int y = 0; y < m_tileSet->height; ++y)
		{
			for (int x = 0; x < m_tileSet->width; ++x)
			{
				Tile& tile = m_tileSet->tiles[x + y*m_tileSet->width]; 
				delete tile.cset;
				tile.cset = 0;
			}
		}
	}
	
	if (m_cfg.maxVertsPerPoly == DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;
		if (!dtCreateNavMeshData(m_polyMesh->verts, m_polyMesh->nverts,
								 m_polyMesh->polys, m_polyMesh->npolys, m_polyMesh->nvp,
								 m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch, &navData, &navDataSize))
		{
			if (rcGetLog())
				rcGetLog()->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}
		
		m_navMesh = new dtStatNavMesh;
		if (!m_navMesh)
		{
			delete [] navData;
			if (rcGetLog())
				rcGetLog()->log(RC_LOG_ERROR, "Could not create Detour navmesh");
			return false;
		}
		
		if (!m_navMesh->init(navData, navDataSize, true))
		{
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
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Fixup contours: %.1fms (%.1f%%)", m_buildTimes.fixupContours/1000.0f, m_buildTimes.fixupContours*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Polymesh: %.1fms (%.1f%%)", m_buildTimes.buildPolymesh/1000.0f, m_buildTimes.buildPolymesh*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Polymesh: Verts:%d  Polys:%d", m_polyMesh->nverts, m_polyMesh->npolys);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "TOTAL: %.1fms", rcGetDeltaTimeUsec(totStartTime, totEndTime)/1000.0f);
	}

	toolRecalc();

	return true;
}
