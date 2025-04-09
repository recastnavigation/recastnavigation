//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "Sample_SoloMesh.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "imgui.h"
#include "InputGeom.h"
#include "NavMeshPruneTool.h"
#include "NavMeshTesterTool.h"
#include "OffMeshConnectionTool.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "RecastDump.h"
#include "Sample.h"
#include "SDL.h"
#include "SDL_opengl.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

Sample_SoloMesh::Sample_SoloMesh()
{
	setTool(new NavMeshTesterTool);
}

Sample_SoloMesh::~Sample_SoloMesh()
{
	cleanup();
}

void Sample_SoloMesh::cleanup()
{
	delete [] m_triareas; m_triareas = 0;
	rcFreeHeightField(m_heightfield); m_heightfield = 0;
	rcFreeCompactHeightfield(m_compactHeightfield); m_compactHeightfield = 0;
	rcFreeContourSet(m_contourSet); m_contourSet = 0;
	rcFreePolyMesh(m_polyMesh); m_polyMesh = 0;
	rcFreePolyMeshDetail(m_detailMesh); m_detailMesh = 0;
	dtFreeNavMesh(m_navMesh); m_navMesh = 0;
}

void Sample_SoloMesh::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_keepIntermediateResults))
	{
		m_keepIntermediateResults = !m_keepIntermediateResults;
	}

	imguiSeparator();

	imguiIndent();
	imguiIndent();

	if (imguiButton("Save"))
	{
		Sample::saveAll("solo_navmesh.bin", m_navMesh);
	}

	if (imguiButton("Load"))
	{
		dtFreeNavMesh(m_navMesh);
		m_navMesh = Sample::loadAll("solo_navmesh.bin");
		m_navQuery->init(m_navMesh, 2048);
	}

	imguiUnindent();
	imguiUnindent();

	char msg[64];
	snprintf(msg, 64, "Build Time: %.1fms", m_totalBuildTimeMs);
	imguiLabel(msg);

	imguiSeparator();
}

void Sample_SoloMesh::handleTools()
{
	const SampleToolType type = !m_tool ? SampleToolType::NONE : m_tool->type();

	if (imguiCheck("Test Navmesh", type == SampleToolType::NAVMESH_TESTER)) { setTool(new NavMeshTesterTool); }
	if (imguiCheck("Prune Navmesh", type == SampleToolType::NAVMESH_PRUNE)) { setTool(new NavMeshPruneTool); }
	if (imguiCheck("Create Off-Mesh Connections", type == SampleToolType::OFFMESH_CONNECTION)) { setTool(new OffMeshConnectionTool); }
	if (imguiCheck("Create Convex Volumes", type == SampleToolType::CONVEX_VOLUME)) { setTool(new ConvexVolumeTool); }
	if (imguiCheck("Create Crowds", type == SampleToolType::CROWD)) { setTool(new CrowdTool); }

	imguiSeparatorLine();

	imguiIndent();

	if (m_tool)
	{
		m_tool->handleMenu();
	}

	imguiUnindent();
}

void Sample_SoloMesh::UI_DrawModeOption(const char* name, DrawMode drawMode, bool enabled)
{
	if (imguiCheck(name, m_drawMode == drawMode, enabled))
	{
		m_drawMode = drawMode;
	}
}

void Sample_SoloMesh::handleDebugMode()
{
	imguiLabel("Draw");
	UI_DrawModeOption("Input Mesh", DrawMode::MESH, true);
	UI_DrawModeOption("Navmesh", DrawMode::NAVMESH, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh Invis", DrawMode::NAVMESH_INVIS, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh Trans", DrawMode::NAVMESH_TRANS, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh BVTree", DrawMode::NAVMESH_BVTREE, m_navMesh != nullptr);
	UI_DrawModeOption("Navmesh Nodes", DrawMode::NAVMESH_NODES, m_navQuery != nullptr);
	UI_DrawModeOption("Voxels", DrawMode::VOXELS, m_heightfield != nullptr);
	UI_DrawModeOption("Walkable Voxels", DrawMode::VOXELS_WALKABLE, m_heightfield != nullptr);
	UI_DrawModeOption("Compact", DrawMode::COMPACT, m_compactHeightfield != nullptr);
	UI_DrawModeOption("Compact Distance", DrawMode::COMPACT_DISTANCE, m_compactHeightfield != nullptr);
	UI_DrawModeOption("Compact Regions", DrawMode::COMPACT_REGIONS, m_compactHeightfield != nullptr);
	UI_DrawModeOption("Region Connections", DrawMode::REGION_CONNECTIONS, m_contourSet != nullptr);
	UI_DrawModeOption("Raw Contours", DrawMode::RAW_CONTOURS, m_contourSet != nullptr);
	UI_DrawModeOption("Both Contours", DrawMode::BOTH_CONTOURS, m_contourSet != nullptr);
	UI_DrawModeOption("Contours", DrawMode::CONTOURS, m_contourSet != nullptr);
	UI_DrawModeOption("Poly Mesh", DrawMode::POLYMESH, m_polyMesh != nullptr);
	UI_DrawModeOption("Poly Mesh Detail", DrawMode::POLYMESH_DETAIL, m_detailMesh != nullptr);
}

void Sample_SoloMesh::handleRender()
{
	if (!m_inputGeometry || !m_inputGeometry->getMesh())
	{
		return;
	}

	glEnable(GL_FOG);
	glDepthMask(GL_TRUE);

	const float texScale = 1.0f / (m_cellSize * 10.0f);

	if (m_drawMode != DrawMode::NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(&m_debugDraw, m_inputGeometry->getMesh()->getVerts(), m_inputGeometry->getMesh()->getVertCount(),
								m_inputGeometry->getMesh()->getTris(), m_inputGeometry->getMesh()->getNormals(), m_inputGeometry->getMesh()->getTriCount(),
								m_agentMaxSlope, texScale);
		m_inputGeometry->drawOffMeshConnections(&m_debugDraw);
	}

	glDisable(GL_FOG);
	glDepthMask(GL_FALSE);

	// Draw bounds
	const float* navmeshBoundsMin = m_inputGeometry->getNavMeshBoundsMin();
	const float* navmeshBoundsMax = m_inputGeometry->getNavMeshBoundsMax();
	duDebugDrawBoxWire(&m_debugDraw, navmeshBoundsMin[0],navmeshBoundsMin[1],navmeshBoundsMin[2], navmeshBoundsMax[0],navmeshBoundsMax[1],navmeshBoundsMax[2], duRGBA(255,255,255,128), 1.0f);
	m_debugDraw.begin(DU_DRAW_POINTS, 5.0f);
	m_debugDraw.vertex(navmeshBoundsMin[0],navmeshBoundsMin[1],navmeshBoundsMin[2],duRGBA(255,255,255,128));
	m_debugDraw.end();

	if (m_navMesh && m_navQuery &&
	    (m_drawMode == DrawMode::NAVMESH ||
	     m_drawMode == DrawMode::NAVMESH_TRANS ||
	     m_drawMode == DrawMode::NAVMESH_BVTREE ||
	     m_drawMode == DrawMode::NAVMESH_NODES ||
	     m_drawMode == DrawMode::NAVMESH_INVIS))
	{
		if (m_drawMode != DrawMode::NAVMESH_INVIS)
		{
			duDebugDrawNavMeshWithClosedList(&m_debugDraw, *m_navMesh, *m_navQuery, m_navMeshDrawFlags);
		}
		if (m_drawMode == DrawMode::NAVMESH_BVTREE)
		{
			duDebugDrawNavMeshBVTree(&m_debugDraw, *m_navMesh);
		}
		if (m_drawMode == DrawMode::NAVMESH_NODES)
		{
			duDebugDrawNavMeshNodes(&m_debugDraw, *m_navQuery);
		}
		duDebugDrawNavMeshPolysWithFlags(&m_debugDraw, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
	}

	glDepthMask(GL_TRUE);

	if (m_compactHeightfield && m_drawMode == DrawMode::COMPACT)
	{
		duDebugDrawCompactHeightfieldSolid(&m_debugDraw, *m_compactHeightfield);
	}

	if (m_compactHeightfield && m_drawMode == DrawMode::COMPACT_DISTANCE)
	{
		duDebugDrawCompactHeightfieldDistance(&m_debugDraw, *m_compactHeightfield);
	}
	if (m_compactHeightfield && m_drawMode == DrawMode::COMPACT_REGIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&m_debugDraw, *m_compactHeightfield);
	}
	if (m_heightfield && m_drawMode == DrawMode::VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&m_debugDraw, *m_heightfield);
		glDisable(GL_FOG);
	}
	if (m_heightfield && m_drawMode == DrawMode::VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&m_debugDraw, *m_heightfield);
		glDisable(GL_FOG);
	}
	if (m_contourSet && m_drawMode == DrawMode::RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_contourSet && m_drawMode == DrawMode::BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_debugDraw, *m_contourSet, 0.5f);
		duDebugDrawContours(&m_debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_contourSet && m_drawMode == DrawMode::CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&m_debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_compactHeightfield && m_contourSet && m_drawMode == DrawMode::REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&m_debugDraw, *m_compactHeightfield);

		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&m_debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_polyMesh && m_drawMode == DrawMode::POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&m_debugDraw, *m_polyMesh);
		glDepthMask(GL_TRUE);
	}
	if (m_detailMesh && m_drawMode == DrawMode::POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&m_debugDraw, *m_detailMesh);
		glDepthMask(GL_TRUE);
	}

	m_inputGeometry->drawConvexVolumes(&m_debugDraw);

	if (m_tool)
	{
		m_tool->handleRender();
	}
	renderToolStates();

	glDepthMask(GL_TRUE);
}

void Sample_SoloMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
	if (m_tool)
	{
		m_tool->handleRenderOverlay(proj, model, view);
	}
	renderOverlayToolStates(proj, model, view);
}

void Sample_SoloMesh::handleMeshChanged(InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

	dtFreeNavMesh(m_navMesh); m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

bool Sample_SoloMesh::handleBuild()
{
	if (!m_inputGeometry || !m_inputGeometry->getMesh())
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}

	cleanup();

	const float* bmin = m_inputGeometry->getNavMeshBoundsMin();
	const float* bmax = m_inputGeometry->getNavMeshBoundsMax();
	const float* verts = m_inputGeometry->getMesh()->getVerts();
	const int nverts = m_inputGeometry->getMesh()->getVertCount();
	const int* tris = m_inputGeometry->getMesh()->getTris();
	const int ntris = m_inputGeometry->getMesh()->getTriCount();

	//
	// Step 1. Initialize build config.
	//

	// Init build configuration from GUI
	memset(&m_config, 0, sizeof(m_config));
	m_config.cs = m_cellSize;
	m_config.ch = m_cellHeight;
	m_config.walkableSlopeAngle = m_agentMaxSlope;
	m_config.walkableHeight = static_cast<int>(ceilf(m_agentHeight / m_config.ch));
	m_config.walkableClimb = static_cast<int>(floorf(m_agentMaxClimb / m_config.ch));
	m_config.walkableRadius = static_cast<int>(ceilf(m_agentRadius / m_config.cs));
	m_config.maxEdgeLen = static_cast<int>(m_edgeMaxLen / m_cellSize);
	m_config.maxSimplificationError = m_edgeMaxError;
	m_config.minRegionArea = static_cast<int>(rcSqr(m_regionMinSize));		// Note: area = size*size
	m_config.mergeRegionArea = static_cast<int>(rcSqr(m_regionMergeSize));	// Note: area = size*size
	m_config.maxVertsPerPoly = static_cast<int>(m_vertsPerPoly);
	m_config.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_config.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

	// Set the area where the navigation will be built.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by a user defined box, etc.
	rcVcopy(m_config.bmin, bmin);
	rcVcopy(m_config.bmax, bmax);
	rcCalcGridSize(m_config.bmin, m_config.bmax, m_config.cs, &m_config.width, &m_config.height);

	// Reset build times gathering.
	m_buildContext->resetTimers();

	// Start the build process.
	m_buildContext->startTimer(RC_TIMER_TOTAL);

	m_buildContext->log(RC_LOG_PROGRESS, "Building navigation:");
	m_buildContext->log(RC_LOG_PROGRESS, " - %d x %d cells", m_config.width, m_config.height);
	m_buildContext->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", static_cast<float>(nverts) / 1000.0f, static_cast<float>(ntris) / 1000.0f);

	//
	// Step 2. Rasterize input polygon soup.
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	m_heightfield = rcAllocHeightfield();
	if (!m_heightfield)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return false;
	}
	if (!rcCreateHeightfield(m_buildContext, *m_heightfield, m_config.width, m_config.height, m_config.bmin, m_config.bmax, m_config.cs, m_config.ch))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}

	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	m_triareas = new unsigned char[ntris];
	if (!m_triareas)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
		return false;
	}

	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the type for each mesh, and rasterize them.
	memset(m_triareas, 0, ntris*sizeof(unsigned char));
	rcMarkWalkableTriangles(m_buildContext, m_config.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
	if (!rcRasterizeTriangles(m_buildContext, verts, nverts, tris, m_triareas, ntris, *m_heightfield, m_config.walkableClimb))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return false;
	}

	if (!m_keepIntermediateResults)
	{
		delete [] m_triareas;
		m_triareas = 0;
	}

	//
	// Step 3. Filter walkable surfaces.
	//

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (m_filterLowHangingObstacles)
	{
		rcFilterLowHangingWalkableObstacles(m_buildContext, m_config.walkableClimb, *m_heightfield);
	}
	if (m_filterLedgeSpans)
	{
		rcFilterLedgeSpans(m_buildContext, m_config.walkableHeight, m_config.walkableClimb, *m_heightfield);
	}
	if (m_filterWalkableLowHeightSpans)
	{
		rcFilterWalkableLowHeightSpans(m_buildContext, m_config.walkableHeight, *m_heightfield);
	}

	//
	// Step 4. Partition walkable surface to simple regions.
	//

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	m_compactHeightfield = rcAllocCompactHeightfield();
	if (!m_compactHeightfield)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(m_buildContext, m_config.walkableHeight, m_config.walkableClimb, *m_heightfield, *m_compactHeightfield))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}

	if (!m_keepIntermediateResults)
	{
		rcFreeHeightField(m_heightfield);
		m_heightfield = 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_buildContext, m_config.walkableRadius, *m_compactHeightfield))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return false;
	}

	// (Optional) Mark areas.
	const ConvexVolume* vols = m_inputGeometry->getConvexVolumes();
	for (int i  = 0; i < m_inputGeometry->getConvexVolumeCount(); ++i)
	{
		rcMarkConvexPolyArea(m_buildContext, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_compactHeightfield);
	}

	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	// There are 3 partitioning methods, each with some pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or overlaps
	//   - the are some corner cases where this method creates produces holes and overlaps
	//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the navmesh, use this if you have large open areas
	// 2) Monotone partitioning
	//   - fastest
	//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
	//   - creates long thin polygons, which sometimes causes paths with detours
	//   * use this if you want fast navmesh generation
	// 3) Layer partitoining
	//   - quite fast
	//   - partitions the heighfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a bit ugly tessellation (still better than monotone)
	//     if you have large open areas with small obstacles (not a problem if you use tiles)
	//   * good choice to use for tiled navmesh with medium and small sized tiles

	if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(m_buildContext, *m_compactHeightfield))
		{
			m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return false;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(m_buildContext, *m_compactHeightfield, 0, m_config.minRegionArea, m_config.mergeRegionArea))
		{
			m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return false;
		}
	}
	else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(m_buildContext, *m_compactHeightfield, 0, m_config.minRegionArea, m_config.mergeRegionArea))
		{
			m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return false;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(m_buildContext, *m_compactHeightfield, 0, m_config.minRegionArea))
		{
			m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return false;
		}
	}

	//
	// Step 5. Trace and simplify region contours.
	//

	// Create contours.
	m_contourSet = rcAllocContourSet();
	if (!m_contourSet)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(m_buildContext, *m_compactHeightfield, m_config.maxSimplificationError, m_config.maxEdgeLen, *m_contourSet))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//

	// Build polygon navmesh from the contours.
	m_polyMesh = rcAllocPolyMesh();
	if (!m_polyMesh)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	if (!rcBuildPolyMesh(m_buildContext, *m_contourSet, m_config.maxVertsPerPoly, *m_polyMesh))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//

	m_detailMesh = rcAllocPolyMeshDetail();
	if (!m_detailMesh)
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}
	if (!rcBuildPolyMeshDetail(m_buildContext, *m_polyMesh, *m_compactHeightfield, m_config.detailSampleDist, m_config.detailSampleMaxError, *m_detailMesh))
	{
		m_buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return false;
	}

	if (!m_keepIntermediateResults)
	{
		rcFreeCompactHeightfield(m_compactHeightfield);
		m_compactHeightfield = 0;
		rcFreeContourSet(m_contourSet);
		m_contourSet = 0;
	}

	// At this point the navigation mesh data is ready, you can access it from m_pmesh.
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh.
	//

	// The GUI may allow more max points per polygon than Detour can handle.
	// Only build the detour navmesh if we do not exceed the limit.
	if (m_config.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

		// Update poly flags from areas.
		for (int i = 0; i < m_polyMesh->npolys; ++i)
		{
			if (m_polyMesh->areas[i] == RC_WALKABLE_AREA)
			{
				m_polyMesh->areas[i] = SAMPLE_POLYAREA_GROUND;
			}

			if (m_polyMesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
			    m_polyMesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
			    m_polyMesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				m_polyMesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (m_polyMesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				m_polyMesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (m_polyMesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				m_polyMesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_polyMesh->verts;
		params.vertCount = m_polyMesh->nverts;
		params.polys = m_polyMesh->polys;
		params.polyAreas = m_polyMesh->areas;
		params.polyFlags = m_polyMesh->flags;
		params.polyCount = m_polyMesh->npolys;
		params.nvp = m_polyMesh->nvp;
		params.detailMeshes = m_detailMesh->meshes;
		params.detailVerts = m_detailMesh->verts;
		params.detailVertsCount = m_detailMesh->nverts;
		params.detailTris = m_detailMesh->tris;
		params.detailTriCount = m_detailMesh->ntris;
		params.offMeshConVerts = m_inputGeometry->getOffMeshConnectionVerts();
		params.offMeshConRad = m_inputGeometry->getOffMeshConnectionRads();
		params.offMeshConDir = m_inputGeometry->getOffMeshConnectionDirs();
		params.offMeshConAreas = m_inputGeometry->getOffMeshConnectionAreas();
		params.offMeshConFlags = m_inputGeometry->getOffMeshConnectionFlags();
		params.offMeshConUserID = m_inputGeometry->getOffMeshConnectionId();
		params.offMeshConCount = m_inputGeometry->getOffMeshConnectionCount();
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		rcVcopy(params.bmin, m_polyMesh->bmin);
		rcVcopy(params.bmax, m_polyMesh->bmax);
		params.cs = m_config.cs;
		params.ch = m_config.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_buildContext->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}

		m_navMesh = dtAllocNavMesh();
		if (!m_navMesh)
		{
			dtFree(navData);
			m_buildContext->log(RC_LOG_ERROR, "Could not create Detour navmesh");
			return false;
		}

		dtStatus status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			m_buildContext->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return false;
		}

		status = m_navQuery->init(m_navMesh, 2048);
		if (dtStatusFailed(status))
		{
			m_buildContext->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
			return false;
		}
	}

	m_buildContext->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*m_buildContext, m_buildContext->getAccumulatedTime(RC_TIMER_TOTAL));
	m_buildContext->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_polyMesh->nverts, m_polyMesh->npolys);

	m_totalBuildTimeMs = static_cast<float>(m_buildContext->getAccumulatedTime(RC_TIMER_TOTAL)) / 1000.0f;

	if (m_tool)
	{
		m_tool->init(this);
	}
	initToolStates(this);

	return true;
}
