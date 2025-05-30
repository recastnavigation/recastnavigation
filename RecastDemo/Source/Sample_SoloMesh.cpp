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
	dtFreeNavMesh(navMesh); navMesh = 0;
}

void Sample_SoloMesh::handleSettings()
{
	handleCommonSettings();

	imguiSeparator();

	imguiIndent();
	imguiIndent();

	if (imguiButton("Save"))
	{
		saveAll("solo_navmesh.bin", navMesh);
	}

	if (imguiButton("Load"))
	{
		dtFreeNavMesh(navMesh);
		navMesh = loadAll("solo_navmesh.bin");
		navQuery->init(navMesh, 2048);
	}

	imguiUnindent();
	imguiUnindent();

	char message[64];
	snprintf(message, 64, "Build Time: %.1fms", m_totalBuildTimeMs);
	imguiLabel(message);

	imguiSeparator();
}

void Sample_SoloMesh::handleTools()
{
	const SampleToolType type = !tool ? SampleToolType::NONE : tool->type();

	if (imguiCheck("Test Navmesh", type == SampleToolType::NAVMESH_TESTER)) { setTool(new NavMeshTesterTool); }
	if (imguiCheck("Prune Navmesh", type == SampleToolType::NAVMESH_PRUNE)) { setTool(new NavMeshPruneTool); }
	if (imguiCheck("Create Off-Mesh Connections", type == SampleToolType::OFFMESH_CONNECTION)) { setTool(new OffMeshConnectionTool); }
	if (imguiCheck("Create Convex Volumes", type == SampleToolType::CONVEX_VOLUME)) { setTool(new ConvexVolumeTool); }
	if (imguiCheck("Create Crowds", type == SampleToolType::CROWD)) { setTool(new CrowdTool); }

	imguiSeparatorLine();

	imguiIndent();

	if (tool)
	{
		tool->handleMenu();
	}

	imguiUnindent();
}

void Sample_SoloMesh::UI_DrawModeOption(const char* name, const DrawMode drawMode, const bool enabled)
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
	UI_DrawModeOption("Navmesh", DrawMode::NAVMESH, navMesh != nullptr);
	UI_DrawModeOption("Navmesh Invis", DrawMode::NAVMESH_INVIS, navMesh != nullptr);
	UI_DrawModeOption("Navmesh Trans", DrawMode::NAVMESH_TRANS, navMesh != nullptr);
	UI_DrawModeOption("Navmesh BVTree", DrawMode::NAVMESH_BVTREE, navMesh != nullptr);
	UI_DrawModeOption("Navmesh Nodes", DrawMode::NAVMESH_NODES, navQuery != nullptr);
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
	if (!inputGeometry || !inputGeometry->getMesh())
	{
		return;
	}

	glEnable(GL_FOG);
	glDepthMask(GL_TRUE);

	const float texScale = 1.0f / (cellSize * 10.0f);

	if (m_drawMode != DrawMode::NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(&debugDraw, inputGeometry->getMesh()->getVerts(), inputGeometry->getMesh()->getVertCount(),
								inputGeometry->getMesh()->getTris(), inputGeometry->getMesh()->getNormals(), inputGeometry->getMesh()->getTriCount(),
								agentMaxSlope, texScale);
		inputGeometry->drawOffMeshConnections(&debugDraw);
	}

	glDisable(GL_FOG);
	glDepthMask(GL_FALSE);

	// Draw bounds
	const float* navmeshBoundsMin = inputGeometry->getNavMeshBoundsMin();
	const float* navmeshBoundsMax = inputGeometry->getNavMeshBoundsMax();
	duDebugDrawBoxWire(&debugDraw, navmeshBoundsMin[0],navmeshBoundsMin[1],navmeshBoundsMin[2], navmeshBoundsMax[0],navmeshBoundsMax[1],navmeshBoundsMax[2], duRGBA(255,255,255,128), 1.0f);
	debugDraw.begin(DU_DRAW_POINTS, 5.0f);
	debugDraw.vertex(navmeshBoundsMin[0],navmeshBoundsMin[1],navmeshBoundsMin[2],duRGBA(255,255,255,128));
	debugDraw.end();

	if (navMesh && navQuery &&
	    (m_drawMode == DrawMode::NAVMESH ||
	     m_drawMode == DrawMode::NAVMESH_TRANS ||
	     m_drawMode == DrawMode::NAVMESH_BVTREE ||
	     m_drawMode == DrawMode::NAVMESH_NODES ||
	     m_drawMode == DrawMode::NAVMESH_INVIS))
	{
		if (m_drawMode != DrawMode::NAVMESH_INVIS)
		{
			duDebugDrawNavMeshWithClosedList(&debugDraw, *navMesh, *navQuery, navMeshDrawFlags);
		}
		if (m_drawMode == DrawMode::NAVMESH_BVTREE)
		{
			duDebugDrawNavMeshBVTree(&debugDraw, *navMesh);
		}
		if (m_drawMode == DrawMode::NAVMESH_NODES)
		{
			duDebugDrawNavMeshNodes(&debugDraw, *navQuery);
		}
		duDebugDrawNavMeshPolysWithFlags(&debugDraw, *navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
	}

	glDepthMask(GL_TRUE);

	if (m_compactHeightfield && m_drawMode == DrawMode::COMPACT)
	{
		duDebugDrawCompactHeightfieldSolid(&debugDraw, *m_compactHeightfield);
	}

	if (m_compactHeightfield && m_drawMode == DrawMode::COMPACT_DISTANCE)
	{
		duDebugDrawCompactHeightfieldDistance(&debugDraw, *m_compactHeightfield);
	}
	if (m_compactHeightfield && m_drawMode == DrawMode::COMPACT_REGIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&debugDraw, *m_compactHeightfield);
	}
	if (m_heightfield && m_drawMode == DrawMode::VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&debugDraw, *m_heightfield);
		glDisable(GL_FOG);
	}
	if (m_heightfield && m_drawMode == DrawMode::VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&debugDraw, *m_heightfield);
		glDisable(GL_FOG);
	}
	if (m_contourSet && m_drawMode == DrawMode::RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_contourSet && m_drawMode == DrawMode::BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&debugDraw, *m_contourSet, 0.5f);
		duDebugDrawContours(&debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_contourSet && m_drawMode == DrawMode::CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_compactHeightfield && m_contourSet && m_drawMode == DrawMode::REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&debugDraw, *m_compactHeightfield);

		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&debugDraw, *m_contourSet);
		glDepthMask(GL_TRUE);
	}
	if (m_polyMesh && m_drawMode == DrawMode::POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&debugDraw, *m_polyMesh);
		glDepthMask(GL_TRUE);
	}
	if (m_detailMesh && m_drawMode == DrawMode::POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&debugDraw, *m_detailMesh);
		glDepthMask(GL_TRUE);
	}

	inputGeometry->drawConvexVolumes(&debugDraw);

	if (tool)
	{
		tool->handleRender();
	}
	renderToolStates();

	glDepthMask(GL_TRUE);
}

void Sample_SoloMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
	if (tool)
	{
		tool->handleRenderOverlay(proj, model, view);
	}
	renderOverlayToolStates(proj, model, view);
}

void Sample_SoloMesh::handleMeshChanged(InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

	dtFreeNavMesh(navMesh); navMesh = 0;

	if (tool)
	{
		tool->reset();
		tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

bool Sample_SoloMesh::handleBuild()
{
	if (!inputGeometry || !inputGeometry->getMesh())
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}

	cleanup();

	const float* boundsMin = inputGeometry->getNavMeshBoundsMin();
	const float* boundsMax = inputGeometry->getNavMeshBoundsMax();
	const float* verts = inputGeometry->getMesh()->getVerts();
	const int numVerts = inputGeometry->getMesh()->getVertCount();
	const int* tris = inputGeometry->getMesh()->getTris();
	const int numTris = inputGeometry->getMesh()->getTriCount();

	//
	// Step 1. Initialize build config.
	//

	// Init build configuration from GUI
	memset(&m_config, 0, sizeof(m_config));
	m_config.cs = cellSize;
	m_config.ch = cellHeight;
	m_config.walkableSlopeAngle = agentMaxSlope;
	m_config.walkableHeight = static_cast<int>(ceilf(agentHeight / m_config.ch));
	m_config.walkableClimb = static_cast<int>(floorf(agentMaxClimb / m_config.ch));
	m_config.walkableRadius = static_cast<int>(ceilf(agentRadius / m_config.cs));
	m_config.maxEdgeLen = static_cast<int>(edgeMaxLen / cellSize);
	m_config.maxSimplificationError = edgeMaxError;
	m_config.minRegionArea = static_cast<int>(rcSqr(regionMinSize));		// Note: area = size*size
	m_config.mergeRegionArea = static_cast<int>(rcSqr(regionMergeSize));	// Note: area = size*size
	m_config.maxVertsPerPoly = static_cast<int>(vertsPerPoly);
	m_config.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
	m_config.detailSampleMaxError = cellHeight * detailSampleMaxError;

	// Set the area where the navigation will be built.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by a user defined box, etc.
	rcVcopy(m_config.bmin, boundsMin);
	rcVcopy(m_config.bmax, boundsMax);
	rcCalcGridSize(m_config.bmin, m_config.bmax, m_config.cs, &m_config.width, &m_config.height);

	// Reset build times gathering.
	buildContext->resetTimers();
	buildContext->startTimer(RC_TIMER_TOTAL);

	// Start the build process.
	buildContext->log(RC_LOG_PROGRESS, "Building navigation:");
	buildContext->log(RC_LOG_PROGRESS, " - %d x %d cells", m_config.width, m_config.height);
	buildContext->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", static_cast<float>(numVerts) / 1000.0f, static_cast<float>(numTris) / 1000.0f);

	//
	// Step 2. Rasterize input meshes.
	//

	// Allocate voxel heightfield where we will store our rasterized input data.
	m_heightfield = rcAllocHeightfield();
	if (!m_heightfield)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_heightfield'.");
		return false;
	}
	if (!rcCreateHeightfield(buildContext, *m_heightfield, m_config.width, m_config.height, m_config.bmin, m_config.bmax, m_config.cs, m_config.ch))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}

	// Allocate array that can hold triangle area types.
	// This is used to store terrain type information and to mark
	// triangles as unwalkable.
	// If you have multiple meshes you need to process, allocate
	// an array which can hold the max number of triangles you need to process.
	m_triareas = new unsigned char[numTris];
	if (!m_triareas)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", numTris);
		return false;
	}
	memset(m_triareas, 0, numTris * sizeof(unsigned char));

	// Record which triangles in the input mesh are walkable.
	// This information is recorded in m_triareas
	rcMarkWalkableTriangles(buildContext, m_config.walkableSlopeAngle, verts, numVerts, tris, numTris, m_triareas);

	// Rasterize the input mesh
	// If your have multiple meshes, you can transform them, calculate the
	// terrain type for each mesh and rasterize them here.
	if (!rcRasterizeTriangles(buildContext, verts, numVerts, tris, m_triareas, numTris, *m_heightfield, m_config.walkableClimb))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return false;
	}

	//
	// Step 3. Filter walkable surfaces.
	//

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as spans where the character cannot possibly stand.
	if (filterLowHangingObstacles)
	{
		rcFilterLowHangingWalkableObstacles(buildContext, m_config.walkableClimb, *m_heightfield);
	}
	if (filterLedgeSpans)
	{
		rcFilterLedgeSpans(buildContext, m_config.walkableHeight, m_config.walkableClimb, *m_heightfield);
	}
	if (filterWalkableLowHeightSpans)
	{
		rcFilterWalkableLowHeightSpans(buildContext, m_config.walkableHeight, *m_heightfield);
	}

	//
	// Step 4. Partition walkable surface into simple regions.
	//

	// Compact the heightfield so that it is faster to work with.
	// This will result more cache coherent data.  This step will also
	// generate neighbor connection information between walkable cells.
	m_compactHeightfield = rcAllocCompactHeightfield();
	if (!m_compactHeightfield)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(buildContext, m_config.walkableHeight, m_config.walkableClimb, *m_heightfield, *m_compactHeightfield))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}

	// Erode the walkable area by agent radius.
	// This allows us to path an agent through the navmesh as if it was a single point
	if (!rcErodeWalkableArea(buildContext, m_config.walkableRadius, *m_compactHeightfield))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return false;
	}

	// (Optional) Marks the surface type of voxels in an area defined by a convex volume.
	// Useful to mark areas of differing cost.
	const ConvexVolume* vols = inputGeometry->getConvexVolumes();
	for (int i  = 0; i < inputGeometry->getConvexVolumeCount(); ++i)
	{
		rcMarkConvexPolyArea(buildContext, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_compactHeightfield);
	}

	// Partition the heightfield into contiguous regions that will each be
	// triangulated into navigation polygons.
	//
	// There are 3 partitioning methods, each with their own pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - the are some corner cases where this method creates holes and
	//     overlaps in the resulting region data.
	//      - holes may appear when a small obstacle is close to a large open
	//        area.  This will not cause triangulation to fail.
	//      - overlaps may occur if you have narrow spiral corridors
	//        e.g. spiral stairs.  This will cause triangulation to fail.
	//   * Generally the best choice if you are precompute the navmesh and/or
	//     there are large open areas in the input geometry.
	// 2) Monotone partitioning
	//   - fastest
	//   - guaranteed to partition the heightfield into regions without holes
	//     or overlaps
	//   - Can create long, thin polygons which sometimes cause paths with detours
	//   * Use this if you want fast navmesh generation
	// 3) Layer partitioning
	//   - quite fast
	//   - partitions the heighfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes, which makes
	//     this slower than monotone partitioning
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a slightly ugly tessellation (still better
	//     than monotone) if you have large open areas with small obstacles.
	//     This is less of a problem if you use a tiled navmesh.
	//   * A good choice for a tiled navmesh with small to medium-sized tiles

	if (partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(buildContext, *m_compactHeightfield))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return false;
		}

		// Partition the walkable surface into contiguous regions.
		if (!rcBuildRegions(buildContext, *m_compactHeightfield, 0, m_config.minRegionArea, m_config.mergeRegionArea))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return false;
		}
	}
	else if (partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into contiguous regions.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(buildContext, *m_compactHeightfield, 0, m_config.minRegionArea, m_config.mergeRegionArea))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return false;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into contiguous regions.
		// Layer partitioning does not need distancefield.
		if (!rcBuildLayerRegions(buildContext, *m_compactHeightfield, 0, m_config.minRegionArea))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return false;
		}
	}

	//
	// Step 5. Trace and simplify region contours.
	//

	// Create contour.
	m_contourSet = rcAllocContourSet();
	if (!m_contourSet)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(buildContext, *m_compactHeightfield, m_config.maxSimplificationError, m_config.maxEdgeLen, *m_contourSet))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}

	//
	// Step 6. Triangulate contours to build navmesh polygons.
	//

	m_polyMesh = rcAllocPolyMesh();
	if (!m_polyMesh)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	if (!rcBuildPolyMesh(buildContext, *m_contourSet, m_config.maxVertsPerPoly, *m_polyMesh))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}

	//
	// Step 7. Create a navmesh from the triangulated polygons.
	//
	// Calculates additional information necessary to run pathing queries.
	//

	m_detailMesh = rcAllocPolyMeshDetail();
	if (!m_detailMesh)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}
	if (!rcBuildPolyMeshDetail(buildContext, *m_polyMesh, *m_compactHeightfield, m_config.detailSampleDist, m_config.detailSampleMaxError, *m_detailMesh))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return false;
	}

	// At this point the navigation mesh data is ready to use.
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access
	// the navmesh data.

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
		params.offMeshConVerts = inputGeometry->getOffMeshConnectionVerts();
		params.offMeshConRad = inputGeometry->getOffMeshConnectionRads();
		params.offMeshConDir = inputGeometry->getOffMeshConnectionDirs();
		params.offMeshConAreas = inputGeometry->getOffMeshConnectionAreas();
		params.offMeshConFlags = inputGeometry->getOffMeshConnectionFlags();
		params.offMeshConUserID = inputGeometry->getOffMeshConnectionId();
		params.offMeshConCount = inputGeometry->getOffMeshConnectionCount();
		params.walkableHeight = agentHeight;
		params.walkableRadius = agentRadius;
		params.walkableClimb = agentMaxClimb;
		rcVcopy(params.bmin, m_polyMesh->bmin);
		rcVcopy(params.bmax, m_polyMesh->bmax);
		params.cs = m_config.cs;
		params.ch = m_config.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			buildContext->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}

		navMesh = dtAllocNavMesh();
		if (!navMesh)
		{
			dtFree(navData);
			buildContext->log(RC_LOG_ERROR, "Could not create Detour navmesh");
			return false;
		}

		dtStatus status = navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			buildContext->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return false;
		}

		status = navQuery->init(navMesh, 2048);
		if (dtStatusFailed(status))
		{
			buildContext->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
			return false;
		}
	}

	// Stop build timers
	buildContext->stopTimer(RC_TIMER_TOTAL);
	auto totalTime = buildContext->getAccumulatedTime(RC_TIMER_TOTAL);
	m_totalBuildTimeMs = static_cast<float>(totalTime) / 1000.0f;

	// Show performance stats.
	duLogBuildTimes(*buildContext, totalTime);
	buildContext->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_polyMesh->nverts, m_polyMesh->npolys);

	if (tool)
	{
		tool->init(this);
	}
	initToolStates(this);

	return true;
}
