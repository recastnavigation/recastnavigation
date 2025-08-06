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

#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "InputGeom.h"
#include "NavMeshPruneTool.h"
#include "NavMeshTesterTool.h"
#include "OffMeshConnectionTool.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "RecastDump.h"
#include "SDL_opengl.h"
#include "Sample.h"

#include <imgui.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef WIN32
#	define snprintf _snprintf
#endif

const char* Sample_SoloMesh::drawModeNames[]{
	"Input Mesh",
	"Navmesh",
	"Navmesh Invis",
	"Navmesh Trans",
	"Navmesh BVTree",
	"Navmesh Nodes",
	"Voxels",
	"Walkable Voxels",
	"Compact",
	"Compact Distance",
	"Compact Regions",
	"Region Connections",
	"Raw Contours",
	"Both Contours",
	"Contours",
	"Poly Mesh",
	"Poly Mesh Detail"};

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
	delete[] triareas;
	triareas = nullptr;
	rcFreeHeightField(heightfield);
	heightfield = nullptr;
	rcFreeCompactHeightfield(compactHeightfield);
	compactHeightfield = nullptr;
	rcFreeContourSet(contourSet);
	contourSet = nullptr;
	rcFreePolyMesh(polyMesh);
	polyMesh = nullptr;
	rcFreePolyMeshDetail(detailMesh);
	detailMesh = nullptr;
	dtFreeNavMesh(navMesh);
	navMesh = nullptr;
}

void Sample_SoloMesh::drawSettingsUI()
{
	static const char* fileName = "solo_navmesh.bin";

	drawCommonSettingsUI();

	ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
	if (ImGui::Button("Save"))
	{
		saveAll(fileName, navMesh);
	}

	if (ImGui::Button("Load"))
	{
		dtFreeNavMesh(navMesh);
		navMesh = loadAll(fileName);
		navQuery->init(navMesh, 2048);
	}
	ImGui::PopItemWidth();

	ImGui::Text("Build Time: %.1fms", totalBuildTimeMs);
}

void Sample_SoloMesh::drawToolsUI()
{
	const SampleToolType currentType = !tool ? SampleToolType::NONE : tool->type();

#define TOOL(toolType, toolClass)                                      \
	if (ImGui::RadioButton(                                            \
			toolNames[static_cast<uint8_t>(SampleToolType::toolType)], \
			currentType == SampleToolType::toolType))                  \
	{                                                                  \
		setTool(new toolClass{});                                      \
	}
	TOOL(NAVMESH_TESTER, NavMeshTesterTool)
	TOOL(NAVMESH_PRUNE, NavMeshPruneTool)
	TOOL(OFFMESH_CONNECTION, OffMeshConnectionTool)
	TOOL(CONVEX_VOLUME, ConvexVolumeTool)
	TOOL(CROWD, CrowdTool)
#undef TOOL

	ImGui::Separator();

	if (tool)
	{
		tool->handleMenu();
	}
}

void Sample_SoloMesh::UI_DrawModeOption(const DrawMode drawMode, const bool enabled)
{
	ImGui::BeginDisabled(!enabled);
	const bool is_selected = currentDrawMode == drawMode;
	if (ImGui::Selectable(drawModeNames[static_cast<int>(drawMode)], is_selected))
	{
		currentDrawMode = drawMode;
	}

	// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
	if (is_selected)
	{
		ImGui::SetItemDefaultFocus();
	}
	ImGui::EndDisabled();
}

void Sample_SoloMesh::drawDebugUI()
{
	ImGui::Text("Draw");
	if (ImGui::BeginCombo("##drawMode", drawModeNames[static_cast<int>(currentDrawMode)], 0))
	{
		UI_DrawModeOption(DrawMode::MESH, true);
		UI_DrawModeOption(DrawMode::NAVMESH, navMesh != nullptr);
		UI_DrawModeOption(DrawMode::NAVMESH_INVIS, navMesh != nullptr);
		UI_DrawModeOption(DrawMode::NAVMESH_TRANS, navMesh != nullptr);
		UI_DrawModeOption(DrawMode::NAVMESH_BVTREE, navMesh != nullptr);
		UI_DrawModeOption(DrawMode::NAVMESH_NODES, navQuery != nullptr);
		UI_DrawModeOption(DrawMode::VOXELS, heightfield != nullptr);
		UI_DrawModeOption(DrawMode::VOXELS_WALKABLE, heightfield != nullptr);
		UI_DrawModeOption(DrawMode::COMPACT, compactHeightfield != nullptr);
		UI_DrawModeOption(DrawMode::COMPACT_DISTANCE, compactHeightfield != nullptr);
		UI_DrawModeOption(DrawMode::COMPACT_REGIONS, compactHeightfield != nullptr);
		UI_DrawModeOption(DrawMode::REGION_CONNECTIONS, contourSet != nullptr);
		UI_DrawModeOption(DrawMode::RAW_CONTOURS, contourSet != nullptr);
		UI_DrawModeOption(DrawMode::BOTH_CONTOURS, contourSet != nullptr);
		UI_DrawModeOption(DrawMode::CONTOURS, contourSet != nullptr);
		UI_DrawModeOption(DrawMode::POLYMESH, polyMesh != nullptr);
		UI_DrawModeOption(DrawMode::POLYMESH_DETAIL, detailMesh != nullptr);
		ImGui::EndCombo();
	}
}

void Sample_SoloMesh::render()
{
	if (!inputGeometry)
	{
		return;
	}

	glEnable(GL_FOG);
	glDepthMask(GL_TRUE);

	const float texScale = 1.0f / (cellSize * 10.0f);

	if (currentDrawMode != DrawMode::NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(
			&debugDraw,
			inputGeometry->mesh.verts.data(),
			inputGeometry->mesh.getVertCount(),
			inputGeometry->mesh.tris.data(),
			inputGeometry->mesh.normals.data(),
			inputGeometry->mesh.getTriCount(),
			agentMaxSlope,
			texScale);
		inputGeometry->drawOffMeshConnections(&debugDraw);
	}

	glDisable(GL_FOG);
	glDepthMask(GL_FALSE);

	// Draw bounds
	const float* navmeshBoundsMin = inputGeometry->getNavMeshBoundsMin();
	const float* navmeshBoundsMax = inputGeometry->getNavMeshBoundsMax();
	duDebugDrawBoxWire(
		&debugDraw,
		navmeshBoundsMin[0],
		navmeshBoundsMin[1],
		navmeshBoundsMin[2],
		navmeshBoundsMax[0],
		navmeshBoundsMax[1],
		navmeshBoundsMax[2],
		duRGBA(255, 255, 255, 128),
		1.0f);
	debugDraw.begin(DU_DRAW_POINTS, 5.0f);
	debugDraw.vertex(navmeshBoundsMin[0], navmeshBoundsMin[1], navmeshBoundsMin[2], duRGBA(255, 255, 255, 128));
	debugDraw.end();

	if (navMesh && navQuery &&
	    (currentDrawMode == DrawMode::NAVMESH || currentDrawMode == DrawMode::NAVMESH_TRANS ||
	     currentDrawMode == DrawMode::NAVMESH_BVTREE || currentDrawMode == DrawMode::NAVMESH_NODES ||
	     currentDrawMode == DrawMode::NAVMESH_INVIS))
	{
		if (currentDrawMode != DrawMode::NAVMESH_INVIS)
		{
			duDebugDrawNavMeshWithClosedList(&debugDraw, *navMesh, *navQuery, navMeshDrawFlags);
		}
		if (currentDrawMode == DrawMode::NAVMESH_BVTREE)
		{
			duDebugDrawNavMeshBVTree(&debugDraw, *navMesh);
		}
		if (currentDrawMode == DrawMode::NAVMESH_NODES)
		{
			duDebugDrawNavMeshNodes(&debugDraw, *navQuery);
		}
		duDebugDrawNavMeshPolysWithFlags(&debugDraw, *navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0, 0, 0, 128));
	}

	glDepthMask(GL_TRUE);

	if (compactHeightfield && currentDrawMode == DrawMode::COMPACT)
	{
		duDebugDrawCompactHeightfieldSolid(&debugDraw, *compactHeightfield);
	}

	if (compactHeightfield && currentDrawMode == DrawMode::COMPACT_DISTANCE)
	{
		duDebugDrawCompactHeightfieldDistance(&debugDraw, *compactHeightfield);
	}
	if (compactHeightfield && currentDrawMode == DrawMode::COMPACT_REGIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&debugDraw, *compactHeightfield);
	}
	if (heightfield && currentDrawMode == DrawMode::VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&debugDraw, *heightfield);
		glDisable(GL_FOG);
	}
	if (heightfield && currentDrawMode == DrawMode::VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&debugDraw, *heightfield);
		glDisable(GL_FOG);
	}
	if (contourSet && currentDrawMode == DrawMode::RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&debugDraw, *contourSet);
		glDepthMask(GL_TRUE);
	}
	if (contourSet && currentDrawMode == DrawMode::BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&debugDraw, *contourSet, 0.5f);
		duDebugDrawContours(&debugDraw, *contourSet);
		glDepthMask(GL_TRUE);
	}
	if (contourSet && currentDrawMode == DrawMode::CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&debugDraw, *contourSet);
		glDepthMask(GL_TRUE);
	}
	if (compactHeightfield && contourSet && currentDrawMode == DrawMode::REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&debugDraw, *compactHeightfield);

		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&debugDraw, *contourSet);
		glDepthMask(GL_TRUE);
	}
	if (polyMesh && currentDrawMode == DrawMode::POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&debugDraw, *polyMesh);
		glDepthMask(GL_TRUE);
	}
	if (detailMesh && currentDrawMode == DrawMode::POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&debugDraw, *detailMesh);
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

void Sample_SoloMesh::renderOverlay(double* proj, double* model, int* view)
{
	if (tool)
	{
		tool->handleRenderOverlay(proj, model, view);
	}
	renderOverlayToolStates(proj, model, view);
}

void Sample_SoloMesh::onMeshChanged(InputGeom* geom)
{
	Sample::onMeshChanged(geom);

	dtFreeNavMesh(navMesh);
	navMesh = nullptr;

	if (tool)
	{
		tool->reset();
		tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

bool Sample_SoloMesh::build()
{
	if (!inputGeometry || inputGeometry->mesh.verts.empty())
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}

	cleanup();

	const float* boundsMin = inputGeometry->getNavMeshBoundsMin();
	const float* boundsMax = inputGeometry->getNavMeshBoundsMax();
	const float* verts = inputGeometry->mesh.verts.data();
	const int numVerts = static_cast<int>(inputGeometry->mesh.verts.size()) / 3;
	const int* tris = inputGeometry->mesh.tris.data();
	const int numTris = static_cast<int>(inputGeometry->mesh.tris.size()) / 3;

	//
	// Step 1. Initialize build config.
	//

	// Init build configuration from GUI
	memset(&config, 0, sizeof(config));
	config.cs = cellSize;
	config.ch = cellHeight;
	config.walkableSlopeAngle = agentMaxSlope;
	config.walkableHeight = static_cast<int>(ceilf(agentHeight / config.ch));
	config.walkableClimb = static_cast<int>(floorf(agentMaxClimb / config.ch));
	config.walkableRadius = static_cast<int>(ceilf(agentRadius / config.cs));
	config.maxEdgeLen = static_cast<int>(edgeMaxLen / cellSize);
	config.maxSimplificationError = edgeMaxError;
	config.minRegionArea = static_cast<int>(rcSqr(regionMinSize));      // Note: area = size*size
	config.mergeRegionArea = static_cast<int>(rcSqr(regionMergeSize));  // Note: area = size*size
	config.maxVertsPerPoly = vertsPerPoly;
	config.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
	config.detailSampleMaxError = cellHeight * detailSampleMaxError;

	// Set the area where the navigation will be built.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by a user defined box, etc.
	rcVcopy(config.bmin, boundsMin);
	rcVcopy(config.bmax, boundsMax);
	rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

	// Reset build times gathering.
	buildContext->resetTimers();
	buildContext->startTimer(RC_TIMER_TOTAL);

	// Start the build process.
	buildContext->log(RC_LOG_PROGRESS, "Building navigation:");
	buildContext->log(RC_LOG_PROGRESS, " - %d x %d cells", config.width, config.height);
	buildContext->log(
		RC_LOG_PROGRESS,
		" - %.1fK verts, %.1fK tris",
		static_cast<float>(numVerts) / 1000.0f,
		static_cast<float>(numTris) / 1000.0f);

	//
	// Step 2. Rasterize input meshes.
	//

	// Allocate voxel heightfield where we will store our rasterized input data.
	heightfield = rcAllocHeightfield();
	if (!heightfield)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'heightfield'.");
		return false;
	}
	if (!rcCreateHeightfield(
			buildContext,
			*heightfield,
			config.width,
			config.height,
			config.bmin,
			config.bmax,
			config.cs,
			config.ch))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}

	// Allocate array that can hold triangle area types.
	// This is used to store terrain type information and to mark
	// triangles as unwalkable.
	// If you have multiple meshes you need to process, allocate
	// an array which can hold the max number of triangles you need to process.
	triareas = new unsigned char[numTris];
	if (!triareas)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", numTris);
		return false;
	}
	memset(triareas, 0, numTris * sizeof(unsigned char));

	// Record which triangles in the input mesh are walkable.
	// This information is recorded in m_triareas
	rcMarkWalkableTriangles(buildContext, config.walkableSlopeAngle, verts, numVerts, tris, numTris, triareas);

	// Rasterize the input mesh
	// If your have multiple meshes, you can transform them, calculate the
	// terrain type for each mesh and rasterize them here.
	if (!rcRasterizeTriangles(buildContext, verts, numVerts, tris, triareas, numTris, *heightfield, config.walkableClimb))
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
		rcFilterLowHangingWalkableObstacles(buildContext, config.walkableClimb, *heightfield);
	}
	if (filterLedgeSpans)
	{
		rcFilterLedgeSpans(buildContext, config.walkableHeight, config.walkableClimb, *heightfield);
	}
	if (filterWalkableLowHeightSpans)
	{
		rcFilterWalkableLowHeightSpans(buildContext, config.walkableHeight, *heightfield);
	}

	//
	// Step 4. Partition walkable surface into simple regions.
	//

	// Compact the heightfield so that it is faster to work with.
	// This will result more cache coherent data.  This step will also
	// generate neighbor connection information between walkable cells.
	compactHeightfield = rcAllocCompactHeightfield();
	if (!compactHeightfield)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(
			buildContext,
			config.walkableHeight,
			config.walkableClimb,
			*heightfield,
			*compactHeightfield))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}

	// Erode the walkable area by agent radius.
	// This allows us to path an agent through the navmesh as if it was a single point
	if (!rcErodeWalkableArea(buildContext, config.walkableRadius, *compactHeightfield))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return false;
	}

	// (Optional) Marks the surface type of voxels in an area defined by a convex volume.
	// Useful to mark areas of differing cost.
	const ConvexVolume* vols = inputGeometry->convexVolumes;
	for (int i = 0; i < inputGeometry->convexVolumeCount; ++i)
	{
		rcMarkConvexPolyArea(
			buildContext,
			vols[i].verts,
			vols[i].nverts,
			vols[i].hmin,
			vols[i].hmax,
			(unsigned char)vols[i].area,
			*compactHeightfield);
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
		if (!rcBuildDistanceField(buildContext, *compactHeightfield))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return false;
		}

		// Partition the walkable surface into contiguous regions.
		if (!rcBuildRegions(buildContext, *compactHeightfield, 0, config.minRegionArea, config.mergeRegionArea))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return false;
		}
	}
	else if (partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into contiguous regions.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(buildContext, *compactHeightfield, 0, config.minRegionArea, config.mergeRegionArea))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return false;
		}
	}
	else  // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into contiguous regions.
		// Layer partitioning does not need distancefield.
		if (!rcBuildLayerRegions(buildContext, *compactHeightfield, 0, config.minRegionArea))
		{
			buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return false;
		}
	}

	//
	// Step 5. Trace and simplify region contours.
	//

	// Create contour.
	contourSet = rcAllocContourSet();
	if (!contourSet)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(buildContext, *compactHeightfield, config.maxSimplificationError, config.maxEdgeLen, *contourSet))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}

	//
	// Step 6. Triangulate contours to build navmesh polygons.
	//

	polyMesh = rcAllocPolyMesh();
	if (!polyMesh)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	if (!rcBuildPolyMesh(buildContext, *contourSet, config.maxVertsPerPoly, *polyMesh))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}

	//
	// Step 7. Create a navmesh from the triangulated polygons.
	//
	// Calculates additional information necessary to run pathing queries.
	//

	detailMesh = rcAllocPolyMeshDetail();
	if (!detailMesh)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}
	if (!rcBuildPolyMeshDetail(
			buildContext,
			*polyMesh,
			*compactHeightfield,
			config.detailSampleDist,
			config.detailSampleMaxError,
			*detailMesh))
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
	if (config.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

		// Update poly flags from areas.
		for (int i = 0; i < polyMesh->npolys; ++i)
		{
			if (polyMesh->areas[i] == RC_WALKABLE_AREA)
			{
				polyMesh->areas[i] = SAMPLE_POLYAREA_GROUND;
			}

			if (polyMesh->areas[i] == SAMPLE_POLYAREA_GROUND || polyMesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
			    polyMesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				polyMesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (polyMesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				polyMesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (polyMesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				polyMesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = polyMesh->verts;
		params.vertCount = polyMesh->nverts;
		params.polys = polyMesh->polys;
		params.polyAreas = polyMesh->areas;
		params.polyFlags = polyMesh->flags;
		params.polyCount = polyMesh->npolys;
		params.nvp = polyMesh->nvp;
		params.detailMeshes = detailMesh->meshes;
		params.detailVerts = detailMesh->verts;
		params.detailVertsCount = detailMesh->nverts;
		params.detailTris = detailMesh->tris;
		params.detailTriCount = detailMesh->ntris;
		params.offMeshConVerts = inputGeometry->offMeshConVerts;
		params.offMeshConRad = inputGeometry->offMeshConRads;
		params.offMeshConDir = inputGeometry->offMeshConDirs;
		params.offMeshConAreas = inputGeometry->offMeshConAreas;
		params.offMeshConFlags = inputGeometry->offMeshConFlags;
		params.offMeshConUserID = inputGeometry->offMeshConId;
		params.offMeshConCount = inputGeometry->offMeshConCount;
		params.walkableHeight = agentHeight;
		params.walkableRadius = agentRadius;
		params.walkableClimb = agentMaxClimb;
		rcVcopy(params.bmin, polyMesh->bmin);
		rcVcopy(params.bmax, polyMesh->bmax);
		params.cs = config.cs;
		params.ch = config.ch;
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
	totalBuildTimeMs = static_cast<float>(totalTime) / 1000.0f;

	// Show performance stats.
	duLogBuildTimes(*buildContext, totalTime);
	buildContext->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", polyMesh->nverts, polyMesh->npolys);

	if (tool)
	{
		tool->init(this);
	}
	initToolStates(this);

	return true;
}
