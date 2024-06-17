//
// Created by joran on 15/12/2023.
//


#include <cstring>
#include <new>

#include <InputGeom.h>
#include <Recast.h>
#include <RecastDump.h>

#include "MeshLoaderObj.h"
#include "Generators.h"

#include <new>

bool generateTheses(rcContext &context, const InputGeom &pGeom, rcConfig &config, const bool filterLowHangingObstacles, const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans, rcPolyMesh *&pMesh, rcPolyMeshDetail *&pDetailedMesh, int *&bounderies, int &bounderyElementCount) {
  if (!pGeom.getMesh()) {
    context.log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
    return false;
  }

  const float *bmin = pGeom.getNavMeshBoundsMin();
  const float *bmax = pGeom.getNavMeshBoundsMax();
  const float *verts = pGeom.getMesh()->getVerts();
  const int nverts = pGeom.getMesh()->getVertCount();
  const int *tris = pGeom.getMesh()->getTris();
  const int ntris = pGeom.getMesh()->getTriCount();

  // Set the area where the navigation will be build.
  // Here the bounds of the input mesh are used, but the
  // area could be specified by an user defined box, etc.
  rcVcopy(config.bmin, bmin);
  rcVcopy(config.bmax, bmax);
  rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

  // Reset build times gathering.
  context.resetTimers();

  // Start the build process.
  context.startTimer(RC_TIMER_TOTAL);

  context.log(RC_LOG_PROGRESS, "Building navigation:");
  context.log(RC_LOG_PROGRESS, " - %d x %d cells", config.width, config.height);
  context.log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", static_cast<float>(nverts) / 1000.0f,static_cast<float>(ntris) / 1000.0f);

  //
  // Step 2. Rasterize input polygon soup.
  //

  // Allocate voxel heightfield where we rasterize our input data to.
  rcHeightfield *m_solid = rcAllocHeightfield();
  if (!m_solid) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
    return false;
  }
  if (!rcCreateHeightfield(&context, *m_solid, config.width, config.height, config.bmin, config.bmax, config.cs,config.ch)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
    return false;
  }

  // Allocate array that can hold triangle area types.
  // If you have multiple meshes you need to process, allocate
  // and array which can hold the max number of triangles you need to process.
  auto *m_triareas = new(std::nothrow) unsigned char[ntris];
  if (!m_triareas) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
    return false;
  }

  // Find triangles which are walkable based on their slope and rasterize them.
  // If your input data is multiple meshes, you can transform them here, calculate
  // the are type for each of the meshes and rasterize them.
  std::memset(m_triareas, 0, ntris * sizeof(unsigned char));
  rcMarkWalkableTriangles(&context, config.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
  if (!rcRasterizeTriangles(&context, verts, nverts, tris, m_triareas, ntris, *m_solid, config.walkableClimb)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
    return false;
  }

  const bool m_keepInterResults = false;
  if  (!m_keepInterResults) {
    delete [] m_triareas;
  }

  //
  // Step 3. Filter walkable surfaces.
  //

  // Once all geometry is rasterized, we do initial pass of filtering to
  // remove unwanted overhangs caused by the conservative rasterization
  // as well as filter spans where the character cannot possibly stand.
  if (filterLowHangingObstacles)
    rcFilterLowHangingWalkableObstacles(&context, config.walkableClimb, *m_solid);
  if (filterLedgeSpans)
    rcFilterLedgeSpans(&context, config.walkableHeight, config.walkableClimb, *m_solid);
  if (filterWalkableLowHeightSpans)
    rcFilterWalkableLowHeightSpans(&context, config.walkableHeight, *m_solid);

  //
  // Step 4. Partition walkable surface to simple regions.
  //

  // Compact the heightfield so that it is faster to handle from now on.
  // This will result more cache coherent data as well as the neighbours
  // between walkable cells will be calculated.
  rcCompactHeightfield *m_chf = rcAllocCompactHeightfield();
  if (!m_chf) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
    return false;
  }
  if (!rcBuildCompactHeightfield(&context, config.walkableHeight, config.walkableClimb, *m_solid, *m_chf)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
    return false;
  }

  if  (!m_keepInterResults) {
    rcFreeHeightField(m_solid);
  }

  // Erode the walkable area by agent radius.
  if (!rcErodeWalkableArea(&context, config.walkableRadius, *m_chf)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
    return false;
  }

  // (Optional) Mark areas.
  const ConvexVolume *vols = pGeom.getConvexVolumes();
  for (int i = 0; i < pGeom.getConvexVolumeCount(); ++i)
    rcMarkConvexPolyArea(&context, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, static_cast<unsigned char>(vols[i].area), *m_chf);

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

  // Prepare for region partitioning, by calculating distance field along the walkable surface.
  if (!rcBuildDistanceField(&context, *m_chf)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
    return false;
  }

  // Partition the walkable surface into simple regions without holes.
  if (!rcBuildRegionsLCM(&context, *m_chf, 0, config.minRegionArea, config.mergeRegionArea)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
    return false;
  }

  //
  // Step 5. Trace and simplify region contours.
  //

  // Create contours.
  rcContourSet *m_cset = rcAllocContourSet();
  if (!m_cset) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
    return false;
  }
  if (!rcBuildContoursLCM(&context, *m_chf, config.maxSimplificationError, config.maxEdgeLen, *m_cset, bounderies, bounderyElementCount)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
    return false;
  }

  //
  // Step 6. Build polygons mesh from contours.
  //

  // Build polygon navmesh from the contours.
  pMesh = rcAllocPolyMesh();
  if (!pMesh) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
    return false;
  }
  if (!rcBuildPolyMesh(&context, *m_cset, config.maxVertsPerPoly, *pMesh)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
    return false;
  }

  //
  // Step 7. Create detail mesh which allows to access approximate height on each polygon.
  //

  pDetailedMesh = rcAllocPolyMeshDetail();
  if (!pDetailedMesh) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
    return false;
  }

  if (!rcBuildPolyMeshDetail(&context, *pMesh, *m_chf, config.detailSampleDist, config.detailSampleMaxError, *pDetailedMesh)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
    return false;
  }

  if  (!m_keepInterResults) {
    rcFreeCompactHeightfield(m_chf);
    rcFreeContourSet(m_cset);
  }

  context.stopTimer(RC_TIMER_TOTAL);

  // Show performance stats.
  duLogBuildTimes(context, context.getAccumulatedTime(RC_TIMER_TOTAL));
  context.log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", pMesh->nverts, pMesh->npolys);

  return true;
}

bool generateSingle(rcContext& context, const InputGeom& pGeom, rcConfig& config, const bool filterLowHangingObstacles, const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans, rcPolyMesh*& pMesh, rcPolyMeshDetail*& pDetailedMesh) {
  if ( !pGeom.getMesh()) {
    context.log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
    return false;
  }

  const float *bmin = pGeom.getNavMeshBoundsMin();
  const float *bmax = pGeom.getNavMeshBoundsMax();
  const float *verts = pGeom.getMesh()->getVerts();
  const int nverts = pGeom.getMesh()->getVertCount();
  const int *tris = pGeom.getMesh()->getTris();
  const int ntris = pGeom.getMesh()->getTriCount();

  //
  // Step 1. Initialize build config.
  //

  // Set the area where the navigation will be build.
  // Here the bounds of the input mesh are used, but the
  // area could be specified by an user defined box, etc.
  rcVcopy(config.bmin, bmin);
  rcVcopy(config.bmax, bmax);
  rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

  // Reset build times gathering.
  context.resetTimers();

  // Start the build process.
  context.startTimer(RC_TIMER_TOTAL);

  context.log(RC_LOG_PROGRESS, "Building navigation:");
  context.log(RC_LOG_PROGRESS, " - %d x %d cells", config.width, config.height);
  context.log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", static_cast<float>(nverts) / 1000.0f,
            static_cast<float>(ntris) / 1000.0f);

  //
  // Step 2. Rasterize input polygon soup.
  //

  // Allocate voxel heightfield where we rasterize our input data to.
rcHeightfield *m_solid = rcAllocHeightfield();
  if (!m_solid) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
    return false;
  }
  if (!rcCreateHeightfield(&context, *m_solid, config.width, config.height, config.bmin, config.bmax, config.cs,
                           config.ch)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
    return false;
  }

  // Allocate array that can hold triangle area types.
  // If you have multiple meshes you need to process, allocate
  // and array which can hold the max number of triangles you need to process.
  auto *triareas = new(std::nothrow) unsigned char[ntris];
  if (!triareas) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
    return false;
  }

  // Find triangles which are walkable based on their slope and rasterize them.
  // If your input data is multiple meshes, you can transform them here, calculate
  // the are type for each of the meshes and rasterize them.
  std::memset(triareas, 0, ntris * sizeof(unsigned char));
  rcMarkWalkableTriangles(&context, config.walkableSlopeAngle, verts, nverts, tris, ntris, triareas);
  if (!rcRasterizeTriangles(&context, verts, nverts, tris, triareas, ntris, *m_solid, config.walkableClimb)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
    return false;
  }

  const bool m_keepInterResults = false;
  if  (!m_keepInterResults) {
    delete [] triareas;
  }

  //
  // Step 3. Filter walkable surfaces.
  //

  // Once all geometry is rasterized, we do initial pass of filtering to
  // remove unwanted overhangs caused by the conservative rasterization
  // as well as filter spans where the character cannot possibly stand.
  if (filterLowHangingObstacles)
    rcFilterLowHangingWalkableObstacles(&context, config.walkableClimb, *m_solid);
  if (filterLedgeSpans)
    rcFilterLedgeSpans(&context, config.walkableHeight, config.walkableClimb, *m_solid);
  if (filterWalkableLowHeightSpans)
    rcFilterWalkableLowHeightSpans(&context, config.walkableHeight, *m_solid);

  //
  // Step 4. Partition walkable surface to simple regions.
  //

  // Compact the heightfield so that it is faster to handle from now on.
  // This will result more cache coherent data as well as the neighbours
  // between walkable cells will be calculated.
  rcCompactHeightfield *compactHeightField = rcAllocCompactHeightfield();
  if (!compactHeightField) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
    return false;
  }
  if (!rcBuildCompactHeightfield(&context, config.walkableHeight, config.walkableClimb, *m_solid, *compactHeightField)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
    return false;
  }

  if  (!m_keepInterResults) {
    rcFreeHeightField(m_solid);
  }

  // Erode the walkable area by agent radius.
  if (!rcErodeWalkableArea(&context, config.walkableRadius, *compactHeightField)
  ) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
    return false;
  }

  // (Optional) Mark areas.
  const ConvexVolume *vols = pGeom.getConvexVolumes();
  for (int i = 0; i < pGeom.getConvexVolumeCount(); ++i)
    rcMarkConvexPolyArea(&context, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, static_cast<unsigned char>(vols[i].area), *compactHeightField);

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

  // Prepare for region partitioning, by calculating distance field along the walkable surface.
  if (!rcBuildDistanceField(&context, *compactHeightField)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
    return false;
  }

  // Partition the walkable surface into simple regions without holes.
  if (!rcBuildRegions(&context, *compactHeightField, 0, config.minRegionArea, config.mergeRegionArea)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
    return false;
  }

  //
  // Step 5. Trace and simplify region contours.
  //

  // Create contours.
  rcContourSet *m_cset = rcAllocContourSet();
  if (!m_cset) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
    return false;
  }
  if (!rcBuildContours(&context, *compactHeightField, config.maxSimplificationError, config.maxEdgeLen, *m_cset)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
    return false;
  }

  //
  // Step 6. Build polygons mesh from contours.
  //

  // Build polygon navmesh from the contours.
  pMesh = rcAllocPolyMesh();
  if (!pMesh) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
    return false;
  }
  if (!rcBuildPolyMesh(&context, *m_cset, config.maxVertsPerPoly, *pMesh)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
    return false;
  }

  //
  // Step 7. Create detail mesh which allows to access approximate height on each polygon.
  //

  pDetailedMesh = rcAllocPolyMeshDetail();
  if (!pDetailedMesh) {
    context.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
    return false;
  }

  if (!rcBuildPolyMeshDetail(&context, *pMesh, *compactHeightField, config.detailSampleDist, config.detailSampleMaxError, *pDetailedMesh)) {
    context.log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
    return false;
  }

  if  (!m_keepInterResults) {
    rcFreeCompactHeightfield(compactHeightField);
    rcFreeContourSet(m_cset);
  }

  // At this point the navigation mesh data is ready, you can access it from m_pmesh.
  // See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

  //
  // (Optional) Step 8. Create Detour data from Recast poly mesh.
  //

  context.stopTimer(RC_TIMER_TOTAL);

  // Show performance stats.
  duLogBuildTimes(context, context.getAccumulatedTime(RC_TIMER_TOTAL));
  context.log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", pMesh->nverts, pMesh->npolys);
  return true;
}
