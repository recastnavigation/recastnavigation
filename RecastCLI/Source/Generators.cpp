//
// Created by joran on 15/12/2023.
//

#include "Generators.h"

#include <cstring>
#include <new>

#include <InputGeom.h>
#include <Recast.h>
#include <RecastDump.h>

#include "MeshLoaderObj.h"

bool GenerateTheses(rcContext *pCtx, const InputGeom *pGeom, rcConfig &config, const bool filterLowHangingObstacles,
                    const bool filterLedgeSpans, const bool filterWalkableLowHeightSpans, float &totalBuildTimeMs,
                    rcPolyMesh *&m_pmesh, rcPolyMeshDetail *&m_dmesh, int *&bounderies, int &bounderyElementCount) {
  if (!pGeom || !pGeom->getMesh()) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
    return false;
  }

  const float *bmin = pGeom->getNavMeshBoundsMin();
  const float *bmax = pGeom->getNavMeshBoundsMax();
  const float *verts = pGeom->getMesh()->getVerts();
  const int nverts = pGeom->getMesh()->getVertCount();
  const int *tris = pGeom->getMesh()->getTris();
  const int ntris = pGeom->getMesh()->getTriCount();

  // Set the area where the navigation will be build.
  // Here the bounds of the input mesh are used, but the
  // area could be specified by an user defined box, etc.
  rcVcopy(config.bmin, bmin);
  rcVcopy(config.bmax, bmax);
  rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

  // Reset build times gathering.
  pCtx->resetTimers();

  // Start the build process.
  pCtx->startTimer(RC_TIMER_TOTAL);

  pCtx->log(RC_LOG_PROGRESS, "Building navigation:");
  pCtx->log(RC_LOG_PROGRESS, " - %d x %d cells", config.width, config.height);
  pCtx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", static_cast<float>(nverts) / 1000.0f,
            static_cast<float>(ntris) / 1000.0f);

  //
  // Step 2. Rasterize input polygon soup.
  //

  // Allocate voxel heightfield where we rasterize our input data to.
  rcHeightfield *m_solid = rcAllocHeightfield();
  if (!m_solid) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
    return false;
  }
  if (!rcCreateHeightfield(pCtx, *m_solid, config.width, config.height, config.bmin, config.bmax, config.cs,
                           config.ch)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
    return false;
  }

  // Allocate array that can hold triangle area types.
  // If you have multiple meshes you need to process, allocate
  // and array which can hold the max number of triangles you need to process.
  auto *m_triareas = new(std::nothrow) unsigned char[ntris];
  if (!m_triareas) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
    return false;
  }

  // Find triangles which are walkable based on their slope and rasterize them.
  // If your input data is multiple meshes, you can transform them here, calculate
  // the are type for each of the meshes and rasterize them.
  std::memset(m_triareas, 0, ntris * sizeof(unsigned char));
  rcMarkWalkableTriangles(pCtx, config.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
  if (!rcRasterizeTriangles(pCtx, verts, nverts, tris, m_triareas, ntris, *m_solid, config.walkableClimb)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
    return false;
  }

  constexpr bool m_keepInterResults = false;
  if constexpr (!m_keepInterResults) {
    delete [] m_triareas;
  }

  //
  // Step 3. Filter walkable surfaces.
  //

  // Once all geometry is rasterized, we do initial pass of filtering to
  // remove unwanted overhangs caused by the conservative rasterization
  // as well as filter spans where the character cannot possibly stand.
  if (filterLowHangingObstacles)
    rcFilterLowHangingWalkableObstacles(pCtx, config.walkableClimb, *m_solid);
  if (filterLedgeSpans)
    rcFilterLedgeSpans(pCtx, config.walkableHeight, config.walkableClimb, *m_solid);
  if (filterWalkableLowHeightSpans)
    rcFilterWalkableLowHeightSpans(pCtx, config.walkableHeight, *m_solid);

  //
  // Step 4. Partition walkable surface to simple regions.
  //

  // Compact the heightfield so that it is faster to handle from now on.
  // This will result more cache coherent data as well as the neighbours
  // between walkable cells will be calculated.
  rcCompactHeightfield *m_chf = rcAllocCompactHeightfield();
  if (!m_chf) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
    return false;
  }
  if (!rcBuildCompactHeightfield(pCtx, config.walkableHeight, config.walkableClimb, *m_solid, *m_chf)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
    return false;
  }

  if constexpr (!m_keepInterResults) {
    rcFreeHeightField(m_solid);
  }

  // Erode the walkable area by agent radius.
  if (!rcErodeWalkableArea(pCtx, config.walkableRadius, *m_chf)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
    return false;
  }

  // (Optional) Mark areas.
  const ConvexVolume *vols = pGeom->getConvexVolumes();
  for (int i = 0; i < pGeom->getConvexVolumeCount(); ++i)
    rcMarkConvexPolyArea(pCtx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax,
                         static_cast<unsigned char>(vols[i].area), *m_chf);

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
  if (!rcBuildDistanceField(pCtx, *m_chf)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
    return false;
  }

  // Partition the walkable surface into simple regions without holes.
  if (!rcBuildRegionsWithSize(pCtx, *m_chf, 0, config.minRegionArea, config.mergeRegionArea)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
    return false;
  }

  //
  // Step 5. Trace and simplify region contours.
  //

  // Create contours.
  rcContourSet *m_cset = rcAllocContourSet();
  if (!m_cset) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
    return false;
  }
  if (!rcBuildContoursWithPortals(pCtx, *m_chf, config.maxSimplificationError, config.maxEdgeLen, *m_cset, bounderies, bounderyElementCount)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
    return false;
  }

  //
  // Step 6. Build polygons mesh from contours.
  //

  // Build polygon navmesh from the contours.
  m_pmesh = rcAllocPolyMesh();
  if (!m_pmesh) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
    return false;
  }
  if (!rcBuildPolyMesh(pCtx, *m_cset, config.maxVertsPerPoly, *m_pmesh)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
    return false;
  }

  //
  // Step 7. Create detail mesh which allows to access approximate height on each polygon.
  //

  m_dmesh = rcAllocPolyMeshDetail();
  if (!m_dmesh) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
    return false;
  }

  if (!rcBuildPolyMeshDetail(pCtx, *m_pmesh, *m_chf, config.detailSampleDist, config.detailSampleMaxError, *m_dmesh)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
    return false;
  }

  if constexpr (!m_keepInterResults) {
    rcFreeCompactHeightfield(m_chf);
    rcFreeContourSet(m_cset);
  }

  pCtx->stopTimer(RC_TIMER_TOTAL);

  // Show performance stats.
  duLogBuildTimes(*pCtx, pCtx->getAccumulatedTime(RC_TIMER_TOTAL));
  pCtx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pmesh->nverts, m_pmesh->npolys);

  totalBuildTimeMs = static_cast<float>(pCtx->getAccumulatedTime(RC_TIMER_TOTAL)) / 1000.0f;
  return true;
}

bool GenerateSingleMeshWaterShed(rcContext *pCtx, const InputGeom *pGeom, rcConfig &config,
                                 const bool filterLowHangingObstacles, const bool filterLedgeSpans,
                                 const bool filterWalkableLowHeightSpans, float &totalBuildTimeMs, rcPolyMesh *&m_pmesh,
                                 rcPolyMeshDetail *&m_dmesh) {
  if (!pGeom || !pGeom->getMesh()) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
    return false;
  }

  const float *bmin = pGeom->getNavMeshBoundsMin();
  const float *bmax = pGeom->getNavMeshBoundsMax();
  const float *verts = pGeom->getMesh()->getVerts();
  const int nverts = pGeom->getMesh()->getVertCount();
  const int *tris = pGeom->getMesh()->getTris();
  const int ntris = pGeom->getMesh()->getTriCount();

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
  pCtx->resetTimers();

  // Start the build process.
  pCtx->startTimer(RC_TIMER_TOTAL);

  pCtx->log(RC_LOG_PROGRESS, "Building navigation:");
  pCtx->log(RC_LOG_PROGRESS, " - %d x %d cells", config.width, config.height);
  pCtx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", static_cast<float>(nverts) / 1000.0f,
            static_cast<float>(ntris) / 1000.0f);

  //
  // Step 2. Rasterize input polygon soup.
  //

  // Allocate voxel heightfield where we rasterize our input data to.
  rcHeightfield *m_solid = rcAllocHeightfield();
  if (!m_solid) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
    return false;
  }
  if (!rcCreateHeightfield(pCtx, *m_solid, config.width, config.height, config.bmin, config.bmax, config.cs,
                           config.ch)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
    return false;
  }

  // Allocate array that can hold triangle area types.
  // If you have multiple meshes you need to process, allocate
  // and array which can hold the max number of triangles you need to process.
  auto *const m_triareas = new(std::nothrow) unsigned char[ntris];
  if (!m_triareas) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
    return false;
  }

  // Find triangles which are walkable based on their slope and rasterize them.
  // If your input data is multiple meshes, you can transform them here, calculate
  // the are type for each of the meshes and rasterize them.
  std::memset(m_triareas, 0, ntris * sizeof(unsigned char));
  rcMarkWalkableTriangles(pCtx, config.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
  if (!rcRasterizeTriangles(pCtx, verts, nverts, tris, m_triareas, ntris, *m_solid, config.walkableClimb)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
    return false;
  }

  constexpr bool m_keepInterResults = false;
  if constexpr (!m_keepInterResults) {
    delete [] m_triareas;
  }

  //
  // Step 3. Filter walkable surfaces.
  //

  // Once all geometry is rasterized, we do initial pass of filtering to
  // remove unwanted overhangs caused by the conservative rasterization
  // as well as filter spans where the character cannot possibly stand.
  if (filterLowHangingObstacles)
    rcFilterLowHangingWalkableObstacles(pCtx, config.walkableClimb, *m_solid);
  if (filterLedgeSpans)
    rcFilterLedgeSpans(pCtx, config.walkableHeight, config.walkableClimb, *m_solid);
  if (filterWalkableLowHeightSpans)
    rcFilterWalkableLowHeightSpans(pCtx, config.walkableHeight, *m_solid);

  //
  // Step 4. Partition walkable surface to simple regions.
  //

  // Compact the heightfield so that it is faster to handle from now on.
  // This will result more cache coherent data as well as the neighbours
  // between walkable cells will be calculated.
  rcCompactHeightfield *m_chf = rcAllocCompactHeightfield();
  if (!m_chf) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
    return false;
  }
  if (!rcBuildCompactHeightfield(pCtx, config.walkableHeight, config.walkableClimb, *m_solid, *m_chf)
  ) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
    return false;
  }

  if constexpr (!m_keepInterResults) {
    rcFreeHeightField(m_solid);
  }

  // Erode the walkable area by agent radius.
  if (!rcErodeWalkableArea(pCtx, config.walkableRadius, *m_chf)
  ) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
    return false;
  }

  // (Optional) Mark areas.
  const ConvexVolume *vols = pGeom->getConvexVolumes();
  for (int i = 0; i < pGeom->getConvexVolumeCount(); ++i)
    rcMarkConvexPolyArea(pCtx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax,
                         static_cast<unsigned char>(vols[i].area), *m_chf);

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
  if (!rcBuildDistanceField(pCtx, *m_chf)
  ) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
    return false;
  }

  // Partition the walkable surface into simple regions without holes.
  if (!rcBuildRegions(pCtx, *m_chf, 0, config.minRegionArea, config.mergeRegionArea)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
    return false;
  }

  //
  // Step 5. Trace and simplify region contours.
  //

  // Create contours.
  rcContourSet *m_cset = rcAllocContourSet();
  if (!m_cset) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
    return false;
  }
  if (!rcBuildContours(pCtx, *m_chf, config.maxSimplificationError, config.maxEdgeLen, *m_cset)
  ) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
    return false;
  }

  //
  // Step 6. Build polygons mesh from contours.
  //

  // Build polygon navmesh from the contours.
  m_pmesh = rcAllocPolyMesh();
  if (!m_pmesh) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
    return false;
  }
  if (!rcBuildPolyMesh(pCtx, *m_cset, config.maxVertsPerPoly, *m_pmesh)
  ) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
    return false;
  }

  //
  // Step 7. Create detail mesh which allows to access approximate height on each polygon.
  //

  m_dmesh = rcAllocPolyMeshDetail();
  if (!m_dmesh) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
    return false;
  }

  if (!rcBuildPolyMeshDetail(pCtx, *m_pmesh, *m_chf, config.detailSampleDist, config.detailSampleMaxError, *m_dmesh)) {
    pCtx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
    return false;
  }

  if constexpr (!m_keepInterResults) {
    rcFreeCompactHeightfield(m_chf);
    rcFreeContourSet(m_cset);
  }

  // At this point the navigation mesh data is ready, you can access it from m_pmesh.
  // See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.

  //
  // (Optional) Step 8. Create Detour data from Recast poly mesh.
  //

  pCtx->stopTimer(RC_TIMER_TOTAL);

  // Show performance stats.
  duLogBuildTimes(*pCtx, pCtx->getAccumulatedTime(RC_TIMER_TOTAL));
  pCtx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pmesh->nverts, m_pmesh->npolys);

  totalBuildTimeMs = static_cast<float>(pCtx->getAccumulatedTime(RC_TIMER_TOTAL)) / 1000.0f;

  return true;
}