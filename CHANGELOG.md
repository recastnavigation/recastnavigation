# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<h2>[Unreleased](https://github.com/recastnavigation/recastnavigation/compare/1.6.0...HEAD)</h2>
<h2>[1.6.0](https://github.com/recastnavigation/recastnavigation/compare/1.5.1...1.6.0) - 2023-05-21</h2>

### Added
- CMake build support
- Unit testing with Catch2 (#147)
- Support for AABB and OBB obstacles in `dtTileCache` (#215, #278)
- `dtTileCache` supports timesliced updates (#203)
- Support for custom assertion functions (#250)
- Variant of `findNearestPoly` that exposes distance and isOverPoly (#448)
- `dtNavMeshQuery::getPathFromDijkstraSearch` gets a path from the explored nodes in a navmesh search (#211)
- A version of `dtPolyQuery::queryPolygon` that operates on batches of polygons rather than just 128 (#175) (Fixes #107)
- `rcNew`/`rcDelete` to match `rcAlloc`/`rcFree` (#324)
- Better error reporting and input sanitization (#179, #303)
- Better debug draw (#253, #254, #255, #256)
- Improved docstrings, documentation
- (RecastDemo) Load/Save navmesh data (#258)

### Fixed
- Improved robustness, speed and accuracy of navmesh point queries (#205, #208, #228, #231, #364, #381, #560)
- Incorrect rasterization at tile borders (#476)
- Off-mesh links in tiles were sometimes added twice (#202)
- Potential heap corruption when collecting region layers (#214)
- `findPath` returns `DT_OUT_OF_NODES` appropriately (#222)
- Spans are filtered if there is just enough height (#626)
- Increased epsilon in detour common segment polygon intersection test (#612)
- Array overrun in `removeVertex` in `DetourTileCacheBuilder` (#601)
- Potential rounding error computing bounding box size in `dtNavMesh::connectExtLinks` (#428)
- An indexing error in updating agents in `DetourCrowd` (#450)
- Allocation perf issues in rcVectorBase (#467)
- Dead website links in comments
- RecastDemo bugs (#180, #184, #186, #187, #200)
- Uninitialized class member values, small memory leaks, rule-of-three violations, other minor issues

### Changed
- Updated stb_image (#184)
- Updated stb_truetype (#183)

### Removed
- Use of _USE_MATH_DEFINES directive (#596)

## [1.5.1](https://github.com/recastnavigation/recastnavigation/compare/1.5.0...1.5.1) - 2016-02-22

Patch release; one bug has been fixed, which would cause silent failure if too many nodes were requested and used in a dtNavMeshQuery.

- Fail when too many nodes are requested (#179)

## 1.5.0 - 2016-01-24

This is the first release of the Recast and Detour libraries since August 2009, containing all fixes and enhancements made since then. As you can imagine, this includes a huge number of commits, so we will forego the list of changes for this release - future releases will contain at least a summary of changes.

We have decided to use Semantic Versioning for version numbers from now onwards - beginning at 1.5.0 rather than 1.0.0 since the last old release on Google Code was 1.4.

## 1.4.0 - 2009-08-24

(Release 1.4 and earlier can be found on the old [archived google code repository](https://code.google.com/archive/p/recastnavigation/))

- Added detail height mesh generation (RecastDetailMesh.cpp) for single, tiled statmeshes as well as tilemesh.
- Added feature to contour tracing which detects extra vertices along tile edges which should be removed later.
- Changed the tiled stat mesh preprocess, so that it first generated polymeshes per tile and finally combines them.
- Fixed bug in the GUI code where invisible buttons could be pressed.

## 1.3.1 - 2009-07-24

- Better cost and heuristic functions.
- Fixed tile navmesh raycast on tile borders.

## 1.3.1 - 2009-07-14

- Added dtTileNavMesh which allows dynamically adding and removing navmesh pieces at runtime.
- Renamed stat navmesh types to dtStat* (i.e. dtPoly is now dtStatPoly).
- Moved common code used by tile and stat navmesh to DetourNode.h/cpp and DetourCommon.h/cpp.
- Refactor the demo code.

## 1.2.0 - 2009-06-17

- Added tiled mesh generation. The tiled generation allows to generate navigation for much larger worlds, it removes some of the artifacts that comes from distance fields in open areas, and allows later streaming and dynamic runtime generation
- Improved and added some debug draw modes
- API change: The helper function rcBuildNavMesh does not exists anymore, had to change few internal things to cope with the tiled processing, similar API functionality will be added later once the tiled process matures
- The demo is getting way too complicated, need to split demos
- Fixed several filtering functions so that the mesh is tighter to the geometry, sometimes there could be up error up to tow voxel units close to walls, now it should be just one.

## 1.1.0 - 2009-04-11

This is the first release of Detour.

## 1.0.0 - 2009-03-29

This is the first release of Recast.

The process is not always as robust as I would wish. The watershed phase sometimes swallows tiny islands which are close to edges. These droppings are handled in rcBuildContours, but the code is not particularly robust either.

Another non-robust case is when portal contours (contours shared between two regions) are always assumed to be straight. That can lead to overlapping contours specially when the level has large open areas.
