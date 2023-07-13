# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]



## [1.6.0] - 2023-05-21

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


## [1.5.1] - 2016-02-22



[unreleased]: https://github.com/recastnavigation/recastnavigation/compare/1.6.0...HEAD
[1.6.0]: https://github.com/recastnavigation/recastnavigation/compare/1.5.1...1.6.0
[1.5.1]: https://github.com/recastnavigation/recastnavigation/compare/1.5.0...1.5.1
